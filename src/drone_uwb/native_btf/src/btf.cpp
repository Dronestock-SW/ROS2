#include "btf/core.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace btf {
namespace {
Time us(double s) {return static_cast<Time>(std::llround(s*1e6));}
double distance(const XY& a,const XY& b) {return std::hypot(a[0]-b[0],a[1]-b[1]);}
double norm(const XY& a) {return std::hypot(a[0],a[1]);}
bool finite(const XY& a) {return std::isfinite(a[0])&&std::isfinite(a[1]);}
bool measured(const Frame& f) {
  return f.height_source==HeightSource::MeasuredTofImu &&
      std::all_of(f.height_m.begin(),f.height_m.end(),[](auto h){return h&&std::isfinite(*h);});
}
struct Point {Time t; XY xy;};
struct LineFit {XY velocity; double error;};
LineFit line_fit(const std::deque<Point>& p) {
  Eigen::MatrixXd a(p.size(),2), xy(p.size(),2);
  for (std::size_t i=0;i<p.size();++i) {
    a.row(i) << (p[i].t-p.back().t)/1e6,1.; xy.row(i) << p[i].xy[0],p[i].xy[1];
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(a,Eigen::ComputeThinU|Eigen::ComputeThinV);
  svd.setThreshold(std::numeric_limits<double>::epsilon()*std::max<std::size_t>(p.size(),2));
  const Eigen::MatrixXd c=svd.solve(xy), error=a*c-xy;
  return {{c(0,0),c(0,1)},error.rowwise().norm().maxCoeff()};
}
// Requalifies measured candidates only; predictions never leave this checker.
struct Motion {
  std::deque<Point> history, pending;
  std::optional<LineFit> model;
  std::optional<Time> last_check;
  bool blocked=false;
  int key=0;
  void confirmation_clear(){pending.clear();key=0;}
  void clear(){*this=Motion{};}
  void freeze(){blocked=true;confirmation_clear();}
  void observe(Time t,XY xy) {
    if(!history.empty()&&(t<=history.back().t||t-history.back().t>100000)) clear();
    history.push_back({t,xy});
    while(t-history.front().t>500000)history.pop_front();
    model.reset();blocked=false;last_check.reset();confirmation_clear();
  }
  bool check(Time t,XY xy,bool eligible,int candidate_key) {
    if(t<=0||!finite(xy)||(last_check&&t<=*last_check)){freeze();return false;}
    last_check=t;
    if(blocked||!eligible){confirmation_clear();return false;}
    if(!model) {
      if(history.size()<8||history.back().t-history.front().t<300000) {
        confirmation_clear();return false;
      }
      auto fit=line_fit(history); const auto speed=norm(fit.velocity);
      const auto middle=history.size()/2;
      const auto first=line_fit(std::deque<Point>(history.begin(),history.begin()+middle+1));
      const auto last=line_fit(std::deque<Point>(history.begin()+middle,history.end()));
      if(speed<.15||speed>2.||fit.error>.04||distance(first.velocity,last.velocity)>.25) {
        confirmation_clear();return false;
      }
      model=fit;
    }
    const auto age=(t-history.back().t)/1e6;
    if(age<=0||age>1.5){freeze();return false;}
    const auto v=model->velocity; const auto speed=norm(v);
    const XY prediction{history.back().xy[0]+v[0]*age,history.back().xy[1]+v[1]*age};
    if(distance(xy,prediction)>std::min(.20,.10+.10*age)){freeze();return false;}
    if(!pending.empty()) {
      const auto dt=(t-pending.back().t)/1e6;
      if(dt>.1||key!=candidate_key)confirmation_clear();
      else if(distance(xy,pending.back().xy)>.05+(speed+.25)*dt){freeze();return false;}
    }
    key=candidate_key;pending.push_back({t,xy});
    if(t-pending.front().t<150000||pending.size()<4)return false;
    const auto fit=line_fit(pending);
    const double along=(fit.velocity[0]*v[0]+fit.velocity[1]*v[1])/(speed*speed);
    if(fit.error>.04||distance(fit.velocity,v)>.25||along<.5||along>1.5){freeze();return false;}
    return true;
  }
};
struct Confirmation {
  std::optional<Time> first,last;
  std::optional<XY> origin,last_xy;
  int anchor=0,count=0;
  void clear(){*this=Confirmation{};}
  bool check(Time t,XY xy,int a,Time max_gap,double radius,double speed,
             bool compare_origin,Time span,int samples) {
    if(last&&(t<=*last||t-*last>max_gap||a!=anchor||
         distance(xy,compare_origin?*origin:*last_xy)>radius+speed*(t-*last)/1e6))clear();
    if(!first){first=t;origin=xy;anchor=a;}
    last=t;last_xy=xy;++count;
    return t-*first>=span&&count>=samples;
  }
};
}  // namespace

struct Processor::Impl {
  explicit Impl(Config c):cfg(std::move(c)){cfg.validate();}
  Config cfg;
  std::deque<RangeSample> ranges;
  std::deque<Frame> frames;
  std::optional<Time> last_end,last_observed;
  std::array<std::optional<Time>,4> last_sample{};
  std::optional<XY> prior,recovery_reference;
  std::optional<Time> prior_stamp;
  int selected=0;
  bool b4_blocked=false;
  Confirmation b4_confirm,near,ground;
  Motion motion;

  bool b4_integrity(const Frame& f,Decision& d) {
    d.b4_height_blocked=b4_blocked;
    if(!measured(f)) {
      if(b4_confirm.last&&(f.end_us<=*b4_confirm.last||f.end_us-*b4_confirm.last>100000))b4_confirm.clear();
      d.b4_height_reason="measured_height_unavailable";return !b4_blocked;
    }
    double sum=0;
    for(int i=0;i<4;++i) {
      if(f.end_us-f.sample_us[i]>us(cfg.solver.window_s)) {
        b4_blocked=true;b4_confirm.clear();d.b4_height_blocked=true;
        d.b4_height_reason="invalid_b4_physical_inputs";return false;
      }
      const double dt=(f.sample_us[i]-f.end_us)/1e6;
      const double x=(*d.b4.xy)[0]+(*d.b4.velocity)[0]*dt;
      const double y=(*d.b4.xy)[1]+(*d.b4.velocity)[1]*dt;
      const auto& a=cfg.anchors[i];
      const double error=std::hypot(std::hypot(x-a[0],y-a[1]),*f.height_m[i]-a[2])-f.raw_range_m[i];
      sum+=error*error;
    }
    d.b4_physical_range_rms_m=std::sqrt(sum/4);
    if(*d.b4_physical_range_rms_m>=cfg.rms_limit_m) {
      b4_blocked=true;b4_confirm.clear();d.b4_height_blocked=true;
      d.b4_height_reason="b4_height_inconsistent";return false;
    }
    if(b4_blocked) {
      if(!b4_confirm.check(f.end_us,{0.,0.},0,100000,1.,0.,false,150000,4)) {
        d.b4_height_reason="b4_height_recovery_pending";return false;
      }
      b4_blocked=false;b4_confirm.clear();
    }
    d.b4_height_blocked=false;d.b4_height_reason="b4_height_consistent";return true;
  }

  Decision candidate(const Frame& f,Decision d) {
    // Baseline parity: append BEFORE acceptance, as in Python. History admission
    // improvement is deliberately not claimed by a language-only translation.
    frames.push_back(f);
    while(!frames.empty()&&frames.front().end_us<f.end_us-us(cfg.solver.window_s))frames.pop_front();
    if(!d.b4.ok)return d;
    if(*d.b4.rms_m<cfg.rms_limit_m) {
      const bool allowed=b4_integrity(f,d);
      if(allowed||!cfg.enforce_b4_height_integrity) {
        d.ok=true;d.xy=d.b4.xy;d.source="unchanged_B4";d.reason="original_fit_consistent";return d;
      }
      if(d.b4_height_reason!="b4_height_inconsistent") {
        d.reason=d.b4_height_reason;return d;
      }
    }
    const auto newest=std::max_element(f.sample_us.begin(),f.sample_us.end())-f.sample_us.begin();
    if(f.height_source==HeightSource::Missing||!f.height_m[newest]) {
      d.reason="measured_height_unavailable";return d;
    }
    Settings subset=cfg.solver;subset.required_anchor_count=3;
    for(int excluded=0;excluded<4;++excluded) {
      std::vector<RangeSample> input;
      for(const auto& row:frames)for(int i=0;i<4;++i)
        if(i!=excluded&&row.sample_us[i]>=f.end_us-us(cfg.solver.window_s))
          input.push_back({row.sample_us[i],i,row.raw_range_m[i]});
      Hypothesis h;h.excluded_anchor=excluded+1;h.fit=fit_h80(cfg.anchors,input,f.end_us,subset);
      if(h.fit.ok) {
        double sum=0;int count=0;std::array<int,4> height_counts{};
        for(const auto& row:frames)for(int i=0;i<4;++i) {
          if(i==excluded||!row.height_m[i]||row.sample_us[i]<f.end_us-us(cfg.solver.window_s))continue;
          const auto dt=(row.sample_us[i]-f.end_us)/1e6;
          const auto x=(*h.fit.xy)[0]+(*h.fit.velocity)[0]*dt;
          const auto y=(*h.fit.xy)[1]+(*h.fit.velocity)[1]*dt;
          const auto& a=cfg.anchors[i];
          const auto error=std::hypot(std::hypot(x-a[0],y-a[1]),*row.height_m[i]-a[2])-row.raw_range_m[i];
          sum+=error*error;++count;++height_counts[i];
        }
        bool enough=true;
        for(int i=0;i<4;++i)if(i!=excluded&&height_counts[i]<cfg.solver.min_samples_per_anchor)enough=false;
        if(enough)h.physical_range_rms_m=std::sqrt(sum/count);
      }
      d.hypotheses.push_back(std::move(h));
    }
    std::vector<const Hypothesis*> eligible;
    for(const auto& h:d.hypotheses)if(h.physical_range_rms_m)eligible.push_back(&h);
    std::stable_sort(eligible.begin(),eligible.end(),[](auto a,auto b){return *a->physical_range_rms_m<*b->physical_range_rms_m;});
    if(eligible.size()<2){d.reason="insufficient_subset_hypotheses";return d;}
    const auto& best=*eligible[0];d.best_excluded_anchor=best.excluded_anchor;
    d.best_score_m=best.physical_range_rms_m;
    d.score_margin_m=*eligible[1]->physical_range_rms_m-*d.best_score_m;
    if(*d.best_score_m>=cfg.rms_limit_m){d.reason="no_height_consistent_subset";return d;}
    if(*d.score_margin_m<cfg.score_margin_m){d.reason="ambiguous_subset";return d;}
    d.ok=true;d.xy=best.fit.xy;d.reason="unique_height_consistent_subset";
    d.source=f.height_source==HeightSource::GroundReference?"ground_reference_B3":"ToF_validated_B3";
    return d;
  }

  Decision track(const Frame& f,Decision d) {
    if(prior_stamp)d.tracking_prior_age_s=(f.end_us-*prior_stamp)/1e6;
    const bool fresh=d.tracking_prior_age_s&&*d.tracking_prior_age_s>0&&*d.tracking_prior_age_s<=cfg.max_prior_age_s;
    bool initial=false,reacquired=false,moving=false;
    const bool unique=d.ok&&d.reason=="unique_height_consistent_subset";
    if(cfg.ground_subset_initialization) {
      const bool eligible=f.height_source==HeightSource::GroundReference&&!fresh&&unique&&
          (!prior||(d.xy&&distance(*d.xy,*prior)<=.1));
      if(eligible)initial=ground.check(f.end_us,*d.xy,d.best_excluded_anchor,100000,.05,0.,true,2000000,30);
      else ground.clear();
    }
    if(!fresh&&measured(f)&&unique&&d.source=="ToF_validated_B3"&&recovery_reference&&
       distance(*d.xy,*recovery_reference)<=cfg.continuity_margin_m)
      reacquired=near.check(f.end_us,*d.xy,d.best_excluded_anchor,us(cfg.max_prior_age_s),
                           cfg.continuity_margin_m,cfg.max_speed_m_s,false,150000,4);
    else near.clear();
    if(!fresh)moving=motion.check(f.end_us,d.xy.value_or(XY{0,0}),
        measured(f)&&unique&&d.source=="ToF_validated_B3",d.best_excluded_anchor);
    if(selected&&d.reason=="ambiguous_subset"&&fresh) {
      for(const auto& h:d.hypotheses)if(h.excluded_anchor==selected&&h.physical_range_rms_m) {
        d.tracking_step_m=distance(*h.fit.xy,*prior);
        d.tracking_limit_m=cfg.continuity_margin_m+cfg.max_speed_m_s**d.tracking_prior_age_s;
        if(*h.physical_range_rms_m<cfg.rms_limit_m&&*d.tracking_step_m<=*d.tracking_limit_m) {
          d.ok=true;d.xy=h.fit.xy;d.best_excluded_anchor=selected;
          d.source=f.height_source==HeightSource::GroundReference?"ground_reference_tracked_B3":"ToF_tracked_B3";
          d.reason="previously_unique_subset_still_consistent";
        }
      }
    }
    if(d.ok&&d.source!="unchanged_B4") {
      if(!fresh&&!initial&&!reacquired&&!moving)reject(d,"no_fresh_continuity_prior");
      else if(reacquired)d.reason="near_prior_subset_reacquired";
      else if(moving)d.reason="moving_subset_reacquired";
      else if(fresh) {
        d.tracking_step_m=distance(*d.xy,*prior);
        d.tracking_limit_m=cfg.continuity_margin_m+cfg.max_speed_m_s**d.tracking_prior_age_s;
        if(*d.tracking_step_m>*d.tracking_limit_m){motion.freeze();reject(d,"subset_position_discontinuous");}
      }
    }
    if(d.ok) {
      selected=d.source=="unchanged_B4"?0:d.best_excluded_anchor;
      prior=d.xy;prior_stamp=f.end_us;recovery_reference=d.xy;near.clear();
      if(measured(f))motion.observe(f.end_us,*d.xy);else motion.clear();
    }
    if(prior_stamp&&(f.end_us-*prior_stamp)/1e6>cfg.max_prior_age_s)selected=0;
    return d;
  }
  static void reject(Decision& d,const std::string& reason) {
    d.ok=false;d.xy.reset();d.source="rejected";d.reason=reason;
  }
  Decision process(Frame f) {
    Decision d;d.source_stamp_us=f.end_us;
    // This boundary accepts only already validated four-anchor cycles.
    if(f.start_us<0||f.end_us<=0||f.end_us>9000000000000000LL||f.end_us<f.start_us||
       f.end_us!=*std::max_element(f.sample_us.begin(),f.sample_us.end())) {
      d.reason="invalid_anchor_times";return d;
    }
    for(int i=0;i<4;++i) {
      f.raw_range_m[i]-=cfg.bias_m[i];
      if(!std::isfinite(f.raw_range_m[i])||f.raw_range_m[i]<=0||
         f.raw_range_m[i]>std::min(80.,cfg.solver.max_range_m)) {
        d.reason="four_valid_ranges_required";return d;
      }
      if(f.sample_us[i]<f.start_us||f.sample_us[i]>f.end_us) {d.reason="invalid_anchor_times";return d;}
      if(f.height_m[i]&&!std::isfinite(*f.height_m[i])) {d.reason="invalid_height";return d;}
    }
    if(last_end&&(f.end_us<=*last_end||f.end_us-*last_end>us(cfg.solver.reset_gap_s))) {
      const bool backwards=f.end_us<=*last_end;
      d.reset_reason=backwards?"source_time_backwards":"gap";
      ranges.clear();frames.clear();last_sample={};last_observed.reset();
      selected=0;prior.reset();prior_stamp.reset();near.clear();
      if(backwards){recovery_reference.reset();motion.clear();}
    }
    last_end=f.end_us;
    std::vector<RangeSample> fresh;
    for(int i=0;i<4;++i)if(!last_sample[i]||f.sample_us[i]>*last_sample[i]) {
      fresh.push_back({f.sample_us[i],i,f.raw_range_m[i]});last_sample[i]=f.sample_us[i];
    }
    if(!fresh.empty()&&last_observed) {
      const auto oldest=std::min_element(fresh.begin(),fresh.end(),[](auto a,auto b){return a.stamp_us<b.stamp_us;})->stamp_us;
      if(oldest-*last_observed>us(cfg.solver.reset_gap_s)) {
        ranges.clear();last_sample={};d.reset_reason="observation_gap";
        for(const auto& o:fresh)last_sample[o.anchor]=o.stamp_us;
      }
    }
    if(!fresh.empty())last_observed=std::max_element(fresh.begin(),fresh.end(),[](auto a,auto b){return a.stamp_us<b.stamp_us;})->stamp_us;
    for(const auto& o:fresh)ranges.push_back(o);
    const Time lower=f.end_us-us(cfg.solver.window_s);
    ranges.erase(std::remove_if(ranges.begin(),ranges.end(),[&](auto o){return o.stamp_us<lower||o.stamp_us>f.end_us;}),ranges.end());
    d.history_samples=ranges.size();
    d.fresh_observation_count=static_cast<int>(fresh.size());
    for(const auto& r:ranges)
      if(!d.newest_sample_us||r.stamp_us>*d.newest_sample_us)d.newest_sample_us=r.stamp_us;
    if(fresh.empty())d.b4.reason="no_fresh_observations";
    else d.b4=fit_h80(cfg.anchors,{ranges.begin(),ranges.end()},f.end_us,cfg.solver);
    d=candidate(f,std::move(d));
    return track(f,std::move(d));
  }
};
Processor::Processor(Config c):impl_(std::make_unique<Impl>(std::move(c))){}
Processor::~Processor()=default;
Processor::Processor(Processor&&) noexcept=default;
Processor& Processor::operator=(Processor&&) noexcept=default;
Decision Processor::process(const Frame& f){return impl_->process(f);}
void Processor::reset(){impl_=std::make_unique<Impl>(impl_->cfg);}
void Processor::reset_models(bool preserve_motion,bool preserve_height_conflict) {
  auto next=std::make_unique<Impl>(impl_->cfg);
  if(preserve_height_conflict)next->b4_blocked=impl_->b4_blocked;
  if(preserve_motion) {
    next->recovery_reference=impl_->recovery_reference;
    next->motion=impl_->motion;
    next->motion.confirmation_clear();
  }
  impl_=std::move(next);
}
}  // namespace btf
