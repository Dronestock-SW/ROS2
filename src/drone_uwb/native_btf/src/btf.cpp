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
    // Baseline candidate uses the original long history. Optional fresh B4
    // recovery replaces that history only after qualifying new measured ranges.
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
    const bool b4_source=d.source=="unchanged_B4"||d.source=="fresh_B4_recovery";
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
    if(d.ok&&!b4_source) {
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
      selected=b4_source?0:d.best_excluded_anchor;
      prior=d.xy;prior_stamp=f.end_us;recovery_reference=d.xy;near.clear();
      if(measured(f))motion.observe(f.end_us,*d.xy);else motion.clear();
    }
    if(prior_stamp&&(f.end_us-*prior_stamp)/1e6>cfg.max_prior_age_s)selected=0;
    return d;
  }
  static void reject(Decision& d,const std::string& reason) {
    d.ok=false;d.xy.reset();d.source="rejected";d.reason=reason;
  }
  void recover_fresh_b4(const Frame& f,Decision& d) {
    // Alternative fit of NEW measured ranges, never a frozen/extrapolated pose.
    // Preserve the baseline decision whenever it already has a valid candidate.
    if(!cfg.fresh_b4_recovery_enabled||d.ok||d.fresh_observation_count!=4||!prior_stamp||!prior||
       !d.b4.ok||!d.b4.rms_m||*d.b4.rms_m<cfg.rms_limit_m)return;
    const double age=(f.end_us-*prior_stamp)/1e6;
    if(age<=0||age>cfg.solver.window_s)return;
    std::vector<RangeSample> clean;int count=0;Time previous=f.end_us;
    for(auto it=frames.rbegin();it!=frames.rend();++it) {
      if(f.end_us-it->end_us>us(cfg.solver.reset_gap_s)||previous-it->end_us>100000)break;
      previous=it->end_us;++count;
      for(int i=0;i<4;++i)clean.push_back({it->sample_us[i],i,it->raw_range_m[i]});
    }
    if(count<cfg.solver.min_samples_per_anchor)return;
    const auto fit=fit_h80(cfg.anchors,clean,f.end_us,cfg.solver);
    if(!fit.ok||!fit.rms_m||*fit.rms_m>=cfg.rms_limit_m||!fit.xy)return;
    if(!fit.velocity||norm(*fit.velocity)>cfg.max_speed_m_s||
       distance(*fit.xy,*prior)>cfg.continuity_margin_m+cfg.max_speed_m_s*age)return;
    // Check the newest four *sequential* measurements at their own times.
    // An instantaneous XY pair residual can misclassify actual motion.
    double current_error=0.;
    for(int i=0;i<4;++i) {
      const double u=(f.sample_us[i]-f.end_us)/1e6/cfg.solver.window_s;
      const auto& c=fit.coefficients;const auto& a=cfg.anchors[i];
      const double rr=c[4]+c[5]*u+c[6]*u*u-2*a[0]*(c[0]+c[2]*u)-2*a[1]*(c[1]+c[3]*u)+a[0]*a[0]+a[1]*a[1];
      if(!std::isfinite(rr)||rr<=0.)return;
      const auto error=f.raw_range_m[i]-std::sqrt(rr);current_error+=error*error;
    }
    d.recovery_raw_residual_m=std::sqrt(current_error/4.);
    if(*d.recovery_raw_residual_m>=cfg.rms_limit_m)return;
    Decision recovered=d;recovered.b4=fit;
    if(!b4_integrity(f,recovered)&&cfg.enforce_b4_height_integrity)return;
    recovered.ok=true;recovered.xy=fit.xy;recovered.source="fresh_B4_recovery";
    recovered.reason="fresh_four_anchor_history_recovered";recovered.recovery_frames=count;
    // Drop the contaminated long history only after the replacement passes.
    ranges.assign(clean.begin(),clean.end());
    while(frames.size()>static_cast<std::size_t>(count))frames.pop_front();
    recovered.history_samples=ranges.size();d=std::move(recovered);
  }
  Decision partial(Frame f) {
    Decision d;d.source_stamp_us=f.end_us;
    std::vector<int> ids;
    for(int i=0;i<4;++i)if(f.valid_mask&(1<<i))ids.push_back(i);
    if(f.valid_mask<0||f.valid_mask>15||ids.size()!=3) {d.reason="three_measured_ranges_required";return d;}
    Time newest=0,oldest=f.end_us;
    for(int i:ids) {
      f.raw_range_m[i]-=cfg.bias_m[i];
      if(!std::isfinite(f.raw_range_m[i])||f.raw_range_m[i]<=0||
         f.raw_range_m[i]>std::min(80.,cfg.solver.max_range_m)||f.sample_us[i]<f.start_us||
         f.sample_us[i]>f.end_us) {d.reason="invalid_partial_ranges";return d;}
      newest=std::max(newest,f.sample_us[i]);oldest=std::min(oldest,f.sample_us[i]);
    }
    if(f.start_us<0||f.end_us<=0||f.end_us>9000000000000000LL||newest!=f.end_us||
       newest-oldest>us(cfg.solver.max_extrapolation_s)||
       (last_end&&f.end_us<=*last_end)) {d.reason="invalid_anchor_times";return d;}
    last_end=f.end_us;d.fresh_observation_count=3;d.newest_sample_us=newest;
    if(f.height_source!=HeightSource::MeasuredTofImu) {d.reason="partial_measured_height_required";return d;}
    for(int i:ids)if(!f.height_m[i]||!std::isfinite(*f.height_m[i])) {
      d.reason="partial_measured_height_required";return d;
    }
    if(!prior_stamp||!prior) {d.reason="partial_fresh_prior_required";return d;}
    const double age=(f.end_us-*prior_stamp)/1e6;
    d.tracking_prior_age_s=age;
    if(age<=0||age>cfg.max_prior_age_s) {d.reason="partial_fresh_prior_required";return d;}
    // Three current, non-collinear ranges plus independently measured height.
    // Missing anchors never receive copied or synthesized distances.
    const auto horizontal_square=[&](int i) {
      const double z=*f.height_m[i]-cfg.anchors[i][2];
      return f.raw_range_m[i]*f.raw_range_m[i]-z*z;
    };
    const int a=ids[0],b=ids[1],c=ids[2];
    for(int i:ids)if(horizontal_square(i)<0) {d.reason="partial_height_inconsistent";return d;}
    const auto rhs=[&](int i) {
      return horizontal_square(a)-horizontal_square(i)
        +cfg.anchors[i][0]*cfg.anchors[i][0]+cfg.anchors[i][1]*cfg.anchors[i][1]
        -cfg.anchors[a][0]*cfg.anchors[a][0]-cfg.anchors[a][1]*cfg.anchors[a][1];
    };
    const double bx=2*(cfg.anchors[b][0]-cfg.anchors[a][0]),by=2*(cfg.anchors[b][1]-cfg.anchors[a][1]);
    const double cx=2*(cfg.anchors[c][0]-cfg.anchors[a][0]),cy=2*(cfg.anchors[c][1]-cfg.anchors[a][1]);
    const double det=bx*cy-by*cx;
    if(std::abs(det)<1e-6) {d.reason="partial_degenerate_geometry";return d;}
    XY xy{(rhs(b)*cy-by*rhs(c))/det,(bx*rhs(c)-rhs(b)*cx)/det};
    // Refine the squared-range initializer against physical range residuals.
    // Fixed measured heights leave two unknowns and three current equations.
    // Bounded Gauss-Newton steps do not add ranges or change their timestamps.
    for(int iteration=0;iteration<8;++iteration) {
      Eigen::Matrix2d normal=Eigen::Matrix2d::Zero();
      Eigen::Vector2d gradient=Eigen::Vector2d::Zero();
      for(int i:ids) {
        const double dx=xy[0]-cfg.anchors[i][0],dy=xy[1]-cfg.anchors[i][1];
        const double predicted=std::hypot(std::hypot(dx,dy),*f.height_m[i]-cfg.anchors[i][2]);
        if(predicted<1e-6) {d.reason="partial_degenerate_geometry";return d;}
        Eigen::Vector2d jac(dx/predicted,dy/predicted);
        normal+=jac*jac.transpose();gradient+=jac*(predicted-f.raw_range_m[i]);
      }
      if(normal.determinant()<1e-8) {d.reason="partial_degenerate_geometry";return d;}
      const Eigen::Vector2d step=normal.ldlt().solve(gradient);
      if(!step.allFinite()||step.norm()>1.) {d.reason="partial_fit_unstable";return d;}
      xy[0]-=step[0];xy[1]-=step[1];
      if(step.norm()<1e-6)break;
    }
    double sum=0;
    for(int i:ids) {
      const double error=std::hypot(std::hypot(xy[0]-cfg.anchors[i][0],xy[1]-cfg.anchors[i][1]),
                                    *f.height_m[i]-cfg.anchors[i][2])-f.raw_range_m[i];
      sum+=error*error;
    }
    d.best_score_m=std::sqrt(sum/3.);
    d.tracking_step_m=distance(xy,*prior);
    d.tracking_limit_m=cfg.continuity_margin_m+cfg.max_speed_m_s*age;
    if(!std::isfinite(*d.best_score_m)||*d.best_score_m>=cfg.rms_limit_m) {
      d.reason="partial_height_inconsistent";return d;
    }
    if(*d.tracking_step_m>*d.tracking_limit_m) {d.reason="partial_continuity_rejected";return d;}
    d.ok=true;d.xy=xy;d.source="ToF_validated_partial_B3";d.reason="partial_three_anchor_consistent";
    prior=xy;prior_stamp=f.end_us;recovery_reference=xy;
    return d;
  }

  Decision process(Frame f) {
    if(f.valid_mask!=15)return partial(std::move(f));
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
    recover_fresh_b4(f,d);
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
