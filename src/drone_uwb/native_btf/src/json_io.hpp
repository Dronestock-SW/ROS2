#pragma once
#include "btf/core.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>
namespace btf::json_io {
using Json=nlohmann::json;
template<class T> Json nullable(const std::optional<T>& x){return x?Json(*x):Json(nullptr);}
btf::Time timestamp(const Json& j) {
  if(!j.is_number_integer()||j.get<double>()<0||j.get<double>()>9e15)
    throw std::invalid_argument("invalid_timestamp_us");
  return j.get<btf::Time>();
}
template<class T,std::size_t N> std::array<T,N> array(const Json& j) {
  if(!j.is_array()||j.size()!=N)throw std::invalid_argument("invalid_array_size");
  return j.get<std::array<T,N>>();
}
btf::Config config(const Json& j) {
  btf::Config c;c.anchors=array<std::array<double,3>,4>(j.at("anchors_xyz_m"));
  for(const auto& a:j.at("anchors_xyz_m"))if(a.size()!=3)throw std::invalid_argument("invalid_anchor_dimensions");
  c.bias_m=array<double,4>(j.at("bias_m"));
  const auto b=j.value("B",Json::object());
#define BSETTING(field) c.solver.field=b.value(#field,c.solver.field)
  BSETTING(window_s);BSETTING(sigma_r_m);BSETTING(huber_m);BSETTING(max_scaled_condition);
  BSETTING(reset_gap_s);BSETTING(max_extrapolation_s);BSETTING(max_range_m);
  BSETTING(min_samples_per_anchor);BSETTING(min_samples_total);BSETTING(required_anchor_count);BSETTING(h80_irls_iterations);
#undef BSETTING
  const auto s=j.value("tof_subset",Json::object());
#define SUBSETTING(field) c.field=s.value(#field,c.field)
  SUBSETTING(rms_limit_m);SUBSETTING(score_margin_m);SUBSETTING(max_speed_m_s);
  SUBSETTING(continuity_margin_m);SUBSETTING(max_prior_age_s);
#undef SUBSETTING
  c.enforce_b4_height_integrity=j.value("enforce_b4_height_integrity",false);
  c.ground_subset_initialization=j.value("ground_subset_initialization",false);
  c.validate();return c;
}
btf::Frame frame(const Json& j) {
  btf::Frame f;f.start_us=timestamp(j.at("cycle_start_us"));f.end_us=timestamp(j.at("time_us"));
  const auto& t=j.at("sample_time_us");
  if(!t.is_array()||t.size()!=4)throw std::invalid_argument("four_anchor_times_required");
  for(int i=0;i<4;++i)f.sample_us[i]=timestamp(t[i]);
  f.raw_range_m=array<double,4>(j.at("raw_slant_m"));
  if(j.contains("sample_height_m")) {
    const auto& h=j.at("sample_height_m");
    if(!h.is_array()||h.size()!=4)throw std::invalid_argument("four_heights_required");
    for(int i=0;i<4;++i)if(!h[i].is_null())f.height_m[i]=h[i].get<double>();
  }
  const auto source=j.value("height_source",std::string("missing"));
  if(source=="measured_tof_imu"||source=="gazebo_tof_imu")f.height_source=btf::HeightSource::MeasuredTofImu;
  else if(source=="ground_antenna_reference")f.height_source=btf::HeightSource::GroundReference;
  else if(source!="missing")throw std::invalid_argument("invalid_height_source");
  return f;
}
Json fit(const btf::Fit& f) {
  Json j={{"ok",f.ok},{"reason",f.reason},{"xy_m",nullable(f.xy)},
    {"velocity_m_s",nullable(f.velocity)},{"rms_m",nullable(f.rms_m)},
    {"max_m",nullable(f.max_m)},{"condition",nullable(f.condition)},
    {"n",f.n},{"mask",f.mask},{"counts",f.counts},{"rank",f.rank},
    {"iterations",f.iterations},{"converged",f.converged}};
  if(f.ok)j["coefficients"]=f.coefficients;
  return j;
}
Json decision(const btf::Decision& d) {
  Json hs=Json::array();
  for(const auto& h:d.hypotheses)hs.push_back({{"excluded_anchor",h.excluded_anchor},
    {"fit",fit(h.fit)},{"physical_range_rms_m",nullable(h.physical_range_rms_m)}});
  return {{"ok",d.ok},{"time_us",d.source_stamp_us},{"xy_m",nullable(d.xy)},
    {"source",d.source},{"reason",d.reason},{"B",fit(d.b4)},{"hypotheses",hs},
    {"best_excluded_anchor",d.best_excluded_anchor},{"best_score_m",nullable(d.best_score_m)},
    {"score_margin_m",nullable(d.score_margin_m)},{"history_samples",d.history_samples},{"fresh_observation_count",d.fresh_observation_count},{"newest_sample_us",nullable(d.newest_sample_us)},
    {"tracking_prior_age_s",nullable(d.tracking_prior_age_s)},
    {"tracking_step_m",nullable(d.tracking_step_m)},{"tracking_limit_m",nullable(d.tracking_limit_m)},
    {"reset_reason",d.reset_reason},{"b4_height_reason",d.b4_height_reason},
    {"b4_height_blocked",d.b4_height_blocked},
    {"b4_physical_range_rms_m",nullable(d.b4_physical_range_rms_m)}};
}
}
