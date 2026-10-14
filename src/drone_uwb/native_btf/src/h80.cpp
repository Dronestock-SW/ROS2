#include "btf/core.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace btf {
void Settings::validate() const {
  for (double v : {window_s, sigma_r_m, huber_m, max_scaled_condition,
                   reset_gap_s, max_extrapolation_s, max_range_m})
    if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("invalid_B_setting");
  if (window_s > 60. || reset_gap_s > 60. || max_extrapolation_s > 60. ||
      max_scaled_condition <= 1. || h80_irls_iterations < 1 || h80_irls_iterations > 100 ||
      min_samples_per_anchor < 3 || min_samples_per_anchor > 100000 ||
      min_samples_total < 4 * min_samples_per_anchor ||
      required_anchor_count != 4)
    throw std::invalid_argument("invalid_four_anchor_B_settings");
}
void Config::validate() const {
  solver.validate();
  for (const auto& a : anchors) for (double v : a)
    if (!std::isfinite(v)) throw std::invalid_argument("invalid_geometry");
  for (double b : bias_m) if (!std::isfinite(b)) throw std::invalid_argument("invalid_bias");
  for (const auto& a : anchors)
    if (std::abs(a[2]-anchors[0][2]) > 1e-9)
      throw std::invalid_argument("reference_slant_requires_equal_anchor_heights");
  for (double v : {rms_limit_m, score_margin_m, max_speed_m_s, continuity_margin_m, max_prior_age_s})
    if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("invalid_subset_limits");
}

Fit fit_h80(const Anchors& anchors, const std::vector<RangeSample>& observations,
            Time reference_us, const Settings& cfg) {
  Fit f;
  auto checked=cfg;checked.required_anchor_count=4;checked.validate();
  if(cfg.required_anchor_count!=3&&cfg.required_anchor_count!=4)
    throw std::invalid_argument("invalid_required_anchor_count");
  if(reference_us<0||reference_us>9000000000000000LL)return f;
  for (const auto& a : anchors) {
    for (double v : a) if (!std::isfinite(v)) return f;
    if (std::abs(a[2]-anchors[0][2]) > 1e-9) {f.reason="unequal_anchor_heights"; return f;}
  }
  for (const auto& o : observations) {
    if (o.anchor < 0 || o.anchor > 3 || o.stamp_us<0 || o.stamp_us>9000000000000000LL ||
        !std::isfinite(o.range_m) || o.range_m <= 0) return f;
    if (o.stamp_us > reference_us || (reference_us-o.stamp_us)/1e6 > cfg.window_s+1e-9) {
      f.reason="outside_causal_window"; return f;
    }
    ++f.counts[o.anchor];
  }
  int participating = 0;
  for (int i=0;i<4;++i) if (f.counts[i] >= cfg.min_samples_per_anchor) {
    ++participating; f.mask |= 1<<i; f.n += f.counts[i];
  }
  if (participating < cfg.required_anchor_count || f.n < cfg.min_samples_total) {
    f.reason="insufficient_anchors"; return f;
  }
  Eigen::MatrixXd a(f.n,7);
  Eigen::VectorXd rhs(f.n), rr(f.n), offsets(f.n), var(f.n);
  int row=0;
  for (const auto& o : observations) {
    if (!(f.mask & (1<<o.anchor))) continue;
    const double u=(o.stamp_us-reference_us)/1e6/cfg.window_s;
    const double ax=anchors[o.anchor][0], ay=anchors[o.anchor][1], r=o.range_m;
    a.row(row) << -2*ax, -2*ay, -2*ax*u, -2*ay*u, 1., u, u*u;
    offsets[row]=ax*ax+ay*ay; rr[row]=r; rhs[row]=r*r-offsets[row];
    const double sigma2=cfg.sigma_r_m*cfg.sigma_r_m;
    var[row]=4*r*r*sigma2+2*sigma2*sigma2; ++row;
  }
  if (!a.allFinite() || !rhs.allFinite() || !var.allFinite() || (var.array()<=0).any()) {
    f.reason="solver_failed"; return f;
  }
  const Eigen::VectorXd scale=a.cwiseAbs().colwise().maxCoeff();
  if ((scale.array()==0).any()) {f.reason="rank_deficient"; return f;}
  const Eigen::MatrixXd scaled=a*scale.cwiseInverse().asDiagonal();
  const Eigen::VectorXd base=var.cwiseInverse();
  Eigen::VectorXd weights=base, theta(7);
  auto solve = [&]() {
    const Eigen::VectorXd root=weights.cwiseSqrt();
    const Eigen::MatrixXd weighted=root.asDiagonal()*scaled;
    const Eigen::VectorXd b=root.array()*rhs.array();
    if (!weighted.allFinite() || !b.allFinite()) throw std::runtime_error("solver_failed");
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(weighted,Eigen::ComputeThinU|Eigen::ComputeThinV);
    // NumPy lstsq(rcond=None): epsilon * max(rows, columns).
    svd.setThreshold(std::numeric_limits<double>::epsilon()*std::max(f.n,7));
    f.rank=static_cast<int>(svd.rank());
    if (f.rank<7) throw std::runtime_error("rank_deficient");
    f.condition=svd.singularValues()[0]/svd.singularValues()[6];
    if (!std::isfinite(*f.condition)) throw std::runtime_error("solver_failed");
    if (*f.condition>cfg.max_scaled_condition) throw std::runtime_error("ill_conditioned");
    theta=svd.solve(b).array()/scale.array();
  };
  try {
    for (int iter=0;iter<cfg.h80_irls_iterations;++iter) {
      solve(); const Eigen::VectorXd residual=a*theta-rhs;
      Eigen::VectorXd updated(f.n); f.converged=true;
      for (int i=0;i<f.n;++i) {
        const double e=residual[i]/(2*std::max(rr[i],.1));
        updated[i]=base[i]*std::min(1.,cfg.huber_m/std::max(std::abs(e),1e-12));
        if (std::abs(updated[i]-weights[i])>1e-6*std::abs(weights[i])) f.converged=false;
      }
      weights=updated; ++f.iterations;
      if (f.converged) break;
    }
    solve();
  } catch (const std::runtime_error& e) {f.reason=e.what(); return f;}
  const Eigen::VectorXd sq=a*theta+offsets;
  double sum=0, maximum=0;
  for (int i=0;i<f.n;++i) {
    const double error=std::sqrt(std::max(sq[i],0.))-rr[i];
    sum+=error*error; maximum=std::max(maximum,std::abs(error));
  }
  f.rms_m=std::sqrt(sum/f.n); f.max_m=maximum;
  if (!theta.allFinite() || !std::isfinite(*f.rms_m)) {
    f.rms_m.reset(); f.max_m.reset(); f.reason="solver_failed"; return f;
  }
  std::copy(theta.data(),theta.data()+7,f.coefficients.begin());
  f.xy=XY{theta[0],theta[1]}; f.velocity=XY{theta[2]/cfg.window_s,theta[3]/cfg.window_s};
  f.ok=true; f.reason="ok"; return f;
}
}  // namespace btf
