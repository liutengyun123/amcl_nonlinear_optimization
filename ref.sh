dr_extrapolator.cpp:
#include "reflector_localization/dr_extrapolator.hpp"

#include <cmath>

namespace reflector_localization {

DrExtrapolator::DrExtrapolator(const DrConfig& cfg) : cfg_(cfg) {}

void DrExtrapolator::Reset(const Pose2& pose, const ros::Time& stamp) {
  pose_ = pose;
  pose_stamp_ = stamp;
  reset_stamp_ = stamp;
  accumulated_distance_ = 0.0;
  accumulated_yaw_ = 0.0;
  initialized_ = true;
}

void DrExtrapolator::UpdateLinearVelocity(double v, const ros::Time& stamp) {
  linear_v_ = v;
  last_linear_stamp_ = stamp;
}

void DrExtrapolator::UpdateAngularVelocity(double wz, const ros::Time& stamp) {
  angular_w_ = wz;
  last_angular_stamp_ = stamp;
}

double DrExtrapolator::ActiveLinearVelocity(const ros::Time& stamp) const {
  if (last_linear_stamp_.isZero()) return 0.0;
  if ((stamp - last_linear_stamp_).toSec() > cfg_.max_linear_sample_age_sec) {
    return 0.0;
  }
  return linear_v_;
}

double DrExtrapolator::ActiveAngularVelocity(const ros::Time& stamp) const {
  if (last_angular_stamp_.isZero()) return 0.0;
  if ((stamp - last_angular_stamp_).toSec() > cfg_.max_angular_sample_age_sec) {
    return 0.0;
  }
  return angular_w_;
}

void DrExtrapolator::IntegrateTo(const ros::Time& stamp) {
  if (!initialized_ || stamp <= pose_stamp_) return;
  const double dt = (stamp - pose_stamp_).toSec();
  if (dt <= 0.0) return;

  const double v = ActiveLinearVelocity(stamp);
  const double w = ActiveAngularVelocity(stamp);
  const double yaw_mid = pose_.yaw + 0.5 * w * dt;

  pose_.x += v * std::cos(yaw_mid) * dt;
  pose_.y += v * std::sin(yaw_mid) * dt;
  pose_.yaw = WrapAngle(pose_.yaw + w * dt);
  pose_.valid = true;

  accumulated_distance_ += std::fabs(v) * dt;
  accumulated_yaw_ += std::fabs(w) * dt;
  pose_stamp_ = stamp;
}

bool DrExtrapolator::ExtrapolateTo(const ros::Time& stamp, Pose2* pose_out,
                                   double* elapsed_out, double* distance_out,
                                   double* yaw_change_out) {
  if (!cfg_.enabled || !initialized_) return false;
  IntegrateTo(stamp);

  const double elapsed = (stamp - reset_stamp_).toSec();
  if (elapsed > cfg_.timeout_sec) return false;
  if (accumulated_distance_ > cfg_.max_distance ||
      accumulated_yaw_ > cfg_.max_yaw_rad) {
    return false;
  }

  if (pose_out) *pose_out = pose_;
  if (elapsed_out) *elapsed_out = elapsed;
  if (distance_out) *distance_out = accumulated_distance_;
  if (yaw_change_out) *yaw_change_out = accumulated_yaw_;
  return true;
}

}  // namespace reflector_localization

pair_joint_fitter.cpp:
#include "reflector_localization/pair_joint_fitter.hpp"

#include <cmath>
#include <limits>

namespace reflector_localization {
namespace {

using StateVector = Eigen::Matrix<double, 6, 1>;

struct PairOrdering {
  int left_idx = -1;
  int right_idx = -1;
  const StripIntervalObservation* left = nullptr;
  const StripIntervalObservation* right = nullptr;
};

struct FitMetrics {
  double cost = std::numeric_limits<double>::infinity();
  double interval_cost = 0.0;
  double support_line_cost = 0.0;
  double mid_prior_cost = 0.0;
  double runtime_sep_cost = 0.0;
  double yaw_prior_cost = 0.0;
  double bias_cost = 0.0;
  double quality = 0.0;
};

struct OptimizeSummary {
  PairState initial_state;
  FitMetrics initial_metrics;
  PairState final_state;
  FitMetrics final_metrics;
};

Point2 IntervalApproxCenterBase(const StripIntervalObservation& interval) {
  return Midpoint(interval.left_base, interval.right_base);
}

PairOrdering MakeOrdering(const std::vector<StripIntervalObservation>& strips,
                          int idx_a,
                          int idx_b) {
  PairOrdering ordering;
  if (strips[idx_a].s_center <= strips[idx_b].s_center) {
    ordering.left_idx = idx_a;
    ordering.right_idx = idx_b;
  } else {
    ordering.left_idx = idx_b;
    ordering.right_idx = idx_a;
  }
  ordering.left = &strips[ordering.left_idx];
  ordering.right = &strips[ordering.right_idx];
  return ordering;
}

PairState InitialState(const PairOrdering& ordering) {
  PairState state;
  state.mid_s = 0.5 * (ordering.left->s_center + ordering.right->s_center);
  state.mid_d = 0.5 * (ordering.left->d_center + ordering.right->d_center);
  state.delta_yaw = 0.0;
  state.separation = std::max(0.05, ordering.right->s_center - ordering.left->s_center);
  state.bias_left = ordering.left->d_center - state.mid_d;
  state.bias_right = ordering.right->d_center - state.mid_d;
  return state;
}

StateVector StateToVector(const PairState& state) {
  StateVector x;
  x << state.mid_s, state.mid_d, state.delta_yaw, state.separation,
      state.bias_left, state.bias_right;
  return x;
}

PairState VectorToState(const StateVector& x, const PairState& template_state) {
  PairState state = template_state;
  state.mid_s = x(0);
  state.mid_d = Clamp(x(1), -0.20, 0.20);
  state.delta_yaw = Clamp(x(2), -15.0 * M_PI / 180.0, 15.0 * M_PI / 180.0);
  state.separation = std::max(0.05, x(3));
  state.bias_left = Clamp(x(4), -0.15, 0.15);
  state.bias_right = Clamp(x(5), -0.15, 0.15);
  return state;
}

FitMetrics EvaluateState(const PairState& state,
                         const PairOrdering& ordering,
                         const SupportLineEstimate& line,
                         const ScenePrior& prior,
                         const PairJointRuntimePrior& runtime_prior,
                         const PairJointFitterConfig& cfg,
                         Eigen::VectorXd* residual_vector) {
  std::vector<double> residuals;
  residuals.reserve(24);

  FitMetrics metrics;

  const double pred_s_left = state.mid_s - 0.5 * state.separation;
  const double pred_s_right = state.mid_s + 0.5 * state.separation;
  const double pred_d_left = state.mid_d + state.bias_left;
  const double pred_d_right = state.mid_d + state.bias_right;

  auto append = [&](double residual, double* accum_cost) {
    residuals.push_back(residual);
    if (accum_cost != nullptr) {
      *accum_cost += residual * residual;
    }
  };

  append((ordering.left->s_center - pred_s_left) /
             std::max(ordering.left->sigma_s_center, 1e-3),
         &metrics.interval_cost);
  append((ordering.right->s_center - pred_s_right) /
             std::max(ordering.right->sigma_s_center, 1e-3),
         &metrics.interval_cost);

  append((ordering.left->width_s - prior.width_nominal_m) /
             std::max(ordering.left->sigma_width_s, 1e-3),
         &metrics.interval_cost);
  append((ordering.right->width_s - prior.width_nominal_m) /
             std::max(ordering.right->sigma_width_s, 1e-3),
         &metrics.interval_cost);

  append((ordering.left->d_center - pred_d_left) /
             std::max(ordering.left->sigma_d_center, 1e-3),
         &metrics.interval_cost);
  append((ordering.right->d_center - pred_d_right) /
             std::max(ordering.right->sigma_d_center, 1e-3),
         &metrics.interval_cost);

  append(ordering.left->skew_n / std::max(ordering.left->sigma_skew_n, 1e-3),
         &metrics.interval_cost);
  append(ordering.right->skew_n / std::max(ordering.right->sigma_skew_n, 1e-3),
         &metrics.interval_cost);

  append(state.mid_d / std::max(line.sigma_offset_m, 1e-3),
         &metrics.support_line_cost);
  if (!line.yaw_frozen) {
    append(state.delta_yaw / std::max(line.sigma_yaw_rad, 1e-3),
           &metrics.support_line_cost);
  }

  const Point2 midpoint = PairMidpointBase(state, line);
  if (prior.mid_x_sigma_m > 1e-6) {
    append((midpoint.x - prior.mid_x_nominal_m) / prior.mid_x_sigma_m,
           &metrics.mid_prior_cost);
  }
  if (prior.mid_y_sigma_m > 1e-6) {
    append((midpoint.y - prior.mid_y_nominal_m) / prior.mid_y_sigma_m,
           &metrics.mid_prior_cost);
  }

  if (runtime_prior.valid && runtime_prior.separation_sigma_m > 1e-6) {
    append((state.separation - runtime_prior.separation_nominal_m) /
               runtime_prior.separation_sigma_m,
           &metrics.runtime_sep_cost);
  }

  append(state.delta_yaw /
             std::max(cfg.delta_yaw_sigma_deg * M_PI / 180.0, 1e-3),
         &metrics.yaw_prior_cost);

  append(state.bias_left / std::max(prior.bias_sigma_m, 1e-3),
         &metrics.bias_cost);
  append(state.bias_right / std::max(prior.bias_sigma_m, 1e-3),
         &metrics.bias_cost);
  append((state.bias_left - state.bias_right) /
             std::max(prior.bias_diff_sigma_m, 1e-3),
         &metrics.bias_cost);

  metrics.cost = metrics.interval_cost + metrics.support_line_cost +
                 metrics.mid_prior_cost + metrics.runtime_sep_cost +
                 metrics.yaw_prior_cost + metrics.bias_cost;
  metrics.quality =
      Clamp(std::min(ordering.left->quality, ordering.right->quality) *
                Clamp(line.quality, 0.2, 1.0) *
                std::exp(-0.5 * metrics.cost /
                         std::max<int>(1, static_cast<int>(residuals.size()))),
            0.0, 1.0);

  if (residual_vector != nullptr) {
    residual_vector->resize(static_cast<Eigen::Index>(residuals.size()));
    for (size_t i = 0; i < residuals.size(); ++i) {
      (*residual_vector)(static_cast<Eigen::Index>(i)) = residuals[i];
    }
  }
  return metrics;
}

Eigen::MatrixXd NumericJacobian(const PairState& state,
                                const PairOrdering& ordering,
                                const SupportLineEstimate& line,
                                const ScenePrior& prior,
                                const PairJointRuntimePrior& runtime_prior,
                                const PairJointFitterConfig& cfg,
                                Eigen::VectorXd* residual_base) {
  Eigen::VectorXd r0;
  EvaluateState(state, ordering, line, prior, runtime_prior, cfg, &r0);
  if (residual_base != nullptr) *residual_base = r0;

  Eigen::MatrixXd jac(r0.size(), 6);
  const StateVector x0 = StateToVector(state);
  for (int col = 0; col < 6; ++col) {
    StateVector x = x0;
    const double step = cfg.numeric_diff_eps * std::max(1.0, std::fabs(x(col)));
    x(col) += step;
    const PairState perturbed = VectorToState(x, state);
    Eigen::VectorXd r1;
    EvaluateState(perturbed, ordering, line, prior, runtime_prior, cfg, &r1);
    jac.col(col) = (r1 - r0) / step;
  }
  return jac;
}

PairCovariance MeasurementCovarianceFromStateCov(const PairState& state,
                                                 const SupportLineEstimate& line,
                                                 const PairStateCovariance& state_cov,
                                                 double eps) {
  auto measure = [&](const PairState& s) {
    Eigen::Matrix<double, 5, 1> y;
    const Point2 mid = PairMidpointBase(s, line);
    y << mid.x, mid.y, PairHeadingTotal(s, line), s.separation,
        0.5 * (s.bias_left + s.bias_right);
    return y;
  };

  const Eigen::Matrix<double, 5, 1> y0 = measure(state);
  const StateVector x0 = StateToVector(state);
  Eigen::Matrix<double, 5, 6> g = Eigen::Matrix<double, 5, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    StateVector x = x0;
    const double step = eps * std::max(1.0, std::fabs(x(i)));
    x(i) += step;
    g.col(i) = (measure(VectorToState(x, state)) - y0) / step;
  }
  return g * state_cov * g.transpose();
}

bool OptimizePair(const PairOrdering& ordering,
                  const SupportLineEstimate& line,
                  const ScenePrior& prior,
                  const PairJointRuntimePrior& runtime_prior,
                  const PairJointFitterConfig& cfg,
                  PairMeasurement* measurement,
                  FitMetrics* fit_metrics,
                  OptimizeSummary* summary) {
  PairState state = InitialState(ordering);
  if (runtime_prior.valid && runtime_prior.separation_nominal_m > 0.05) {
    state.separation =
        Clamp(0.5 * state.separation + 0.5 * runtime_prior.separation_nominal_m,
              0.05, 1.0);
  }

  FitMetrics best_metrics =
      EvaluateState(state, ordering, line, prior, runtime_prior, cfg, nullptr);
  const PairState initial_state = state;
  const FitMetrics initial_metrics = best_metrics;
  double lambda = cfg.lm_lambda;

  for (int iter = 0; iter < cfg.max_iterations; ++iter) {
    Eigen::VectorXd residuals;
    const Eigen::MatrixXd jac = NumericJacobian(state, ordering, line, prior,
                                                runtime_prior, cfg, &residuals);
    const Eigen::Matrix<double, 6, 6> h =
        jac.transpose() * jac + lambda * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> g = jac.transpose() * residuals;
    const Eigen::Matrix<double, 6, 1> dx = h.ldlt().solve(-g);
    if (!dx.allFinite()) break;

    const PairState candidate = VectorToState(StateToVector(state) + dx, state);
    const FitMetrics candidate_metrics =
        EvaluateState(candidate, ordering, line, prior, runtime_prior, cfg, nullptr);
    if (candidate_metrics.cost + cfg.cost_tolerance < best_metrics.cost) {
      state = candidate;
      best_metrics = candidate_metrics;
      lambda = std::max(lambda * 0.5, 1e-6);
      if (dx.norm() < 1e-4) break;
    } else {
      lambda = std::min(lambda * 4.0, 1e3);
    }
  }

  Eigen::VectorXd residuals;
  const Eigen::MatrixXd jac = NumericJacobian(state, ordering, line, prior,
                                              runtime_prior, cfg, &residuals);
  Eigen::Matrix<double, 6, 6> info = jac.transpose() * jac;
  info += 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
  const PairStateCovariance state_cov = info.inverse();
  const PairCovariance measurement_cov =
      MeasurementCovarianceFromStateCov(state, line, state_cov, cfg.numeric_diff_eps);

  measurement->valid = true;
  measurement->strip_idx_left = ordering.left->strip_id;
  measurement->strip_idx_right = ordering.right->strip_id;
  measurement->state = state;
  measurement->support_line = line;
  measurement->midpoint_base = PairMidpointBase(state, line);
  measurement->heading_base = PairHeadingTotal(state, line);
  measurement->strip_left_center_base = PairStripCenterLeft(state, line);
  measurement->strip_right_center_base = PairStripCenterRight(state, line);
  measurement->covariance = measurement_cov;
  measurement->fit_cost = best_metrics.cost;
  measurement->quality = best_metrics.quality;
  measurement->residual_strip_left = ordering.left->skew_n;
  measurement->residual_strip_right = ordering.right->skew_n;
  measurement->residual_support_line = std::sqrt(best_metrics.support_line_cost);
  measurement->translation_sigma_m =
      std::sqrt(std::max(measurement_cov(0, 0), measurement_cov(1, 1)));
  measurement->yaw_sigma_rad =
      std::sqrt(std::max(measurement_cov(2, 2), line.sigma_yaw_rad * line.sigma_yaw_rad));
  measurement->freeze_yaw = line.yaw_frozen ||
                            measurement->yaw_sigma_rad > line.sigma_yaw_rad;
  measurement->freeze_normal =
      std::sqrt(std::max(measurement_cov(4, 4), 1e-6)) > 0.04;

  if (fit_metrics != nullptr) *fit_metrics = best_metrics;
  if (summary != nullptr) {
    summary->initial_state = initial_state;
    summary->initial_metrics = initial_metrics;
    summary->final_state = state;
    summary->final_metrics = best_metrics;
  }
  return true;
}

}  // namespace

PairJointFitter::PairJointFitter(const PairJointFitterConfig& cfg) : cfg_(cfg) {}

bool PairJointFitter::Fit(const std::vector<StripIntervalObservation>& strips,
                          const SupportLineEstimate& line,
                          const ScenePrior& prior,
                          const PairJointRuntimePrior& runtime_prior,
                          PairMeasurement* out,
                          std::string* error,
                          PairJointDebugInfo* debug_info) const {
  if (out == nullptr) {
    if (error) *error = "Fit() failed: out is null";
    return false;
  }

  *out = PairMeasurement();
  if (debug_info != nullptr) *debug_info = PairJointDebugInfo();

  if (strips.size() < 2) {
    if (error) *error = "need at least two strip interval observations";
    return false;
  }
  if (!line.valid) {
    if (error) *error = "support line invalid";
    return false;
  }

  PairMeasurement best_measurement;
  double best_score = std::numeric_limits<double>::infinity();
  double best_range = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < strips.size(); ++i) {
    for (size_t j = i + 1; j < strips.size(); ++j) {
      PairJointCandidateDebug dbg;
      dbg.strip_idx_a = strips[i].strip_id;
      dbg.strip_idx_b = strips[j].strip_id;
      dbg.left_s_center = strips[i].s_center;
      dbg.right_s_center = strips[j].s_center;
      dbg.left_width_s = strips[i].width_s;
      dbg.right_width_s = strips[j].width_s;
      dbg.left_d_center = strips[i].d_center;
      dbg.right_d_center = strips[j].d_center;
      dbg.left_skew_n = strips[i].skew_n;
      dbg.right_skew_n = strips[j].skew_n;

      if (!strips[i].usable_for_position || !strips[j].usable_for_position) {
        dbg.rejection_reason = "interval_position_gate";
        if (debug_info != nullptr) debug_info->candidates.push_back(dbg);
        continue;
      }

      const Point2 ci = IntervalApproxCenterBase(strips[i]);
      const Point2 cj = IntervalApproxCenterBase(strips[j]);
      if ((cfg_.max_strip_range_m > 0.0 &&
           (Norm(ci) > cfg_.max_strip_range_m || Norm(cj) > cfg_.max_strip_range_m))) {
        dbg.rejection_reason = "strip_range_gate";
        if (debug_info != nullptr) debug_info->candidates.push_back(dbg);
        continue;
      }
      if (cfg_.max_pair_separation_m > 0.0 &&
          std::fabs(strips[i].s_center - strips[j].s_center) > cfg_.max_pair_separation_m) {
        dbg.rejection_reason = "pair_separation_gate";
        if (debug_info != nullptr) debug_info->candidates.push_back(dbg);
        continue;
      }

      const PairOrdering ordering = MakeOrdering(strips, static_cast<int>(i), static_cast<int>(j));
      PairMeasurement measurement;
      FitMetrics metrics;
      OptimizeSummary summary;
      if (!OptimizePair(ordering, line, prior, runtime_prior, cfg_,
                        &measurement, &metrics, &summary)) {
        continue;
      }

      const bool accepted =
          measurement.fit_cost <= prior.reject_fit_cost &&
          measurement.quality >= prior.min_quality;
      const double midpoint_range = Norm(measurement.midpoint_base);

      dbg.strip_idx_a = ordering.left->strip_id;
      dbg.strip_idx_b = ordering.right->strip_id;
      dbg.left_s_center = ordering.left->s_center;
      dbg.right_s_center = ordering.right->s_center;
      dbg.left_width_s = ordering.left->width_s;
      dbg.right_width_s = ordering.right->width_s;
      dbg.left_d_center = ordering.left->d_center;
      dbg.right_d_center = ordering.right->d_center;
      dbg.left_skew_n = ordering.left->skew_n;
      dbg.right_skew_n = ordering.right->skew_n;
      dbg.init_mid_s = summary.initial_state.mid_s;
      dbg.init_mid_d = summary.initial_state.mid_d;
      dbg.init_delta_yaw_deg = summary.initial_state.delta_yaw * 180.0 / M_PI;
      dbg.init_sep = summary.initial_state.separation;
      dbg.init_bias_l = summary.initial_state.bias_left;
      dbg.init_bias_r = summary.initial_state.bias_right;
      dbg.final_mid_s = summary.final_state.mid_s;
      dbg.final_mid_d = summary.final_state.mid_d;
      dbg.final_delta_yaw_deg = summary.final_state.delta_yaw * 180.0 / M_PI;
      dbg.final_sep = summary.final_state.separation;
      dbg.final_bias_l = summary.final_state.bias_left;
      dbg.final_bias_r = summary.final_state.bias_right;
      dbg.init_fit_cost = summary.initial_metrics.cost;
      dbg.fit_cost = measurement.fit_cost;
      dbg.quality = measurement.quality;
      dbg.interval_cost = summary.final_metrics.interval_cost;
      dbg.support_line_cost = summary.final_metrics.support_line_cost;
      dbg.mid_prior_cost = summary.final_metrics.mid_prior_cost;
      dbg.runtime_sep_cost = summary.final_metrics.runtime_sep_cost;
      dbg.yaw_prior_cost = summary.final_metrics.yaw_prior_cost;
      dbg.bias_cost = summary.final_metrics.bias_cost;
      dbg.accepted = accepted;
      dbg.rejection_reason = accepted ? "accepted" : "fit_or_quality_gate";
      if (debug_info != nullptr) debug_info->candidates.push_back(dbg);
      if (!accepted) continue;

      const double score =
          measurement.fit_cost - 4.0 * measurement.quality + 0.10 * midpoint_range;
      if (!runtime_prior.valid) {
        if (midpoint_range + 1e-6 < best_range ||
            (std::fabs(midpoint_range - best_range) <= 1e-6 && score < best_score)) {
          best_score = score;
          best_range = midpoint_range;
          best_measurement = measurement;
        }
      } else if (score < best_score) {
        best_score = score;
        best_range = midpoint_range;
        best_measurement = measurement;
      }
    }
  }

  if (!best_measurement.valid) {
    if (error) *error = "no pair candidate passed joint fit";
    return false;
  }

  *out = best_measurement;
  if (error) error->clear();
  return true;
}

}  // namespace reflector_localization
pair_map_associator.cpp:
#include "reflector_localization/pair_map_associator.hpp"

#include <cmath>
#include <limits>

namespace reflector_localization {
namespace {

double NormalizedCost(double value, double gate) {
  return value / std::max(gate, 1e-6);
}

}  // namespace

PairMapAssociator::PairMapAssociator(const PairMapAssociatorConfig& cfg) : cfg_(cfg) {}

bool PairMapAssociator::PreferredPair(int a, int b, int pa, int pb) {
  if (pa < 0 || pb < 0) return true;
  return (a == pa && b == pb) || (a == pb && b == pa);
}

PairAssociationResult PairMapAssociator::Match(const PairMeasurement& measurement,
                                               const ReflectorMap& map,
                                               const Pose2& prior_pose,
                                               int preferred_map_a,
                                               int preferred_map_b,
                                               PairAssociationDebugInfo* debug_info) const {
  if (debug_info) {
    *debug_info = PairAssociationDebugInfo();
  }

  if (preferred_map_a >= 0 && preferred_map_b >= 0) {
    if (debug_info) {
      debug_info->preferred_search_requested = true;
      debug_info->preferred_map_a = preferred_map_a;
      debug_info->preferred_map_b = preferred_map_b;
    }
    PairAssociationResult best = MatchOnce(
        measurement, map, prior_pose, preferred_map_a, preferred_map_b,
        true, true, debug_info ? &debug_info->preferred_search : nullptr);
    if (best.success) return best;
    if (debug_info) debug_info->fallback_to_global = true;
  }

  return MatchOnce(measurement, map, prior_pose, -1, -1, false, false,
                   debug_info ? &debug_info->global_search : nullptr);
}

PairAssociationResult PairMapAssociator::MatchOnce(
    const PairMeasurement& measurement,
    const ReflectorMap& map,
    const Pose2& prior_pose,
    int preferred_map_a,
    int preferred_map_b,
    bool restrict_to_preferred,
    bool allow_preferred_bonus,
    PairAssociationSearchStats* stats) const {
  PairAssociationResult best;
  best.measurement = measurement;

  for (const auto& map_pair : map.pairs()) {
    if (std::fabs(map_pair.distance - measurement.state.separation) > cfg_.pair_distance_tol) {
      continue;
    }
    const bool preferred =
        PreferredPair(map_pair.idx_a, map_pair.idx_b, preferred_map_a, preferred_map_b);
    if (restrict_to_preferred && !preferred) continue;
    if (stats) ++stats->distance_compatible_pairs;

    PairAssociationResult candidate =
        EvaluateCandidate(measurement, map_pair, prior_pose,
                          allow_preferred_bonus && preferred, stats);
    if (!candidate.success) continue;
    if (candidate.score < best.score) best = candidate;
  }

  return best;
}

PairAssociationResult PairMapAssociator::EvaluateCandidate(
    const PairMeasurement& measurement,
    const MapPair& map_pair,
    const Pose2& prior_pose,
    bool preferred_bonus,
    PairAssociationSearchStats* stats) const {
  PairAssociationResult best;
  best.measurement = measurement;

  const Point2 prior_left = TransformPoint(prior_pose, measurement.strip_left_center_base);
  const Point2 prior_right = TransformPoint(prior_pose, measurement.strip_right_center_base);
  const Point2 prior_mid = Midpoint(prior_left, prior_right);
  const double prior_heading = Heading(prior_left, prior_right);

  for (int swap = 0; swap < 2; ++swap) {
    if (stats) ++stats->ordered_candidates_tested;

    const Point2 ordered_left = swap == 0 ? map_pair.a_map : map_pair.b_map;
    const Point2 ordered_right = swap == 0 ? map_pair.b_map : map_pair.a_map;
    const int ordered_idx_left = swap == 0 ? map_pair.idx_a : map_pair.idx_b;
    const int ordered_idx_right = swap == 0 ? map_pair.idx_b : map_pair.idx_a;

    const double point_cost =
        Distance(prior_left, ordered_left) + Distance(prior_right, ordered_right);
    if (point_cost > cfg_.prior_point_gate) {
      if (stats) ++stats->rejected_point_gate;
      continue;
    }

    const Point2 map_mid = Midpoint(ordered_left, ordered_right);
    const double midpoint_cost = Distance(prior_mid, map_mid);
    if (midpoint_cost > cfg_.midpoint_gate) {
      if (stats) ++stats->rejected_midpoint_gate;
      continue;
    }

    const double heading_cost =
        std::fabs(WrapAngle(prior_heading - Heading(ordered_left, ordered_right)));
    if (heading_cost > cfg_.heading_gate_rad) {
      if (stats) ++stats->rejected_heading_gate;
      continue;
    }

    const Pose2 pose = pose_solver_.Solve(measurement, map_pair, swap == 1);
    const double trans_delta =
        std::hypot(pose.x - prior_pose.x, pose.y - prior_pose.y);
    const double yaw_delta = std::fabs(WrapAngle(pose.yaw - prior_pose.yaw));
    if (trans_delta > cfg_.max_prior_translation_delta ||
        yaw_delta > cfg_.max_prior_yaw_delta) {
      if (stats) ++stats->rejected_prior_delta_gate;
      continue;
    }

    if (stats) ++stats->successful_candidates;

    double score = 1.0 * NormalizedCost(point_cost, cfg_.prior_point_gate) +
                   0.7 * NormalizedCost(midpoint_cost, cfg_.midpoint_gate) +
                   0.6 * NormalizedCost(heading_cost, cfg_.heading_gate_rad) +
                   0.3 * NormalizedCost(trans_delta, cfg_.max_prior_translation_delta) +
                   0.3 * NormalizedCost(yaw_delta, cfg_.max_prior_yaw_delta) +
                   0.1 * measurement.fit_cost -
                   0.5 * measurement.quality;
    if (preferred_bonus) score -= 0.10;

    if (score < best.score) {
      best.success = true;
      best.map_idx_a = ordered_idx_left;
      best.map_idx_b = ordered_idx_right;
      best.swapped = (swap == 1);
      best.pose_map_base = pose;
      best.score = score;
      best.prior_point_cost = point_cost;
      best.midpoint_cost = midpoint_cost;
      best.heading_cost = heading_cost;
      best.prior_translation_delta = trans_delta;
      best.prior_yaw_delta = yaw_delta;
      best.preferred_pair_used = preferred_bonus;
    }
  }

  return best;
}

}  // namespace reflector_localization
pair_pose_solver.cpp:
#include "reflector_localization/pair_pose_solver.hpp"

namespace reflector_localization {

Pose2 PairPoseSolver::Solve(const PairMeasurement& measurement,
                            const MapPair& map_pair,
                            bool swapped) const {
  const Point2 map_left = swapped ? map_pair.b_map : map_pair.a_map;
  const Point2 map_right = swapped ? map_pair.a_map : map_pair.b_map;
  return SolvePoseFromTwoPoints(measurement.strip_left_center_base,
                                measurement.strip_right_center_base,
                                map_left, map_right);
}

}  // namespace reflector_localization
pose_filter.cpp:
#include "reflector_localization/pose_filter.hpp"

namespace reflector_localization {

PoseFilter::PoseFilter(const PoseFilterConfig& cfg) : cfg_(cfg) {}

PoseEstimate PoseFilter::Fuse(const PairAssociationResult& association,
                              const Pose2& prior_pose,
                              const PoseFilterRuntime& runtime) const {
  PoseEstimate estimate;
  estimate.pose_map_base = association.pose_map_base;
  estimate.used_measurement = association.success;
  estimate.dr_mode = runtime.dr_mode;

  if (!association.success) {
    estimate.pose_map_base = prior_pose;
    estimate.pose_map_base.valid = prior_pose.valid;
    return estimate;
  }

  const Pose2& measured = association.pose_map_base;
  if (!prior_pose.valid || runtime.first_measurement ||
      (runtime.pair_changed && runtime.pair_change_direct_accept)) {
    estimate.pose_map_base = measured;
    estimate.pose_map_base.valid = true;
  } else {
    const double sigma_t =
        std::max(0.003, association.measurement.translation_sigma_m);
    const double sigma_y =
        std::max(0.003, association.measurement.yaw_sigma_rad);
    const double quality = Clamp(association.measurement.quality, 0.05, 1.0);
    const double speed_scale =
        runtime.motion_level < 0.05 ? cfg_.low_speed_gain_scale : 1.0;

    double gain_t = cfg_.translation_sigma_ref_m /
                    (cfg_.translation_sigma_ref_m + sigma_t);
    double gain_y = cfg_.yaw_sigma_ref_rad /
                    (cfg_.yaw_sigma_ref_rad + sigma_y);
    gain_t = Clamp(gain_t * quality * speed_scale,
                   cfg_.translation_gain_min, cfg_.translation_gain_max);
    gain_y = Clamp(gain_y * quality * speed_scale,
                   cfg_.yaw_gain_min, cfg_.yaw_gain_max);
    if (association.measurement.freeze_yaw) {
      gain_y = 0.0;
    }

    estimate.translation_gain = gain_t;
    estimate.yaw_gain = gain_y;
    estimate.pose_map_base = prior_pose;
    estimate.pose_map_base.x = prior_pose.x + gain_t * (measured.x - prior_pose.x);
    estimate.pose_map_base.y = prior_pose.y + gain_t * (measured.y - prior_pose.y);
    estimate.pose_map_base.yaw =
        WrapAngle(prior_pose.yaw + gain_y * WrapAngle(measured.yaw - prior_pose.yaw));
    estimate.pose_map_base.valid = true;
  }

  estimate.covariance = Eigen::Matrix3d::Zero();
  estimate.covariance(0, 0) =
      association.measurement.translation_sigma_m *
      association.measurement.translation_sigma_m;
  estimate.covariance(1, 1) =
      association.measurement.translation_sigma_m *
      association.measurement.translation_sigma_m;
  estimate.covariance(2, 2) =
      association.measurement.yaw_sigma_rad *
      association.measurement.yaw_sigma_rad;
  return estimate;
}

}  // namespace reflector_localization
reflector_detector.cpp:
#include "reflector_localization/reflector_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace reflector_localization {
namespace {

struct BeamSample {
  int scan_index = -1;
  double angle = 0.0;
  double range = 0.0;
  double intensity = 0.0;
  Point2 laser;
};
reflector_localization_node.cpp:
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <loc_manager/AlgoEnable.h>
#include <loc_manager/AlgoSetInitialPose.h>
#include <loc_manager/AlgoSwitchMap.h>
#include <loc_manager/RegistLoc.h>
#include <map_server/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "reflector_localization/dr_extrapolator.hpp"
#include "reflector_localization/pair_joint_fitter.hpp"
#include "reflector_localization/pair_map_associator.hpp"
#include "reflector_localization/pose_filter.hpp"
#include "reflector_localization/reflector_map.hpp"
#include "reflector_localization/scan_window_builder.hpp"
#include "reflector_localization/scene_prior.hpp"
#include "reflector_localization/strip_edge_extractor.hpp"
#include "reflector_localization/strip_interval_projector.hpp"
#include "reflector_localization/support_line_fitter.hpp"
#include "reflector_localization/types.hpp"
#include "reflector_localization/visualizer.hpp"

namespace reflector_localization {
namespace {

std::string TrimCopy(const std::string& s) {
  size_t b = 0;
  size_t e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b])) != 0) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])) != 0) --e;
  return s.substr(b, e - b);
}

bool ParseMapLayerSuffix(const std::string& in,
                         std::string* base,
                         int* layer) {
  if (!base || !layer) return false;
  *base = in;
  *layer = 0;
  if (in.size() < 3) return false;

  const size_t pos = in.find_last_of('_');
  if (pos == std::string::npos || pos + 2 > in.size()) return false;
  const char c = in[pos + 1];
  if (c != 'l' && c != 'L') return false;

  const std::string num = in.substr(pos + 2);
  if (num.empty()) return false;
  for (char ch : num) {
    if (std::isdigit(static_cast<unsigned char>(ch)) == 0) return false;
  }

  try {
    *layer = std::stoi(num);
  } catch (...) {
    return false;
  }
  *base = in.substr(0, pos);
  return !base->empty() && *layer > 0;
}

double ToDeg(double rad) { return rad * 180.0 / M_PI; }

void CanonicalizePairIds(int* a, int* b) {
  if (!a || !b) return;
  if (*a > *b) std::swap(*a, *b);
}

std::string PairString(int a, int b) {
  if (a < 0 || b < 0) return "<none>";
  CanonicalizePairIds(&a, &b);
  std::ostringstream oss;
  oss << "(" << a << "," << b << ")";
  return oss.str();
}

std::string PoseString(const Pose2& pose) {
  if (!pose.valid) return "<invalid>";
  std::ostringstream oss;
  oss << "(" << pose.x << "," << pose.y << "," << ToDeg(pose.yaw) << "deg)";
  return oss.str();
}

std::string PointString(const Point2& p) {
  std::ostringstream oss;
  oss << "(" << p.x << "," << p.y << ")";
  return oss.str();
}

std::string FormatSearchStats(const PairAssociationSearchStats& stats) {
  std::ostringstream oss;
  oss << "pairs=" << stats.distance_compatible_pairs
      << " tested=" << stats.ordered_candidates_tested
      << " ok=" << stats.successful_candidates
      << " rej(point/mid/head/prior)="
      << stats.rejected_point_gate << "/"
      << stats.rejected_midpoint_gate << "/"
      << stats.rejected_heading_gate << "/"
      << stats.rejected_prior_delta_gate;
  return oss.str();
}

std::string FormatAssociationDebug(const PairAssociationDebugInfo& debug_info) {
  std::ostringstream oss;
  if (debug_info.preferred_search_requested) {
    oss << "preferred=" << PairString(debug_info.preferred_map_a,
                                      debug_info.preferred_map_b)
        << " pref_stats={" << FormatSearchStats(debug_info.preferred_search) << "}"
        << " fallback=" << (debug_info.fallback_to_global ? "true" : "false");
  }
  if (!debug_info.preferred_search_requested || debug_info.fallback_to_global) {
    if (oss.tellp() > 0) oss << " ";
    oss << "global_stats={" << FormatSearchStats(debug_info.global_search) << "}";
  }
  return oss.str();
}

}  // namespace

enum class State : uint8_t {
  INIT = 0,
  SEARCHING = 1,
  LOCKED = 2,
  DR_ONLY = 3,
  LOST = 4,
  ERROR = 5,
};

class ReflectorLocalizationNode {
 public:
  ReflectorLocalizationNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    LoadParams();

    scan_window_builder_.reset(new ScanWindowBuilder(scan_window_cfg_));
    strip_edge_extractor_.reset(new StripEdgeExtractor(strip_cfg_));
    support_line_fitter_.reset(new SupportLineFitter(support_line_cfg_));
    strip_interval_projector_.reset(new StripIntervalProjector(strip_interval_cfg_));
    pair_joint_fitter_.reset(new PairJointFitter(pair_joint_cfg_));
    pair_map_associator_.reset(new PairMapAssociator(associator_cfg_));
    pose_filter_.reset(new PoseFilter(pose_filter_cfg_));
    dr_.reset(new DrExtrapolator(dr_cfg_));
    if (enable_markers_) {
      visualizer_.reset(new Visualizer(nh_, marker_topic_, map_frame_, base_frame_));
    }

    InitRosInterfaces();
    (void)RegisterToLocManager();

    std::string preload_err;
    if (!PreloadActiveMap("startup", &preload_err) && !json_path_.empty()) {
      ROS_WARN_STREAM("[REF] preload active map failed: " << preload_err
                      << "; fallback to ~json_path=" << json_path_);
      std::string fallback_err;
      if (!LoadMapFromJson(json_path_, "", 0, &fallback_err)) {
        ROS_WARN_STREAM("[REF] fallback json load failed: " << fallback_err);
      }
    }

    algo_enabled_ = !wait_for_enable_;
    if (algo_enabled_) {
      state_ = has_prior_pose_ ? State::SEARCHING : State::INIT;
    }
    PublishStatus();

    ROS_INFO_STREAM("[REF] dual-strip joint localizer ready. map="
                    << (has_map_ ? map_name_ : std::string("<none>"))
                    << " layer=" << map_layer_
                    << " scene_profile=" << scene_prior_.name);
  }

 private:
  void LoadParams() {
    pnh_.param<std::string>("json_path", json_path_, "");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/imu");
    pnh_.param<std::string>("wheel_odom_topic", wheel_odom_topic_, "/odom/wheel");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/reflector_loc/odom");
    pnh_.param<std::string>("status_topic", status_topic_, "/reflector_loc/status");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/reflector_loc/markers");
    pnh_.param<std::string>("prior_odom_topic", prior_odom_topic_, std::string(""));

    pnh_.param<std::string>("algo_name", algo_name_, algo_name_);
    pnh_.param<std::string>("loc_manager_regist_srv", loc_manager_regist_srv_,
                            loc_manager_regist_srv_);
    pnh_.param<std::string>("map_server_get_map_srv", map_server_get_map_srv_,
                            map_server_get_map_srv_);
    pnh_.param<std::string>("loc_manager_map_topic", loc_manager_map_topic_,
                            loc_manager_map_topic_);
    pnh_.param("wait_for_enable", wait_for_enable_, true);
    pnh_.param("auto_reload_on_map_event", auto_reload_on_map_event_, true);
    pnh_.param("need_mask_for_get_map", need_mask_for_get_map_, 0);

    pnh_.param("publish_tf", publish_tf_, true);
    pnh_.param("enable_markers", enable_markers_, true);
    pnh_.param("debug_log_match_events", debug_log_match_events_, true);
    pnh_.param("debug_log_observed_pair_geometry",
               debug_log_observed_pair_geometry_, true);

    pnh_.param("min_observations", min_observations_, 5);
    pnh_.param("max_rms_radius", max_rms_radius_, 0.05);

    pnh_.param("intensity_high_threshold",
               strip_cfg_.intensity_high_threshold, 250.0);
    if (!pnh_.getParam("intensity_low_threshold",
                       strip_cfg_.intensity_low_threshold)) {
      strip_cfg_.intensity_low_threshold =
          strip_cfg_.intensity_high_threshold - 40.0;
    }
    pnh_.param("detect_max_range", scan_window_cfg_.max_range_m, 3.0);
    pnh_.param("min_segment_points", strip_cfg_.min_segment_points, 4);
    pnh_.param("max_segment_points", strip_cfg_.max_segment_points, 20);
    pnh_.param("max_gap_points", strip_cfg_.max_gap_points, 1);
    pnh_.param("front_only", scan_window_cfg_.front_only, true);
    pnh_.param("front_min_x", scan_window_cfg_.front_min_x, 0.0);
    pnh_.param("detector_max_base_range_m", scan_window_cfg_.max_base_range_m, 2.4);
    pnh_.param("detector_max_abs_y_m", scan_window_cfg_.max_abs_y_m, 0.45);

    pnh_.param<std::string>("scene_profile", scene_profile_name_, "auto");
    scene_prior_ = ScenePriorProvider::Defaults(scene_profile_name_);
    pnh_.param("mid_x_nominal_m", scene_prior_.mid_x_nominal_m, scene_prior_.mid_x_nominal_m);
    pnh_.param("mid_x_sigma_m", scene_prior_.mid_x_sigma_m, scene_prior_.mid_x_sigma_m);
    pnh_.param("mid_y_nominal_m", scene_prior_.mid_y_nominal_m, scene_prior_.mid_y_nominal_m);
    pnh_.param("mid_y_sigma_m", scene_prior_.mid_y_sigma_m, scene_prior_.mid_y_sigma_m);
    pnh_.param("width_nominal_m", scene_prior_.width_nominal_m, scene_prior_.width_nominal_m);
    pnh_.param("width_sigma_m", scene_prior_.width_sigma_m, scene_prior_.width_sigma_m);
    pnh_.param("bias_sigma_m", scene_prior_.bias_sigma_m, scene_prior_.bias_sigma_m);
    pnh_.param("bias_diff_sigma_m",
               scene_prior_.bias_diff_sigma_m, scene_prior_.bias_diff_sigma_m);
    pnh_.param("reject_fit_cost", scene_prior_.reject_fit_cost, scene_prior_.reject_fit_cost);
    pnh_.param("min_pair_quality", scene_prior_.min_quality, scene_prior_.min_quality);
    {
      double deg = ToDeg(scene_prior_.yaw_prior_sigma_rad);
      pnh_.param("yaw_prior_sigma_deg", deg, deg);
      scene_prior_.yaw_prior_sigma_rad = deg * M_PI / 180.0;
    }
    {
      double deg = ToDeg(scene_prior_.freeze_yaw_sigma_rad);
      pnh_.param("freeze_yaw_sigma_deg", deg, deg);
      scene_prior_.freeze_yaw_sigma_rad = deg * M_PI / 180.0;
    }
    strip_interval_cfg_.width_nominal_m = scene_prior_.width_nominal_m;
    strip_interval_cfg_.width_sigma_m = scene_prior_.width_sigma_m;
    pnh_.param("interval_d_center_sigma_m",
               strip_interval_cfg_.d_center_sigma_m, 0.015);
    pnh_.param("interval_skew_sigma_m",
               strip_interval_cfg_.skew_sigma_m, 0.02);
    pnh_.param("interval_max_abs_d_center_m",
               strip_interval_cfg_.max_abs_d_center_m, 0.08);
    pnh_.param("interval_max_abs_skew_n_m",
               strip_interval_cfg_.max_abs_skew_n_m, 0.03);

    pnh_.param("support_line_window_m",
               support_line_cfg_.support_line_window_m, 0.60);
    pnh_.param("support_line_neighbor_margin_m",
               support_line_cfg_.support_line_neighbor_margin_m, 0.12);
    pnh_.param("support_line_sigma_n_m",
               support_line_cfg_.support_line_sigma_n_m, 0.015);
    pnh_.param("support_line_huber_delta",
               support_line_cfg_.support_line_huber_delta, 0.03);
    pnh_.param("support_line_min_points",
               support_line_cfg_.support_line_min_points, 8);
    pnh_.param("support_line_min_span_m",
               support_line_cfg_.support_line_min_span_m, 0.25);
    pnh_.param("support_line_min_condition_ratio",
               support_line_cfg_.support_line_min_condition_ratio, 8.0);
    pnh_.param("support_line_blend_ratio",
               support_line_cfg_.support_line_blend_ratio, 0.35);

    pnh_.param("pair_joint_max_iterations", pair_joint_cfg_.max_iterations, 12);
    pnh_.param("pair_joint_lm_lambda", pair_joint_cfg_.lm_lambda, 1e-3);
    pnh_.param("pair_joint_numeric_diff_eps",
               pair_joint_cfg_.numeric_diff_eps, 1e-4);
    pnh_.param("pair_joint_cost_tolerance",
               pair_joint_cfg_.cost_tolerance, 1e-5);
    pnh_.param("pair_detect_max_strip_range_m",
               pair_joint_cfg_.max_strip_range_m, 2.4);
    pnh_.param("pair_detect_max_separation_m",
               pair_joint_cfg_.max_pair_separation_m, 0.4);
    pnh_.param("bootstrap_sep_sigma_m", bootstrap_sep_sigma_m_, 0.015);
    pnh_.param("bootstrap_sep_update_alpha", bootstrap_sep_update_alpha_, 0.20);

    pnh_.param("pair_distance_tol", associator_cfg_.pair_distance_tol, 0.03);
    pnh_.param("prior_point_gate", associator_cfg_.prior_point_gate, 0.80);
    pnh_.param("midpoint_gate", associator_cfg_.midpoint_gate, 0.60);
    {
      double deg = 20.0;
      pnh_.param("heading_gate_deg", deg, 20.0);
      associator_cfg_.heading_gate_rad = deg * M_PI / 180.0;
    }
    pnh_.param("max_prior_translation_delta",
               associator_cfg_.max_prior_translation_delta, 0.70);
    {
      double deg = 25.0;
      pnh_.param("max_prior_yaw_delta_deg", deg, 25.0);
      associator_cfg_.max_prior_yaw_delta = deg * M_PI / 180.0;
    }

    pnh_.param("dr_timeout_sec", dr_cfg_.timeout_sec, 1.0);
    pnh_.param("dr_max_distance", dr_cfg_.max_distance, 0.40);
    {
      double deg = 20.0;
      pnh_.param("dr_max_yaw_deg", deg, 20.0);
      dr_cfg_.max_yaw_rad = deg * M_PI / 180.0;
    }
    pnh_.param("dr_max_linear_sample_age_sec",
               dr_cfg_.max_linear_sample_age_sec, 0.25);
    pnh_.param("dr_max_angular_sample_age_sec",
               dr_cfg_.max_angular_sample_age_sec, 0.25);

    pnh_.param("filter_translation_sigma_ref_m",
               pose_filter_cfg_.translation_sigma_ref_m, 0.010);
    {
      double deg = 0.50;
      pnh_.param("filter_yaw_sigma_ref_deg", deg, 0.50);
      pose_filter_cfg_.yaw_sigma_ref_rad = deg * M_PI / 180.0;
    }
    pnh_.param("filter_translation_gain_min",
               pose_filter_cfg_.translation_gain_min, 0.20);
    pnh_.param("filter_translation_gain_max",
               pose_filter_cfg_.translation_gain_max, 0.90);
    pnh_.param("filter_yaw_gain_min",
               pose_filter_cfg_.yaw_gain_min, 0.20);
    pnh_.param("filter_yaw_gain_max",
               pose_filter_cfg_.yaw_gain_max, 0.90);
    pnh_.param("low_speed_scale",
               pose_filter_cfg_.low_speed_gain_scale, 0.65);
    pnh_.param("pair_change_direct_accept", pair_change_direct_accept_, true);
  }

  void InitRosInterfaces() {
    scan_sub_ = nh_.subscribe(scan_topic_, 1,
                              &ReflectorLocalizationNode::ScanCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 100,
                             &ReflectorLocalizationNode::ImuCallback, this);
    wheel_sub_ = nh_.subscribe(wheel_odom_topic_, 100,
                               &ReflectorLocalizationNode::WheelOdomCallback, this);
    if (!prior_odom_topic_.empty()) {
      prior_sub_ = nh_.subscribe(prior_odom_topic_, 20,
                                 &ReflectorLocalizationNode::PriorOdomCallback, this);
    }
    if (!loc_manager_map_topic_.empty()) {
      map_event_sub_ = nh_.subscribe(loc_manager_map_topic_, 1,
                                     &ReflectorLocalizationNode::MapEventCallback,
                                     this);
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    status_pub_ = nh_.advertise<std_msgs::UInt8>(status_topic_, 10);

    srv_algo_enable_ =
        pnh_.advertiseService("algo_enable",
                              &ReflectorLocalizationNode::OnAlgoEnable, this);
    srv_algo_switch_map_ =
        pnh_.advertiseService("algo_switch_map",
                              &ReflectorLocalizationNode::OnAlgoSwitchMap, this);
    srv_algo_set_initial_pose_ =
        pnh_.advertiseService("algo_set_initial_pose",
                              &ReflectorLocalizationNode::OnAlgoSetInitialPose,
                              this);
  }

  bool RegisterToLocManager() {
    loc_manager::RegistLoc srv;
    srv.request.name = algo_name_;
    srv.request.need_mask = static_cast<uint32_t>(std::max(0, need_mask_for_get_map_));
    srv.request.capability_mask = 0u;
    srv.request.enable_srv = pnh_.resolveName("algo_enable");
    srv.request.switch_map_srv = pnh_.resolveName("algo_switch_map");
    srv.request.set_initial_pose_srv = pnh_.resolveName("algo_set_initial_pose");
    srv.request.pose_topic = odom_pub_.getTopic().empty() ? odom_topic_ : odom_pub_.getTopic();
    srv.request.status_topic = status_pub_.getTopic();

    ros::ServiceClient c =
        nh_.serviceClient<loc_manager::RegistLoc>(loc_manager_regist_srv_);
    if (!c.exists()) c.waitForExistence(ros::Duration(1.0));
    if (!c.call(srv)) {
      ROS_WARN_STREAM("[REF] regist_loc call failed: " << loc_manager_regist_srv_);
      return false;
    }
    if (!srv.response.ok) {
      ROS_WARN_STREAM("[REF] regist_loc rejected: " << srv.response.message);
      return false;
    }
    return true;
  }

  bool FetchMapJsonFromServer(const std::string& map, int layer,
                              std::string* out_json_path,
                              std::string* out_err,
                              std::string* out_resolved_map,
                              int* out_resolved_layer) {
    if (out_err) out_err->clear();
    if (out_json_path) out_json_path->clear();
    if (out_resolved_map) out_resolved_map->clear();
    if (out_resolved_layer) *out_resolved_layer = 0;

    map_server::GetMap srv;
    srv.request.map = TrimCopy(map);
    srv.request.layer = layer;
    srv.request.need_mask = static_cast<uint32_t>(std::max(0, need_mask_for_get_map_));

    ros::ServiceClient c =
        nh_.serviceClient<map_server::GetMap>(map_server_get_map_srv_);
    if (!c.exists()) c.waitForExistence(ros::Duration(1.0));
    if (!c.call(srv)) {
      if (out_err) *out_err = "call failed: " + map_server_get_map_srv_;
      return false;
    }
    if (!srv.response.ok) {
      if (out_err) *out_err = srv.response.message;
      return false;
    }

    if (out_json_path) *out_json_path = srv.response.json_path;
    if (out_resolved_map) *out_resolved_map = TrimCopy(srv.response.map);
    if (out_resolved_layer) *out_resolved_layer = srv.response.layer;
    return true;
  }

  bool LoadMapFromJson(const std::string& json_path,
                       const std::string& resolved_map,
                       int resolved_layer,
                       std::string* out_err) {
    if (out_err) out_err->clear();
    if (json_path.empty()) {
      if (out_err) *out_err = "json_path is empty";
      return false;
    }

    ReflectorMap new_map;
    if (!new_map.Load(json_path, min_observations_, max_rms_radius_)) {
      if (out_err) *out_err = "failed to load reflector map: " + json_path;
      return false;
    }

    map_ = std::move(new_map);
    has_map_ = true;
    map_json_path_ = json_path;
    if (!resolved_map.empty()) map_name_ = resolved_map;
    if (resolved_layer > 0) map_layer_ = resolved_layer;

    latest_ref_pose_ = Pose2();
    has_prev_observed_pair_geometry_ = false;
    has_locked_pair_ = false;
    locked_map_idx_a_ = -1;
    locked_map_idx_b_ = -1;
    has_runtime_bootstrap_prior_ = false;
    runtime_bootstrap_sep_nominal_m_ = 0.0;
    latest_support_line_ = SupportLineEstimate();
    state_ = has_prior_pose_ ? State::SEARCHING : State::INIT;
    return true;
  }

  bool PreloadActiveMap(const std::string& reason, std::string* out_err) {
    if (out_err) out_err->clear();
    std::string json_path;
    std::string resolved_map;
    int resolved_layer = 0;
    std::string fetch_err;

    if (!FetchMapJsonFromServer("", 0, &json_path, &fetch_err,
                                &resolved_map, &resolved_layer)) {
      if (out_err) *out_err = fetch_err;
      return false;
    }
    if (json_path == map_json_path_ && has_map_) return true;

    std::string load_err;
    if (!LoadMapFromJson(json_path, resolved_map, resolved_layer, &load_err)) {
      if (out_err) *out_err = load_err;
      return false;
    }

    ROS_INFO_STREAM("[REF] preloaded active map from " << reason
                    << ": map=" << map_name_ << " layer=" << map_layer_);
    return true;
  }

  bool OnAlgoEnable(loc_manager::AlgoEnable::Request& req,
                    loc_manager::AlgoEnable::Response& res) {
    if (!req.enable) {
      algo_enabled_ = false;
      state_ = State::INIT;
      has_runtime_bootstrap_prior_ = false;
      runtime_bootstrap_sep_nominal_m_ = 0.0;
      latest_support_line_ = SupportLineEstimate();
      PublishStatus();
      res.ok = true;
      res.message = "standby";
      return true;
    }

    if (!has_map_) {
      std::string err;
      if (!PreloadActiveMap("algo_enable", &err) && !json_path_.empty()) {
        (void)LoadMapFromJson(json_path_, "", 0, &err);
      }
      if (!has_map_) {
        res.ok = false;
        res.message = err.empty() ? "map not ready" : err;
        return true;
      }
    }

    algo_enabled_ = true;
    state_ = has_prior_pose_ ? State::SEARCHING : State::INIT;
    PublishStatus();
    res.ok = true;
    res.message = has_prior_pose_ ? "active" : "active_waiting_initial_pose";
    return true;
  }

  bool OnAlgoSwitchMap(loc_manager::AlgoSwitchMap::Request& req,
                       loc_manager::AlgoSwitchMap::Response& res) {
    std::string map = TrimCopy(req.map);
    int layer = req.layer;

    if (layer <= 0) {
      std::string base;
      int layer_from_name = 0;
      if (ParseMapLayerSuffix(map, &base, &layer_from_name)) {
        map = base;
        layer = layer_from_name;
      }
    }
    if (!map.empty() && layer <= 0) layer = 1;

    std::string err;
    if (map.empty()) {
      if (!PreloadActiveMap("algo_switch_map(active)", &err)) {
        res.ok = false;
        res.message = err;
        return true;
      }
    } else {
      std::string json_path;
      std::string resolved_map;
      int resolved_layer = 0;
      if (!FetchMapJsonFromServer(map, layer, &json_path, &err,
                                  &resolved_map, &resolved_layer)) {
        res.ok = false;
        res.message = err;
        return true;
      }
      if (!LoadMapFromJson(json_path, resolved_map, resolved_layer, &err)) {
        res.ok = false;
        res.message = err;
        return true;
      }
    }

    res.ok = true;
    res.message = "ok";
    PublishStatus();
    return true;
  }

  bool OnAlgoSetInitialPose(loc_manager::AlgoSetInitialPose::Request& req,
                            loc_manager::AlgoSetInitialPose::Response& res) {
    prior_pose_.x = req.x;
    prior_pose_.y = req.y;
    prior_pose_.yaw = WrapAngle(req.yaw);
    prior_pose_.valid = true;
    has_prior_pose_ = true;

    latest_ref_pose_ = Pose2();
    has_prev_observed_pair_geometry_ = false;
    has_locked_pair_ = false;
    locked_map_idx_a_ = -1;
    locked_map_idx_b_ = -1;
    has_runtime_bootstrap_prior_ = false;
    runtime_bootstrap_sep_nominal_m_ = 0.0;
    latest_support_line_ = SupportLineEstimate();
    dr_->Reset(prior_pose_, ros::Time::now());

    state_ = algo_enabled_ ? State::SEARCHING : State::INIT;
    PublishStatus();

    res.ok = true;
    res.message = "ok";
    return true;
  }

  void MapEventCallback(const std_msgs::StringConstPtr&) {
    if (!auto_reload_on_map_event_) return;
    std::string err;
    if (!PreloadActiveMap("map_event", &err)) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[REF] map reload failed: " << err);
    }
  }

  void PriorOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    prior_pose_.x = msg->pose.pose.position.x;
    prior_pose_.y = msg->pose.pose.position.y;
    prior_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    prior_pose_.valid = true;
    has_prior_pose_ = true;
    if (!latest_ref_pose_.valid) {
      dr_->Reset(prior_pose_, msg->header.stamp);
    }
    if (algo_enabled_ && state_ == State::INIT) {
      state_ = State::SEARCHING;
      PublishStatus();
    }
  }

  void ImuCallback(const sensor_msgs::ImuConstPtr& msg) {
    latest_angular_vel_ = msg->angular_velocity.z;
    dr_->UpdateAngularVelocity(latest_angular_vel_, msg->header.stamp);
  }

  void WheelOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    latest_linear_vel_ = msg->twist.twist.linear.x;
    dr_->UpdateLinearVelocity(latest_linear_vel_, msg->header.stamp);
  }

  bool EnsureScanToBaseTf(const sensor_msgs::LaserScan& scan) {
    try {
      scan_to_base_tf_ = tf_buffer_.lookupTransform(
          base_frame_, scan.header.frame_id,
          scan.header.stamp.isZero() ? ros::Time(0) : scan.header.stamp,
          ros::Duration(0.05));
      return true;
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(1.0, "[REF] TF %s -> %s not ready: %s",
                        scan.header.frame_id.c_str(), base_frame_.c_str(),
                        e.what());
      state_ = State::SEARCHING;
      return false;
    }
  }

  Pose2 BuildMatchPrior(const ros::Time& stamp) {
    if (latest_ref_pose_.valid) {
      Pose2 predicted;
      double elapsed = 0.0;
      double distance = 0.0;
      double yaw_change = 0.0;
      if (dr_->ExtrapolateTo(stamp, &predicted, &elapsed, &distance, &yaw_change)) {
        predicted.valid = true;
        return predicted;
      }
      return latest_ref_pose_;
    }
    return prior_pose_;
  }

  double MotionLevel() const {
    return std::max(std::fabs(latest_linear_vel_),
                    0.20 * std::fabs(latest_angular_vel_));
  }

  PairJointRuntimePrior BuildPairRuntimePrior() const {
    PairJointRuntimePrior prior;
    prior.valid = has_runtime_bootstrap_prior_;
    prior.separation_nominal_m = runtime_bootstrap_sep_nominal_m_;
    prior.separation_sigma_m = bootstrap_sep_sigma_m_;
    return prior;
  }

  void ResetRuntimeBootstrapPrior() {
    has_runtime_bootstrap_prior_ = false;
    runtime_bootstrap_sep_nominal_m_ = 0.0;
  }

  void UpdateRuntimeBootstrapPrior(double separation_m, bool hard_reset) {
    if (!std::isfinite(separation_m) || separation_m <= 0.05) return;
    if (hard_reset || !has_runtime_bootstrap_prior_) {
      runtime_bootstrap_sep_nominal_m_ = separation_m;
      has_runtime_bootstrap_prior_ = true;
      return;
    }

    const double alpha = Clamp(bootstrap_sep_update_alpha_, 0.01, 1.0);
    runtime_bootstrap_sep_nominal_m_ =
        (1.0 - alpha) * runtime_bootstrap_sep_nominal_m_ + alpha * separation_m;
    has_runtime_bootstrap_prior_ = true;
  }

  void HandleNoMatch(const ros::Time& stamp,
                     size_t strip_count,
                     const std::string& reason,
                     const PairAssociationDebugInfo& association_debug,
                     const Pose2& match_prior) {
    const int old_locked_a = locked_map_idx_a_;
    const int old_locked_b = locked_map_idx_b_;
    Pose2 predicted;
    double elapsed = 0.0;
    double distance = 0.0;
    double yaw_change = 0.0;
    if (latest_ref_pose_.valid &&
        dr_->ExtrapolateTo(stamp, &predicted, &elapsed, &distance, &yaw_change)) {
      latest_ref_pose_ = predicted;
      latest_ref_pose_.valid = true;
      state_ = State::DR_ONLY;
      PoseEstimate dr_estimate;
      dr_estimate.pose_map_base = latest_ref_pose_;
      dr_estimate.pose_map_base.valid = true;
      dr_estimate.dr_mode = true;
      PublishOdometry(dr_estimate, stamp);
      if (debug_log_match_events_) {
        ROS_WARN_STREAM_THROTTLE(
            0.5,
            "[REF] " << reason
                     << " -> DR_ONLY lock_pair=" << PairString(old_locked_a, old_locked_b)
                     << " strips=" << strip_count
                     << " dr=(elapsed=" << elapsed
                     << "s dist=" << distance
                     << " yaw_deg=" << ToDeg(yaw_change) << ")"
                     << " prior=" << PoseString(match_prior)
                     << " assoc={" << FormatAssociationDebug(association_debug) << "}");
      }
      has_prev_observed_pair_geometry_ = false;
      return;
    }

    has_locked_pair_ = false;
    locked_map_idx_a_ = -1;
    locked_map_idx_b_ = -1;
    ResetRuntimeBootstrapPrior();
    state_ = latest_ref_pose_.valid ? State::LOST : State::SEARCHING;
    if (debug_log_match_events_) {
      ROS_WARN_STREAM_THROTTLE(
          0.5,
          "[REF] " << reason
                   << " -> " << (state_ == State::LOST ? "LOST" : "SEARCHING")
                   << " last_lock=" << PairString(old_locked_a, old_locked_b)
                   << " strips=" << strip_count
                   << " prior=" << PoseString(match_prior)
                   << " assoc={" << FormatAssociationDebug(association_debug) << "}");
    }
    has_prev_observed_pair_geometry_ = false;
  }

  void LogSparseStripState(const ScanWindowDebugInfo& scan_debug,
                           const StripEdgeDebugInfo& strip_debug,
                           const std::vector<StripEdgeObservation>& strips) const {
    if (!debug_log_observed_pair_geometry_ || strips.size() >= 2) return;

    std::ostringstream oss;
    oss << "[REF] sparse_strip_observation scan_pts=" << scan_debug.scan_point_count
        << " valid_range=" << scan_debug.valid_range_count
        << " roi_beams=" << scan_debug.roi_beam_count
        << " segments=" << strip_debug.segment_count
        << " accepted=" << strip_debug.accepted_count
        << " rejected_size=" << strip_debug.rejected_size_count;

    for (size_t i = 0; i < strips.size(); ++i) {
      oss << " strip" << i
          << "{center=" << PointString(strips[i].center_base)
          << " width=" << strips[i].observed_width
          << " q=" << strips[i].segment_quality
          << "}";
    }
    ROS_WARN_STREAM_THROTTLE(0.5, oss.str());
  }

  void LogPairMeasurementGeometry(const PairMeasurement& measurement,
                                  int match_pair_a,
                                  int match_pair_b,
                                  bool pair_changed) {
    if (!debug_log_observed_pair_geometry_ || !measurement.valid) return;

    const double axis_heading = measurement.heading_base;
    std::ostringstream oss;
    oss << "[REF] pair_measurement pair=" << PairString(match_pair_a, match_pair_b)
        << " mid_base=" << PointString(measurement.midpoint_base)
        << " head_base_deg=" << ToDeg(axis_heading)
        << " sep=" << measurement.state.separation
        << " bias=(" << measurement.state.bias_left << ","
        << measurement.state.bias_right << ")"
        << " sigma_t=" << measurement.translation_sigma_m
        << " sigma_head_deg=" << ToDeg(measurement.yaw_sigma_rad)
        << " fit_cost=" << measurement.fit_cost
        << " q=" << measurement.quality
        << " res_strip=(" << measurement.residual_strip_left << ","
        << measurement.residual_strip_right << ")"
        << " res_line=" << measurement.residual_support_line
        << " freeze_yaw=" << (measurement.freeze_yaw ? 1 : 0);

    if (has_prev_observed_pair_geometry_ &&
        prev_observed_pair_a_ == match_pair_a &&
        prev_observed_pair_b_ == match_pair_b &&
        !pair_changed) {
      const Point2 dmid{measurement.midpoint_base.x - prev_observed_midpoint_base_.x,
                        measurement.midpoint_base.y - prev_observed_midpoint_base_.y};
      const double dhead = WrapAngle(axis_heading - prev_observed_heading_base_);
      oss << " dmid_base=" << PointString(dmid)
          << " dhead_deg=" << ToDeg(dhead);
    } else {
      oss << " dmid_base=<reset> dhead_deg=<reset>";
    }

    ROS_INFO_STREAM_THROTTLE(0.5, oss.str());

    prev_observed_pair_a_ = match_pair_a;
    prev_observed_pair_b_ = match_pair_b;
    prev_observed_midpoint_base_ = measurement.midpoint_base;
    prev_observed_heading_base_ = axis_heading;
    has_prev_observed_pair_geometry_ = true;
  }

  void LogInputDiagnostics(const sensor_msgs::LaserScan& scan,
                           const ros::Time& stamp,
                           const ScanWindowDebugInfo& scan_debug) {
    if (!debug_log_observed_pair_geometry_) return;

    std::ostringstream oss;
    oss << "[REF] input_diag stamp=" << stamp.toSec()
        << " scan_dt=";
    if (has_prev_scan_diag_stamp_) {
      oss << (stamp - prev_scan_diag_stamp_).toSec();
    } else {
      oss << "<reset>";
    }
    oss << " linear_v=" << latest_linear_vel_
        << " angular_v=" << latest_angular_vel_
        << " raw_scan_pts=" << scan.ranges.size()
        << " valid_range=" << scan_debug.valid_range_count
        << " roi_beams=" << scan_debug.roi_beam_count
        << " angle_inc_deg=" << ToDeg(scan.angle_increment);
    ROS_INFO_STREAM_THROTTLE(0.5, oss.str());

    prev_scan_diag_stamp_ = stamp;
    has_prev_scan_diag_stamp_ = true;
  }

  double StripTangentYaw(const StripEdgeObservation& strip) const {
    return Heading(strip.left.base, strip.right.base);
  }

  void LogStripDiagnostics(const std::vector<StripEdgeObservation>& strips) {
    if (!debug_log_observed_pair_geometry_) return;

    std::ostringstream oss;
    oss << "[REF] strip_diag count=" << strips.size();
    const size_t max_strips = std::min<size_t>(strips.size(), 3);
    for (size_t i = 0; i < max_strips; ++i) {
      const auto& strip = strips[i];
      const double tangent_yaw = StripTangentYaw(strip);
      oss << " s" << i
          << "{center_b=" << PointString(strip.center_base)
          << " left_l=" << PointString(strip.left.laser)
          << " right_l=" << PointString(strip.right.laser)
          << " left_b=" << PointString(strip.left.base)
          << " right_b=" << PointString(strip.right.base)
          << " tan_deg=" << ToDeg(tangent_yaw)
          << " width=" << strip.observed_width
          << " asym=" << strip.asymmetry_score
          << " curv=" << strip.profile_curvature;
      if (has_prev_strip_trace_ && i < prev_strip_traces_.size()) {
        const Point2 dcenter{strip.center_base.x - prev_strip_traces_[i].center_base.x,
                             strip.center_base.y - prev_strip_traces_[i].center_base.y};
        const double dyaw = WrapAngle(tangent_yaw - prev_strip_traces_[i].tangent_yaw);
        const double dwidth = strip.observed_width - prev_strip_traces_[i].width;
        oss << " dcenter=" << PointString(dcenter)
            << " dyaw_deg=" << ToDeg(dyaw)
            << " dwidth=" << dwidth;
      } else {
        oss << " dcenter=<reset> dyaw_deg=<reset> dwidth=<reset>";
      }
      oss << "}";
    }
    ROS_INFO_STREAM_THROTTLE(0.5, oss.str());

    prev_strip_traces_.clear();
    prev_strip_traces_.reserve(strips.size());
    for (const auto& strip : strips) {
      StripTrace trace;
      trace.center_base = strip.center_base;
      trace.tangent_yaw = StripTangentYaw(strip);
      trace.width = strip.observed_width;
      prev_strip_traces_.push_back(trace);
    }
    has_prev_strip_trace_ = true;
  }

  void LogSupportLineDiagnostics(const SupportLineEstimate& support_line,
                                 const SupportLineFitterDebug& support_debug) {
    if (!debug_log_observed_pair_geometry_) return;

    std::ostringstream oss;
    oss << "[REF] support_line_diag valid=" << (support_line.valid ? 1 : 0)
        << " yaw_deg=" << ToDeg(support_line.yaw)
        << " sigma_yaw_deg=" << ToDeg(support_line.sigma_yaw_rad)
        << " offset=" << SupportLineOffset(support_line)
        << " sigma_offset=" << support_line.sigma_offset_m
        << " fit_res=" << support_line.normal_rms
        << " span=" << support_line.tangent_span_m
        << " cond=" << support_line.condition_ratio
        << " quality=" << support_line.quality
        << " cand_pts=" << support_debug.candidate_points_base.size()
        << " inliers=" << support_debug.inlier_points_base.size()
        << " source=" << static_cast<int>(support_line.source)
        << " yaw_frozen=" << (support_line.yaw_frozen ? 1 : 0);
    ROS_INFO_STREAM_THROTTLE(0.5, oss.str());
  }

  const PairJointCandidateDebug* FindCandidateDebug(const PairJointDebugInfo& debug_info,
                                                    int strip_idx_a,
                                                    int strip_idx_b) const {
    for (const auto& candidate : debug_info.candidates) {
      if (candidate.strip_idx_a == strip_idx_a &&
          candidate.strip_idx_b == strip_idx_b) {
        return &candidate;
      }
    }
    return nullptr;
  }

  void LogYawChainDiagnostics(const std::vector<StripEdgeObservation>& strips,
                              const SupportLineEstimate& support_line,
                              const PairMeasurement& measurement,
                              const PairJointDebugInfo& joint_debug) {
    if (!debug_log_observed_pair_geometry_ || !measurement.valid) return;
    if (measurement.strip_idx_left < 0 || measurement.strip_idx_right < 0 ||
        measurement.strip_idx_left >= static_cast<int>(strips.size()) ||
        measurement.strip_idx_right >= static_cast<int>(strips.size())) {
      return;
    }

    const auto& strip_left = strips[static_cast<size_t>(measurement.strip_idx_left)];
    const auto& strip_right = strips[static_cast<size_t>(measurement.strip_idx_right)];
    const double strip_left_yaw = StripTangentYaw(strip_left);
    const double strip_right_yaw = StripTangentYaw(strip_right);
    const PairJointCandidateDebug* candidate =
        FindCandidateDebug(joint_debug, measurement.strip_idx_left, measurement.strip_idx_right);

    std::ostringstream oss;
    oss << "[REF] yaw_chain strips=(" << measurement.strip_idx_left
        << "," << measurement.strip_idx_right << ")"
        << " strip_tan_deg=(" << ToDeg(strip_left_yaw)
        << "," << ToDeg(strip_right_yaw) << ")"
        << " support_line_yaw_deg="
        << (support_line.valid ? ToDeg(support_line.yaw) : std::numeric_limits<double>::quiet_NaN());
    if (candidate != nullptr) {
      oss << " directed_init_yaw_deg=" << (ToDeg(support_line.yaw) + candidate->init_delta_yaw_deg)
          << " init_fit=" << candidate->init_fit_cost;
    } else {
      oss << " directed_init_yaw_deg=<missing> init_fit=<missing>";
    }
    oss << " final_head_deg=" << ToDeg(measurement.heading_base)
        << " d_final_line_deg="
        << (support_line.valid ? ToDeg(WrapAngle(measurement.heading_base - support_line.yaw))
                               : std::numeric_limits<double>::quiet_NaN())
        << " d_final_strip_deg=("
        << ToDeg(WrapAngle(measurement.heading_base - strip_left_yaw)) << ","
        << ToDeg(WrapAngle(measurement.heading_base - strip_right_yaw)) << ")";
    ROS_INFO_STREAM_THROTTLE(0.5, oss.str());
  }

  std::string FormatPairJointDebug(const PairJointDebugInfo& debug_info) const {
    std::ostringstream oss;
    oss << "candidates=" << debug_info.candidates.size();
    const size_t max_candidates = std::min<size_t>(debug_info.candidates.size(), 8);
    for (size_t i = 0; i < max_candidates; ++i) {
      const auto& c = debug_info.candidates[i];
      oss << " c" << i
          << "{idx=" << c.strip_idx_a << "," << c.strip_idx_b
          << " interval=(" << c.left_s_center << ","
          << c.right_s_center << ","
          << c.left_width_s << ","
          << c.right_width_s << ","
          << c.left_d_center << ","
          << c.right_d_center << ","
          << c.left_skew_n << ","
          << c.right_skew_n << ")"
          << " init_state=(" << c.init_mid_s << ","
          << c.init_mid_d << "," << c.init_delta_yaw_deg << "deg,"
          << c.init_sep << "," << c.init_bias_l << "," << c.init_bias_r << ")"
          << " final_state=(" << c.final_mid_s << ","
          << c.final_mid_d << "," << c.final_delta_yaw_deg << "deg,"
          << c.final_sep << "," << c.final_bias_l << "," << c.final_bias_r << ")"
          << " init_fit=" << c.init_fit_cost
          << " fit=" << c.fit_cost
          << " q=" << c.quality
          << " cost(interval/line/mid/sep/yaw/bias)="
          << c.interval_cost << "/"
          << c.support_line_cost << "/"
          << c.mid_prior_cost << "/"
          << c.runtime_sep_cost << "/"
          << c.yaw_prior_cost << "/"
          << c.bias_cost
          << " accepted=" << (c.accepted ? 1 : 0)
          << " reason=" << c.rejection_reason
          << "}";
    }
    return oss.str();
  }

  void ScanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
    if (!algo_enabled_) {
      state_ = State::INIT;
      PublishStatus();
      return;
    }
    if (!has_map_) {
      state_ = State::INIT;
      PublishStatus();
      return;
    }
    if (!has_prior_pose_) {
      state_ = State::INIT;
      PublishStatus();
      return;
    }
    if (!EnsureScanToBaseTf(*msg)) {
      PublishStatus();
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const Pose2 match_prior = BuildMatchPrior(stamp);

    BeamCloud cloud;
    ScanWindowDebugInfo scan_debug;
    std::string error;
    if (!scan_window_builder_->Build(*msg, scan_to_base_tf_, &cloud, &error, &scan_debug)) {
      state_ = State::SEARCHING;
      PublishStatus();
      ROS_WARN_THROTTLE(1.0, "[REF] scan_window_builder failed: %s", error.c_str());
      return;
    }
    LogInputDiagnostics(*msg, stamp, scan_debug);

    std::vector<StripEdgeObservation> strips;
    StripEdgeDebugInfo strip_debug;
    if (!strip_edge_extractor_->Extract(cloud, &strips, &error, &strip_debug)) {
      PairAssociationDebugInfo association_debug;
      LogSparseStripState(scan_debug, strip_debug, strips);
      HandleNoMatch(stamp, 0, "strip_edge_extractor_failed", association_debug, match_prior);
      PublishStatus();
      return;
    }
    LogSparseStripState(scan_debug, strip_debug, strips);
    LogStripDiagnostics(strips);

    SupportLineEstimate support_line;
    SupportLineFitterDebug support_debug;
    SupportLineFitterInput support_input;
    support_input.cloud = &cloud;
    support_input.raw_strips = &strips;
    support_input.prior_base_pose = has_prior_pose_ ? &prior_pose_ : nullptr;
    support_input.prior_support_line =
        latest_support_line_.valid ? &latest_support_line_ : nullptr;
    support_input.scene_prior = &scene_prior_;
    if (!support_line_fitter_->Fit(support_input, &support_line,
                                   &support_debug, &error)) {
      PairAssociationDebugInfo association_debug;
      HandleNoMatch(stamp, strips.size(), "support_line_fitter_failed",
                    association_debug, match_prior);
      PublishStatus();
      return;
    }
    LogSupportLineDiagnostics(support_line, support_debug);
    latest_support_line_ = support_line;

    std::vector<StripIntervalObservation> strip_intervals;
    strip_intervals.reserve(strips.size());
    for (size_t i = 0; i < strips.size(); ++i) {
      StripIntervalObservation interval;
      StripProjectorDebug projector_debug;
      StripProjectorInput projector_input;
      projector_input.strip = &strips[i];
      projector_input.support_line = &support_line;
      if (!strip_interval_projector_->Project(projector_input, &interval,
                                              &projector_debug, &error)) {
        continue;
      }
      interval.strip_id = static_cast<int>(i);
      strip_intervals.push_back(interval);
    }

    PairMeasurement measurement;
    PairJointDebugInfo joint_debug;
    const PairJointRuntimePrior runtime_prior = BuildPairRuntimePrior();
    if (!pair_joint_fitter_->Fit(strip_intervals, support_line, scene_prior_, runtime_prior,
                                 &measurement, &error, &joint_debug)) {
      PairAssociationDebugInfo association_debug;
      HandleNoMatch(stamp, strips.size(), "pair_joint_fitter_failed",
                    association_debug, match_prior);
      if (debug_log_match_events_) {
        ROS_WARN_STREAM_THROTTLE(
            0.5,
            "[REF] pair_joint_fit failed: " << error
            << " runtime_prior={valid=" << (runtime_prior.valid ? 1 : 0)
            << " sep=" << runtime_prior.separation_nominal_m
            << " sigma=" << runtime_prior.separation_sigma_m
            << "} joint_dbg={" << FormatPairJointDebug(joint_debug) << "}");
      }
      PublishStatus();
      if (enable_markers_ && visualizer_) {
        PairAssociationResult empty_assoc;
        PoseEstimate empty_pose;
        empty_pose.pose_map_base = latest_ref_pose_;
        visualizer_->Publish(strips, support_line, measurement, empty_assoc, map_,
                             empty_pose, stamp);
      }
      return;
    }
    LogYawChainDiagnostics(strips, support_line, measurement, joint_debug);

    const int pref_a = has_locked_pair_ ? locked_map_idx_a_ : -1;
    const int pref_b = has_locked_pair_ ? locked_map_idx_b_ : -1;
    PairAssociationDebugInfo association_debug;
    const PairAssociationResult association =
        pair_map_associator_->Match(measurement, map_, match_prior,
                                    pref_a, pref_b, &association_debug);

    PoseEstimate pose_estimate;
    pose_estimate.pose_map_base = latest_ref_pose_;
    pose_estimate.pose_map_base.valid = latest_ref_pose_.valid;

    if (association.success) {
      const bool first_measurement = !latest_ref_pose_.valid;
      const int old_locked_a = locked_map_idx_a_;
      const int old_locked_b = locked_map_idx_b_;
      int match_pair_a = association.map_idx_a;
      int match_pair_b = association.map_idx_b;
      CanonicalizePairIds(&match_pair_a, &match_pair_b);
      const bool pair_changed =
          has_locked_pair_ &&
          (locked_map_idx_a_ != match_pair_a ||
           locked_map_idx_b_ != match_pair_b);
      UpdateRuntimeBootstrapPrior(association.measurement.state.separation,
                                  pair_changed || !has_runtime_bootstrap_prior_);

      PoseFilterRuntime runtime;
      runtime.first_measurement = first_measurement;
      runtime.pair_changed = pair_changed;
      runtime.pair_change_direct_accept = pair_change_direct_accept_;
      runtime.motion_level = MotionLevel();
      pose_estimate = pose_filter_->Fuse(association, match_prior, runtime);

      latest_ref_pose_ = pose_estimate.pose_map_base;
      latest_ref_pose_.valid = true;
      prior_pose_ = latest_ref_pose_;
      has_prior_pose_ = true;

      dr_->Reset(latest_ref_pose_, stamp);
      if (debug_log_match_events_ && pair_changed) {
        ROS_WARN_STREAM(
            "[REF] pair switch old="
            << PairString(old_locked_a, old_locked_b)
            << " new=" << PairString(match_pair_a, match_pair_b)
            << " q=" << association.measurement.quality
            << " bootstrap_sep=" << runtime_bootstrap_sep_nominal_m_
            << " sigma_t=" << association.measurement.translation_sigma_m
            << " sigma_y_deg=" << ToDeg(association.measurement.yaw_sigma_rad)
            << " prior=" << PoseString(match_prior)
            << " measured=" << PoseString(association.pose_map_base)
            << " fused=" << PoseString(latest_ref_pose_)
            << " assoc={" << FormatAssociationDebug(association_debug) << "}");
      } else if (debug_log_match_events_ && association_debug.fallback_to_global) {
        ROS_WARN_STREAM_THROTTLE(
            0.5,
            "[REF] preferred_search_failed reacquired="
                << PairString(match_pair_a, match_pair_b)
                << " q=" << association.measurement.quality
                << " bootstrap_sep=" << runtime_bootstrap_sep_nominal_m_
                << " prior=" << PoseString(match_prior)
                << " measured=" << PoseString(association.pose_map_base)
                << " assoc={" << FormatAssociationDebug(association_debug) << "}");
      }
      LogPairMeasurementGeometry(association.measurement, match_pair_a, match_pair_b,
                                 pair_changed);
      has_locked_pair_ = true;
      locked_map_idx_a_ = match_pair_a;
      locked_map_idx_b_ = match_pair_b;
      state_ = State::LOCKED;

      PublishOdometry(pose_estimate, stamp);

      ROS_INFO_STREAM_THROTTLE(
          0.5,
          "[REF] lock pair=(" << match_pair_a << "," << match_pair_b
                              << ") q=" << association.measurement.quality
                              << " bootstrap_sep=" << runtime_bootstrap_sep_nominal_m_
                              << " sigma_t=" << association.measurement.translation_sigma_m
                              << " sigma_y_deg="
                              << ToDeg(association.measurement.yaw_sigma_rad)
                              << " prior_dt=" << association.prior_translation_delta
                              << " prior_dyaw_deg="
                              << ToDeg(association.prior_yaw_delta)
                              << " fit_cost=" << association.measurement.fit_cost
                              << " freeze_yaw="
                              << (association.measurement.freeze_yaw ? 1 : 0)
                              << " pose=(" << latest_ref_pose_.x << ","
                              << latest_ref_pose_.y << ","
                              << ToDeg(latest_ref_pose_.yaw) << "deg)");
    } else {
      HandleNoMatch(stamp, strips.size(), "pair_association_failed",
                    association_debug, match_prior);
      pose_estimate.pose_map_base = latest_ref_pose_;
      pose_estimate.pose_map_base.valid = latest_ref_pose_.valid;
    }

    if (enable_markers_ && visualizer_) {
      visualizer_->Publish(strips, support_line, measurement, association, map_,
                           pose_estimate, stamp);
    }
    PublishStatus();
  }

  void PublishStatus() {
    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(state_);
    status_pub_.publish(msg);
  }

  void PublishOdometry(const PoseEstimate& estimate, const ros::Time& stamp) {
    const Pose2& pose = estimate.pose_map_base;
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = latest_linear_vel_;
    odom.twist.twist.angular.z = latest_angular_vel_;
    for (double& c : odom.pose.covariance) c = 0.0;
    if (estimate.dr_mode) {
      odom.pose.covariance[0] = 0.03;
      odom.pose.covariance[7] = 0.03;
      odom.pose.covariance[35] = 0.02;
    } else {
      odom.pose.covariance[0] = estimate.covariance(0, 0);
      odom.pose.covariance[7] = estimate.covariance(1, 1);
      odom.pose.covariance[35] = estimate.covariance(2, 2);
    }

    odom_pub_.publish(odom);
    PublishMapToBaseTf(pose, stamp);
  }

  void PublishMapToBaseTf(const Pose2& pose, const ros::Time& stamp) {
    if (!publish_tf_ || !pose.valid) return;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = pose.x;
    tf_msg.transform.translation.y = pose.y;
    tf_msg.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    tf_msg.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(tf_msg);
  }

 private:
  struct StripTrace {
    Point2 center_base;
    double tangent_yaw = 0.0;
    double width = 0.0;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped scan_to_base_tf_;

  std::unique_ptr<ScanWindowBuilder> scan_window_builder_;
  std::unique_ptr<StripEdgeExtractor> strip_edge_extractor_;
  std::unique_ptr<SupportLineFitter> support_line_fitter_;
  std::unique_ptr<StripIntervalProjector> strip_interval_projector_;
  std::unique_ptr<PairJointFitter> pair_joint_fitter_;
  std::unique_ptr<PairMapAssociator> pair_map_associator_;
  std::unique_ptr<PoseFilter> pose_filter_;
  std::unique_ptr<DrExtrapolator> dr_;
  std::unique_ptr<Visualizer> visualizer_;

  ReflectorMap map_;
  ScanWindowBuilderConfig scan_window_cfg_;
  StripEdgeExtractorConfig strip_cfg_;
  SupportLineFitterConfig support_line_cfg_;
  StripIntervalProjectorConfig strip_interval_cfg_;
  PairJointFitterConfig pair_joint_cfg_;
  PairMapAssociatorConfig associator_cfg_;
  PoseFilterConfig pose_filter_cfg_;
  DrConfig dr_cfg_;
  ScenePrior scene_prior_;

  ros::Subscriber scan_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber wheel_sub_;
  ros::Subscriber prior_sub_;
  ros::Subscriber map_event_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher status_pub_;
  ros::ServiceServer srv_algo_enable_;
  ros::ServiceServer srv_algo_switch_map_;
  ros::ServiceServer srv_algo_set_initial_pose_;

  std::string json_path_;
  std::string map_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string imu_topic_;
  std::string wheel_odom_topic_;
  std::string odom_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string prior_odom_topic_;
  std::string scene_profile_name_ = "auto";

  std::string algo_name_ = "reflector_localization";
  std::string loc_manager_regist_srv_ = "/loc_manager/regist_loc";
  std::string map_server_get_map_srv_ = "/map_server/get_map";
  std::string loc_manager_map_topic_ = "/loc_manager/map";

  bool publish_tf_ = true;
  bool enable_markers_ = true;
  bool debug_log_match_events_ = true;
  bool debug_log_observed_pair_geometry_ = true;
  bool wait_for_enable_ = true;
  bool auto_reload_on_map_event_ = true;
  int need_mask_for_get_map_ = 0;

  int min_observations_ = 5;
  double max_rms_radius_ = 0.05;
  bool pair_change_direct_accept_ = true;
  double bootstrap_sep_sigma_m_ = 0.015;
  double bootstrap_sep_update_alpha_ = 0.20;

  bool algo_enabled_ = false;
  bool has_map_ = false;
  bool has_prior_pose_ = false;
  bool has_locked_pair_ = false;
  bool has_runtime_bootstrap_prior_ = false;

  int map_layer_ = 0;
  int locked_map_idx_a_ = -1;
  int locked_map_idx_b_ = -1;
  double runtime_bootstrap_sep_nominal_m_ = 0.0;
  SupportLineEstimate latest_support_line_;

  std::string map_name_;
  std::string map_json_path_;

  Pose2 prior_pose_;
  Pose2 latest_ref_pose_;
  bool has_prev_observed_pair_geometry_ = false;
  int prev_observed_pair_a_ = -1;
  int prev_observed_pair_b_ = -1;
  Point2 prev_observed_midpoint_base_;
  double prev_observed_heading_base_ = 0.0;
  bool has_prev_scan_diag_stamp_ = false;
  ros::Time prev_scan_diag_stamp_;
  bool has_prev_strip_trace_ = false;
  std::vector<StripTrace> prev_strip_traces_;

  double latest_linear_vel_ = 0.0;
  double latest_angular_vel_ = 0.0;

  State state_ = State::INIT;
};

}  // namespace reflector_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "reflector_localization_node");
  reflector_localization::ReflectorLocalizationNode node;
  ros::spin();
  return 0;
}

Point2 TransformLaserPointToBase(
    const Point2& point_laser, const sensor_msgs::LaserScan& scan,
    const geometry_msgs::TransformStamped& tf_laser_to_base) {
  geometry_msgs::PointStamped p_laser;
  geometry_msgs::PointStamped p_base;
  p_laser.header = scan.header;
  p_laser.point.x = point_laser.x;
  p_laser.point.y = point_laser.y;
  p_laser.point.z = 0.0;
  tf2::doTransform(p_laser, p_base, tf_laser_to_base);
  return Point2{p_base.point.x, p_base.point.y};
}

bool IsValidRange(const sensor_msgs::LaserScan& scan, float r, double max_range_m) {
  return std::isfinite(r) && r >= scan.range_min && r <= scan.range_max &&
         r <= max_range_m;
}

Point2 ApplyCenterOffset(const Point2& left, const Point2& right,
                         double center_offset_m) {
  const Point2 mid = Midpoint(left, right);
  if (std::fabs(center_offset_m) <= 1e-9) return mid;

  double ax = right.x - left.x;
  double ay = right.y - left.y;
  const double n = std::hypot(ax, ay);
  if (n < 1e-9) return mid;
  ax /= n;
  ay /= n;

  const Point2 n1{-ay, ax};
  const Point2 n2{ay, -ax};

  const Point2 c1{mid.x + center_offset_m * n1.x,
                  mid.y + center_offset_m * n1.y};
  const Point2 c2{mid.x + center_offset_m * n2.x,
                  mid.y + center_offset_m * n2.y};

  return std::hypot(c2.x, c2.y) > std::hypot(c1.x, c1.y) ? c2 : c1;
}

double ComputeQuality(const DetectorConfig& cfg, int point_count,
                      double mean_intensity, double observed_width,
                      double width_error) {
  const double width_scale = std::max(0.01, cfg.reflector_width_tol_m);
  const double width_score =
      std::exp(-(width_error * width_error) / (2.0 * width_scale * width_scale));

  const double point_target =
      std::max(cfg.min_segment_points + 2, cfg.min_segment_points);
  const double point_score =
      Clamp(static_cast<double>(point_count) / static_cast<double>(point_target),
            0.0, 1.0);

  const double intensity_range =
      std::max(1.0, cfg.intensity_high_threshold - cfg.intensity_low_threshold);
  const double intensity_score =
      Clamp((mean_intensity - cfg.intensity_low_threshold) / intensity_range,
            0.0, 1.0);

  double width_shape_bonus = 1.0;
  if (observed_width > cfg.reflector_width_m + 2.0 * cfg.reflector_width_tol_m) {
    width_shape_bonus = 0.75;
  }

  return Clamp((0.60 * width_score + 0.25 * point_score +
                0.15 * intensity_score) *
                   width_shape_bonus,
               0.0, 1.0);
}

double EstimateCenterSigma(double mean_range, double angle_increment,
                           int point_count, double width_error) {
  const double beam_step =
      std::max(0.001, 0.5 * std::fabs(angle_increment) * mean_range);
  const double n = std::max(1, point_count);
  const double quant = beam_step / std::sqrt(static_cast<double>(n));
  const double width_term = 0.35 * std::fabs(width_error);
  return std::max(0.003, quant + width_term + 0.0015);
}

struct BoundaryFitResult {
  bool valid = false;
  double angle = 0.0;
  double range = 0.0;
  Point2 laser;
  double sigma_m = 0.02;
};

bool FitWeightedLineIntensityAngle(const std::vector<const BeamSample*>& samples,
                                   double* slope_out,
                                   double* bias_out) {
  if (slope_out == nullptr || bias_out == nullptr || samples.size() < 2) {
    return false;
  }

  double sw = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  const double center_angle =
      0.5 * (samples.front()->angle + samples.back()->angle);

  for (size_t i = 0; i < samples.size(); ++i) {
    const double w = 1.0 / (1.0 + std::fabs(static_cast<double>(i) -
                                            0.5 * static_cast<double>(samples.size() - 1)));
    const double x = samples[i]->angle - center_angle;
    const double y = samples[i]->intensity;
    sw += w;
    sx += w * x;
    sy += w * y;
    sxx += w * x * x;
    sxy += w * x * y;
  }

  const double denom = sw * sxx - sx * sx;
  if (std::fabs(denom) < 1e-12) return false;

  const double slope = (sw * sxy - sx * sy) / denom;
  const double bias = (sy - slope * sx) / sw - slope * center_angle;
  if (!std::isfinite(slope) || !std::isfinite(bias)) return false;

  *slope_out = slope;
  *bias_out = bias;
  return true;
}

int FindOutsideLowSample(const std::vector<BeamSample>& all_samples,
                         int inside_pos,
                         int step,
                         double low_threshold) {
  for (int p = inside_pos + step;
       p >= 0 && p < static_cast<int>(all_samples.size()) &&
       std::abs(p - inside_pos) <= 2;
       p += step) {
    if (all_samples[p].intensity < low_threshold) return p;
  }
  return -1;
}

bool FitSubBeamBoundary(const std::vector<BeamSample>& all_samples,
                        int outside_pos,
                        int inside_pos,
                        double low_threshold,
                        double angle_increment,
                        BoundaryFitResult* out) {
  if (out == nullptr) return false;
  *out = BoundaryFitResult();
  if (outside_pos < 0 || inside_pos < 0 ||
      outside_pos >= static_cast<int>(all_samples.size()) ||
      inside_pos >= static_cast<int>(all_samples.size())) {
    return false;
  }

  const BeamSample& outside = all_samples[outside_pos];
  const BeamSample& inside = all_samples[inside_pos];
  const double intensity_delta = inside.intensity - outside.intensity;
  if (std::fabs(intensity_delta) < 1e-6) return false;

  double t = (low_threshold - outside.intensity) / intensity_delta;
  t = Clamp(t, 0.0, 1.0);

  double cross_angle = outside.angle + t * (inside.angle - outside.angle);
  const double min_angle = std::min(outside.angle, inside.angle);
  const double max_angle = std::max(outside.angle, inside.angle);

  std::vector<const BeamSample*> neighborhood;
  const int start = std::max(0, std::min(outside_pos, inside_pos) - 1);
  const int end = std::min(static_cast<int>(all_samples.size()) - 1,
                           std::max(outside_pos, inside_pos) + 2);
  neighborhood.reserve(end - start + 1);
  for (int i = start; i <= end; ++i) {
    neighborhood.push_back(&all_samples[i]);
  }

  double slope = 0.0;
  double bias = 0.0;
  if (FitWeightedLineIntensityAngle(neighborhood, &slope, &bias) &&
      std::fabs(slope) > 1e-3) {
    const double fit_angle = (low_threshold - bias) / slope;
    if (std::isfinite(fit_angle) &&
        fit_angle >= min_angle - 1.5 * std::fabs(angle_increment) &&
        fit_angle <= max_angle + 1.5 * std::fabs(angle_increment)) {
      cross_angle = Clamp(fit_angle, min_angle, max_angle);
    }
  }

  const double span = inside.angle - outside.angle;
  const double u = std::fabs(span) > 1e-9
                       ? Clamp((cross_angle - outside.angle) / span, 0.0, 1.0)
                       : t;
  const double cross_range = outside.range + u * (inside.range - outside.range);

  const double mean_range = 0.5 * (outside.range + inside.range);
  const double beam_step =
      std::max(0.001, 0.5 * std::fabs(angle_increment) * mean_range);
  const double slope_mag =
      std::fabs(intensity_delta) /
      std::max(std::fabs(inside.angle - outside.angle), 1e-6);
  const double slope_scale =
      slope_mag > 1e-3 ? Clamp(20.0 / slope_mag, 0.20, 1.0) : 1.0;

  out->valid = true;
  out->angle = cross_angle;
  out->range = cross_range;
  out->laser = Point2{cross_range * std::cos(cross_angle),
                      cross_range * std::sin(cross_angle)};
  out->sigma_m = std::max(0.0015, 0.35 * beam_step * slope_scale);
  return true;
}

}  // namespace

ReflectorDetector::ReflectorDetector(const DetectorConfig& cfg) : cfg_(cfg) {}

bool ReflectorDetector::Detect(const sensor_msgs::LaserScan& scan,
                               const geometry_msgs::TransformStamped& tf_laser_to_base,
                               std::vector<ObservedReflector>* out,
                               std::string* error,
                               DetectDebugInfo* debug_info) const {
  if (out == nullptr) {
    if (error) *error = "Detect() failed: out is null.";
    return false;
  }
  out->clear();
  if (debug_info) *debug_info = DetectDebugInfo();

  if (scan.ranges.empty()) {
    if (error) *error = "Detect() failed: scan.ranges is empty.";
    return false;
  }
  if (scan.intensities.size() != scan.ranges.size()) {
    if (error) *error = "Detect() failed: intensities size mismatch.";
    return false;
  }

  if (debug_info) debug_info->scan_point_count = scan.ranges.size();

  std::vector<BeamSample> all_samples;
  all_samples.reserve(scan.ranges.size());
  std::vector<int> sample_pos_by_scan(scan.ranges.size(), -1);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    const float intensity = scan.intensities[i];
    if (!IsValidRange(scan, r, cfg_.max_range_m) || !std::isfinite(intensity)) {
      continue;
    }
    BeamSample s;
    s.scan_index = static_cast<int>(i);
    s.angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    s.range = static_cast<double>(r);
    s.intensity = static_cast<double>(intensity);
    s.laser.x = s.range * std::cos(s.angle);
    s.laser.y = s.range * std::sin(s.angle);
    sample_pos_by_scan[i] = static_cast<int>(all_samples.size());
    all_samples.push_back(s);
  }

  std::vector<std::vector<BeamSample>> segments;
  std::vector<BeamSample> current;
  bool saw_high = false;
  int gap_count = 0;

  auto flush_segment = [&]() {
    if (!current.empty() && saw_high) {
      segments.push_back(current);
    }
    current.clear();
    saw_high = false;
    gap_count = 0;
  };

  for (const auto& s : all_samples) {
    const bool is_low = s.intensity >= cfg_.intensity_low_threshold;
    const bool is_high = s.intensity >= cfg_.intensity_high_threshold;

    if (current.empty()) {
      if (is_high) {
        current.push_back(s);
        saw_high = true;
      }
      continue;
    }

    const int expected_next = current.back().scan_index + 1;
    const int gap = std::max(0, s.scan_index - expected_next);

    if (is_low && gap <= cfg_.max_gap_points) {
      current.push_back(s);
      saw_high = saw_high || is_high;
      gap_count = 0;
      continue;
    }

    if (!is_low && gap == 0 && gap_count < cfg_.max_gap_points) {
      ++gap_count;
      continue;
    }

    flush_segment();
    if (is_high) {
      current.push_back(s);
      saw_high = true;
    }
  }
  flush_segment();

  if (debug_info) {
    debug_info->segment_count = segments.size();
    debug_info->clusters.reserve(segments.size());
  }

  out->reserve(segments.size());

  for (const auto& segment : segments) {
    DetectorClusterDebug dbg;
    dbg.point_count = static_cast<int>(segment.size());
    dbg.first_scan_index = segment.front().scan_index;
    dbg.last_scan_index = segment.back().scan_index;
    dbg.raw_points.reserve(segment.size());

    double sum_range = 0.0;
    double sum_intensity = 0.0;
    for (const auto& s : segment) {
      DetectorContourPointDebug p;
      p.scan_index = s.scan_index;
      p.angle = s.angle;
      p.range = s.range;
      p.intensity = s.intensity;
      p.laser = s.laser;
      dbg.raw_points.push_back(p);
      sum_range += s.range;
      sum_intensity += s.intensity;
    }
    dbg.mean_range = sum_range / static_cast<double>(segment.size());
    dbg.mean_intensity = sum_intensity / static_cast<double>(segment.size());

    if (debug_info) {
      debug_info->candidate_point_count += segment.size();
    }

    if (dbg.point_count < cfg_.min_segment_points ||
        dbg.point_count > cfg_.max_segment_points) {
      dbg.rejection_reason = "segment_size";
      if (debug_info) {
        ++debug_info->rejected_size_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }

    const auto& left = segment.front();
    const auto& right = segment.back();
    const int left_inside_pos =
        (left.scan_index >= 0 && left.scan_index < static_cast<int>(sample_pos_by_scan.size()))
            ? sample_pos_by_scan[left.scan_index]
            : -1;
    const int right_inside_pos =
        (right.scan_index >= 0 && right.scan_index < static_cast<int>(sample_pos_by_scan.size()))
            ? sample_pos_by_scan[right.scan_index]
            : -1;
    const int left_outside_pos =
        FindOutsideLowSample(all_samples, left_inside_pos, -1, cfg_.intensity_low_threshold);
    const int right_outside_pos =
        FindOutsideLowSample(all_samples, right_inside_pos, +1, cfg_.intensity_low_threshold);

    BoundaryFitResult left_edge;
    BoundaryFitResult right_edge;
    const bool left_fit =
        FitSubBeamBoundary(all_samples, left_outside_pos, left_inside_pos,
                           cfg_.intensity_low_threshold, scan.angle_increment, &left_edge);
    const bool right_fit =
        FitSubBeamBoundary(all_samples, right_outside_pos, right_inside_pos,
                           cfg_.intensity_low_threshold, scan.angle_increment, &right_edge);

    dbg.left_edge_laser = left_fit ? left_edge.laser : left.laser;
    dbg.right_edge_laser = right_fit ? right_edge.laser : right.laser;
    dbg.observed_width = Distance(dbg.left_edge_laser, dbg.right_edge_laser);
    dbg.width_error = dbg.observed_width - cfg_.reflector_width_m;
    dbg.sigma_center_m = EstimateCenterSigma(dbg.mean_range, scan.angle_increment,
                                             dbg.point_count, dbg.width_error);
    if (left_fit || right_fit) {
      const double left_sigma = left_fit ? left_edge.sigma_m : dbg.sigma_center_m;
      const double right_sigma = right_fit ? right_edge.sigma_m : dbg.sigma_center_m;
      const double edge_sigma =
          0.5 * std::sqrt(left_sigma * left_sigma + right_sigma * right_sigma);
      dbg.sigma_center_m = std::max(0.0025, std::min(dbg.sigma_center_m, edge_sigma + 0.002));
    }
    dbg.quality_score =
        ComputeQuality(cfg_, dbg.point_count, dbg.mean_intensity,
                       dbg.observed_width, dbg.width_error);

    if (std::fabs(dbg.width_error) > cfg_.reflector_width_tol_m) {
      dbg.rejection_reason = "width";
      if (debug_info) {
        ++debug_info->rejected_width_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }
    if (dbg.quality_score < cfg_.min_quality_score) {
      dbg.rejection_reason = "quality";
      if (debug_info) {
        ++debug_info->rejected_quality_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }

    ObservedReflector obs;
    obs.scan_start = left.scan_index;
    obs.scan_end = right.scan_index;
    obs.left_angle = left_fit ? left_edge.angle : left.angle;
    obs.right_angle = right_fit ? right_edge.angle : right.angle;
    obs.center_angle = 0.5 * (obs.left_angle + obs.right_angle);
    obs.left_laser = dbg.left_edge_laser;
    obs.right_laser = dbg.right_edge_laser;
    obs.laser = ApplyCenterOffset(obs.left_laser, obs.right_laser,
                                  cfg_.reflector_center_offset_m);
    obs.left_base = TransformLaserPointToBase(obs.left_laser, scan, tf_laser_to_base);
    obs.right_base = TransformLaserPointToBase(obs.right_laser, scan, tf_laser_to_base);
    obs.base = TransformLaserPointToBase(obs.laser, scan, tf_laser_to_base);
    obs.mean_intensity = dbg.mean_intensity;
    obs.point_count = dbg.point_count;
    obs.observed_width_m = dbg.observed_width;
    obs.width_error_m = dbg.width_error;
    obs.quality_score = dbg.quality_score;
    obs.sigma_center_m = dbg.sigma_center_m;
    obs.sigma_lateral_m = dbg.sigma_center_m;
    obs.sigma_longitudinal_m = std::max(0.004, 1.25 * dbg.sigma_center_m);
    obs.left_edge_sigma_m = left_fit ? left_edge.sigma_m : dbg.sigma_center_m;
    obs.right_edge_sigma_m = right_fit ? right_edge.sigma_m : dbg.sigma_center_m;
    obs.subbeam_fitted = left_fit || right_fit;
    obs.range = std::hypot(obs.base.x, obs.base.y);
    obs.bearing = std::atan2(obs.base.y, obs.base.x);

    dbg.selected_center_laser = obs.laser;
    dbg.selected_center_base = obs.base;

    if (cfg_.front_only && obs.base.x < cfg_.front_min_x) {
      dbg.rejection_reason = "front_gate";
      if (debug_info) {
        ++debug_info->rejected_front_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }
    if (cfg_.max_base_range_m > 0.0 && obs.range > cfg_.max_base_range_m) {
      dbg.rejection_reason = "base_range_gate";
      if (debug_info) {
        ++debug_info->rejected_base_range_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }
    if (cfg_.max_abs_y_m > 0.0 && std::fabs(obs.base.y) > cfg_.max_abs_y_m) {
      dbg.rejection_reason = "lateral_gate";
      if (debug_info) {
        ++debug_info->rejected_lateral_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }

    dbg.accepted = true;
    if (debug_info) {
      ++debug_info->accepted_count;
      debug_info->clusters.push_back(dbg);
    }
    out->push_back(obs);
  }

  std::sort(out->begin(), out->end(),
            [](const ObservedReflector& a, const ObservedReflector& b) {
              if (a.quality_score == b.quality_score) return a.range < b.range;
              return a.quality_score > b.quality_score;
            });

  if (error) {
    if (out->empty()) {
      *error = "No valid narrow-board observations after segment extraction.";
    } else {
      error->clear();
    }
  }
  return true;
}

}  // namespace reflector_localization
reflector_map.cpp:
#include "reflector_localization/reflector_map.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>

namespace reflector_localization {

bool ReflectorMap::Load(const std::string& json_path, int min_observations,
                        double max_rms_radius) {
  reflectors_.clear();
  pairs_.clear();

  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(json_path, root);
    const auto reflectors_node = root.get_child_optional("reflectors");
    if (!reflectors_node) {
      std::cerr << "Reflector json missing field: reflectors" << std::endl;
      return false;
    }

    int id = 0;
    for (const auto& item : reflectors_node.get()) {
      const auto& node = item.second;
      Reflector r;
      r.id = id++;
      r.x = node.get<double>("x", 0.0);
      r.y = node.get<double>("y", 0.0);
      r.mean_intensity = node.get<double>("mean_intensity", 0.0);
      const auto observations_opt = node.get_optional<int>("observations");
      r.observations = observations_opt ? observations_opt.get() : 0;
      r.total_points = node.get<int>("total_points", 0);
      const auto rms_radius_opt = node.get_optional<double>("rms_radius");
      r.rms_radius = rms_radius_opt ? rms_radius_opt.get() : 0.0;

      // Backward-compatible filtering:
      // - If observations/rms_radius exist, keep old quality gates.
      // - If fields are absent (map_server style), keep this reflector.
      if (observations_opt && r.observations < min_observations) continue;
      if (rms_radius_opt && r.rms_radius > max_rms_radius) continue;
      reflectors_.push_back(r);
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse reflector json: " << e.what() << std::endl;
    return false;
  }

  if (reflectors_.size() < 2) {
    std::cerr << "Not enough valid reflectors after filtering: "
              << reflectors_.size() << " (need >= 2)" << std::endl;
    return false;
  }
  BuildPairs();
  if (pairs_.empty()) {
    std::cerr << "No reflector pairs built from map." << std::endl;
    return false;
  }
  return true;
}

void ReflectorMap::BuildPairs() {
  pairs_.clear();
  for (size_t i = 0; i < reflectors_.size(); ++i) {
    for (size_t j = i + 1; j < reflectors_.size(); ++j) {
      const Reflector& a = reflectors_[i];
      const Reflector& b = reflectors_[j];
      MapPair p;
      p.idx_a = static_cast<int>(i);
      p.idx_b = static_cast<int>(j);
      p.a_map = Point2{a.x, a.y};
      p.b_map = Point2{b.x, b.y};
      p.distance = Distance(p.a_map, p.b_map);
      p.midpoint_map = Midpoint(p.a_map, p.b_map);
      p.heading_map = Heading(p.a_map, p.b_map);
      pairs_.push_back(p);
    }
  }
}

}  // namespace reflector_localization
reflector_matcher.cpp:
#include "reflector_localization/reflector_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace reflector_localization {
namespace {

double Square(double x) { return x * x; }

double HeadingResidualToLateral(double heading_rad) {
  const double r1 = std::fabs(WrapAngle(heading_rad - 0.5 * M_PI));
  const double r2 = std::fabs(WrapAngle(heading_rad + 0.5 * M_PI));
  return std::min(r1, r2);
}

struct PairSceneFitResult {
  double cost = 0.0;
  double fit_quality = 1.0;
  double midpoint_x_residual_m = 0.0;
  double midpoint_y_residual_m = 0.0;
  double separation_residual_m = 0.0;
  double heading_lateral_residual_rad = 0.0;
  double strip_width_residual_m = 0.0;
  double translation_penalty = 0.0;
  double yaw_penalty = 0.0;
  bool freeze_yaw_update = false;
};

PairSceneFitResult ComputeSceneFit(const ObservedReflector& a,
                                   const ObservedReflector& b,
                                   const SceneProfileConfig& scene) {
  PairSceneFitResult fit;
  const Point2 midpoint = Midpoint(a.base, b.base);
  const double separation = Distance(a.base, b.base);
  const double heading = Heading(a.base, b.base);

  fit.midpoint_x_residual_m =
      scene.midpoint_x_sigma_m > 1e-6
          ? std::fabs(midpoint.x - scene.midpoint_x_nominal_m)
          : 0.0;
  fit.midpoint_y_residual_m = std::fabs(midpoint.y - scene.midpoint_y_nominal_m);
  fit.separation_residual_m = std::fabs(separation - scene.separation_nominal_m);
  fit.heading_lateral_residual_rad = HeadingResidualToLateral(heading);
  fit.strip_width_residual_m =
      0.5 * (std::fabs(a.observed_width_m - scene.strip_width_nominal_m) +
             std::fabs(b.observed_width_m - scene.strip_width_nominal_m));

  double cost = 0.0;
  if (scene.midpoint_x_sigma_m > 1e-6) {
    cost += Square(fit.midpoint_x_residual_m / scene.midpoint_x_sigma_m);
  }
  if (scene.midpoint_y_sigma_m > 1e-6) {
    cost += Square(fit.midpoint_y_residual_m / scene.midpoint_y_sigma_m);
  }
  if (scene.separation_sigma_m > 1e-6) {
    cost += Square(fit.separation_residual_m / scene.separation_sigma_m);
  }
  const double baseline_sigma_rad =
      std::max(1e-6, scene.baseline_lateral_sigma_deg * M_PI / 180.0);
  cost += Square(fit.heading_lateral_residual_rad / baseline_sigma_rad);
  if (scene.strip_width_sigma_m > 1e-6) {
    cost += Square(fit.strip_width_residual_m / scene.strip_width_sigma_m);
  }

  fit.cost = cost;
  fit.fit_quality = std::exp(-0.5 * Clamp(cost, 0.0, 60.0));
  fit.translation_penalty =
      0.60 * Square(fit.midpoint_y_residual_m /
                    std::max(scene.midpoint_y_sigma_m, 1e-6)) +
      0.40 * Square(fit.separation_residual_m /
                    std::max(scene.separation_sigma_m, 1e-6)) +
      0.30 * Square(fit.strip_width_residual_m /
                    std::max(scene.strip_width_sigma_m, 1e-6));
  if (scene.midpoint_x_sigma_m > 1e-6) {
    fit.translation_penalty +=
        0.25 * Square(fit.midpoint_x_residual_m / scene.midpoint_x_sigma_m);
  }
  fit.yaw_penalty =
      0.90 * Square(fit.heading_lateral_residual_rad / baseline_sigma_rad) +
      0.55 * Square(fit.separation_residual_m /
                    std::max(scene.separation_sigma_m, 1e-6)) +
      0.35 * Square(fit.strip_width_residual_m /
                    std::max(scene.strip_width_sigma_m, 1e-6));
  fit.freeze_yaw_update =
      (fit.cost > scene.yaw_freeze_fit_cost) ||
      (baseline_sigma_rad > 0.0 &&
       fit.heading_lateral_residual_rad > 1.5 * baseline_sigma_rad);
  return fit;
}

}  // namespace

ReflectorMatcher::ReflectorMatcher(const MatcherConfig& cfg) : cfg_(cfg) {}

std::vector<ObservedPair> ReflectorMatcher::BuildObservedPairs(
    const std::vector<ObservedReflector>& observed) const {
  std::vector<ObservedPair> out;
  for (size_t i = 0; i < observed.size(); ++i) {
    for (size_t j = i + 1; j < observed.size(); ++j) {
      const auto& a = observed[i];
      const auto& b = observed[j];

      if (cfg_.front_only &&
          (a.base.x < cfg_.front_min_x || b.base.x < cfg_.front_min_x)) {
        continue;
      }

      const double d = Distance(a.base, b.base);
      if (d < cfg_.pair_distance_min || d > cfg_.pair_distance_max) continue;

      ObservedPair p;
      p.idx_a = static_cast<int>(i);
      p.idx_b = static_cast<int>(j);
      p.a_base = a.base;
      p.b_base = b.base;
      p.distance = d;
      p.midpoint_base = Midpoint(a.base, b.base);
      if (cfg_.pair_midpoint_abs_y_max > 0.0 &&
          std::fabs(p.midpoint_base.y) > cfg_.pair_midpoint_abs_y_max) {
        continue;
      }
      p.heading_base = Heading(a.base, b.base);
      p.mean_intensity = 0.5 * (a.mean_intensity + b.mean_intensity);
      const PairSceneFitResult fit =
          ComputeSceneFit(a, b, cfg_.scene_profile);
      if (fit.cost > cfg_.scene_profile.fit_reject_cost) continue;

      p.quality_score = Clamp(std::min(a.quality_score, b.quality_score) *
                                  fit.fit_quality,
                              0.0, 1.0);
      p.sigma_translation_m =
          0.5 * std::sqrt(a.sigma_center_m * a.sigma_center_m +
                          b.sigma_center_m * b.sigma_center_m);
      p.sigma_translation_m = std::max(0.003, p.sigma_translation_m);

      const double sigma_pair =
          std::sqrt(a.sigma_center_m * a.sigma_center_m +
                    b.sigma_center_m * b.sigma_center_m);
      p.sigma_yaw_rad = std::max(0.003, sigma_pair / std::max(d, 0.05));

      p.sigma_translation_m *=
          (1.0 + cfg_.scene_profile.covariance_penalty_gain * fit.translation_penalty);
      p.sigma_yaw_rad *=
          (1.0 + cfg_.scene_profile.covariance_penalty_gain * fit.yaw_penalty);
      p.scene_fit_cost = fit.cost;
      p.midpoint_x_residual_m = fit.midpoint_x_residual_m;
      p.midpoint_y_residual_m = fit.midpoint_y_residual_m;
      p.separation_residual_m = fit.separation_residual_m;
      p.heading_lateral_residual_rad = fit.heading_lateral_residual_rad;
      p.strip_width_residual_m = fit.strip_width_residual_m;
      p.freeze_yaw_update =
          fit.freeze_yaw_update ||
          (p.sigma_yaw_rad * 180.0 / M_PI >
           cfg_.scene_profile.yaw_freeze_sigma_deg);

      if (p.quality_score < cfg_.min_pair_quality) continue;
      out.push_back(p);
    }
  }

  std::sort(out.begin(), out.end(),
            [](const ObservedPair& lhs, const ObservedPair& rhs) {
              if (lhs.quality_score == rhs.quality_score) {
                return lhs.sigma_yaw_rad < rhs.sigma_yaw_rad;
              }
              return lhs.quality_score > rhs.quality_score;
            });
  return out;
}

bool ReflectorMatcher::PreferredPair(int a, int b, int pa, int pb) const {
  if (pa < 0 || pb < 0) return true;
  return (a == pa && b == pb) || (a == pb && b == pa);
}

MatchResult ReflectorMatcher::Match(const std::vector<ObservedPair>& observed_pairs,
                                    const ReflectorMap& map,
                                    const Pose2& prior_pose,
                                    int preferred_map_a,
                                    int preferred_map_b,
                                    MatchDebugInfo* debug_info) const {
  if (debug_info != nullptr) {
    *debug_info = MatchDebugInfo();
    debug_info->observed_pair_count = static_cast<int>(observed_pairs.size());
  }

  if (preferred_map_a >= 0 && preferred_map_b >= 0) {
    if (debug_info != nullptr) {
      debug_info->preferred_search_requested = true;
      debug_info->preferred_map_a = preferred_map_a;
      debug_info->preferred_map_b = preferred_map_b;
    }

    MatchResult best = MatchOnce(
        observed_pairs, map, prior_pose, preferred_map_a, preferred_map_b,
        true, true, debug_info != nullptr ? &debug_info->preferred_search : nullptr);
    if (best.success) return best;

    if (debug_info != nullptr) {
      debug_info->fallback_to_global = true;
    }
    return MatchOnce(observed_pairs, map, prior_pose, -1, -1, false, false,
                     debug_info != nullptr ? &debug_info->global_search : nullptr);
  }

  return MatchOnce(observed_pairs, map, prior_pose, -1, -1, false, false,
                   debug_info != nullptr ? &debug_info->global_search : nullptr);
}

MatchResult ReflectorMatcher::MatchOnce(const std::vector<ObservedPair>& observed_pairs,
                                        const ReflectorMap& map,
                                        const Pose2& prior_pose,
                                        int preferred_map_a,
                                        int preferred_map_b,
                                        bool restrict_to_preferred,
                                        bool allow_preferred_bonus,
                                        MatchSearchStats* stats) const {
  MatchResult best;
  for (const auto& obs_pair : observed_pairs) {
    for (const auto& map_pair : map.pairs()) {
      if (std::fabs(map_pair.distance - obs_pair.distance) > cfg_.pair_distance_tol) {
        continue;
      }
      const bool preferred =
          PreferredPair(map_pair.idx_a, map_pair.idx_b, preferred_map_a,
                        preferred_map_b);
      if (restrict_to_preferred && !preferred) {
        continue;
      }
      if (stats != nullptr) {
        ++stats->distance_compatible_pairs;
      }

      MatchResult candidate =
          EvaluateCandidate(obs_pair, map_pair, prior_pose,
                            allow_preferred_bonus && preferred, stats);
      if (!candidate.success) continue;
      if (candidate.score < best.score) best = candidate;
    }
  }
  return best;
}

MatchResult ReflectorMatcher::EvaluateCandidate(const ObservedPair& obs_pair,
                                                const MapPair& map_pair,
                                                const Pose2& prior_pose,
                                                bool preferred_bonus,
                                                MatchSearchStats* stats) const {
  MatchResult best;
  best.observed_pair = obs_pair;

  const Point2 prior_a = TransformPoint(prior_pose, obs_pair.a_base);
  const Point2 prior_b = TransformPoint(prior_pose, obs_pair.b_base);
  const Point2 prior_mid = Midpoint(prior_a, prior_b);
  const double prior_heading = Heading(prior_a, prior_b);

  struct OrderedPair {
    Point2 a;
    Point2 b;
    int idx_a = -1;
    int idx_b = -1;
  } orderings[2] = {
      {map_pair.a_map, map_pair.b_map, map_pair.idx_a, map_pair.idx_b},
      {map_pair.b_map, map_pair.a_map, map_pair.idx_b, map_pair.idx_a},
  };

  for (const auto& ordered : orderings) {
    if (stats != nullptr) {
      ++stats->ordered_candidates_tested;
    }
    const double point_cost =
        Distance(prior_a, ordered.a) + Distance(prior_b, ordered.b);
    if (point_cost > cfg_.prior_point_gate) {
      if (stats != nullptr) {
        ++stats->rejected_point_gate;
      }
      continue;
    }

    const Point2 map_mid = Midpoint(ordered.a, ordered.b);
    const double midpoint_cost = Distance(prior_mid, map_mid);
    if (midpoint_cost > cfg_.midpoint_gate) {
      if (stats != nullptr) {
        ++stats->rejected_midpoint_gate;
      }
      continue;
    }

    const double heading_cost =
        std::fabs(WrapAngle(prior_heading - Heading(ordered.a, ordered.b)));
    if (heading_cost > cfg_.heading_gate_rad) {
      if (stats != nullptr) {
        ++stats->rejected_heading_gate;
      }
      continue;
    }

    const Pose2 pose =
        SolvePoseFromTwoPoints(obs_pair.a_base, obs_pair.b_base,
                               ordered.a, ordered.b);
    const double trans_delta =
        std::hypot(pose.x - prior_pose.x, pose.y - prior_pose.y);
    const double yaw_delta = std::fabs(WrapAngle(pose.yaw - prior_pose.yaw));
    if (trans_delta > cfg_.max_prior_translation_delta ||
        yaw_delta > cfg_.max_prior_yaw_delta) {
      if (stats != nullptr) {
        ++stats->rejected_prior_delta_gate;
      }
      continue;
    }
    if (stats != nullptr) {
      ++stats->successful_candidates;
    }

    const double point_norm = point_cost / std::max(cfg_.prior_point_gate, 1e-6);
    const double mid_norm = midpoint_cost / std::max(cfg_.midpoint_gate, 1e-6);
    const double head_norm =
        heading_cost / std::max(cfg_.heading_gate_rad, 1e-6);
    const double trans_norm =
        trans_delta / std::max(cfg_.max_prior_translation_delta, 1e-6);
    const double yaw_norm =
        yaw_delta / std::max(cfg_.max_prior_yaw_delta, 1e-6);

    double score = 1.0 * point_norm + 0.70 * mid_norm + 0.65 * head_norm +
                   0.30 * trans_norm + 0.30 * yaw_norm -
                   0.35 * obs_pair.quality_score;
    if (preferred_bonus) score -= 0.10;

    if (score < best.score) {
      best.success = true;
      best.obs_idx_a = obs_pair.idx_a;
      best.obs_idx_b = obs_pair.idx_b;
      best.map_idx_a = ordered.idx_a;
      best.map_idx_b = ordered.idx_b;
      best.pose_map_base = pose;
      best.score = score;
      best.prior_point_cost = point_cost;
      best.midpoint_cost = midpoint_cost;
      best.heading_cost = heading_cost;
      best.prior_translation_delta = trans_delta;
      best.prior_yaw_delta = yaw_delta;
      best.pair_quality_score = obs_pair.quality_score;
      best.measurement_translation_sigma_m = obs_pair.sigma_translation_m;
      best.measurement_yaw_sigma_rad = obs_pair.sigma_yaw_rad;
      best.preferred_pair_used = preferred_bonus;
    }
  }

  return best;
}

}  // namespace reflector_localization
scan_window_builder.cpp:
#include "reflector_localization/scan_window_builder.hpp"

#include <cmath>

#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace reflector_localization {
namespace {

Point2 TransformLaserPointToBase(
    const Point2& point_laser, const sensor_msgs::LaserScan& scan,
    const geometry_msgs::TransformStamped& tf_laser_to_base) {
  geometry_msgs::PointStamped p_laser;
  geometry_msgs::PointStamped p_base;
  p_laser.header = scan.header;
  p_laser.point.x = point_laser.x;
  p_laser.point.y = point_laser.y;
  p_laser.point.z = 0.0;
  tf2::doTransform(p_laser, p_base, tf_laser_to_base);
  return Point2{p_base.point.x, p_base.point.y};
}

bool IsValidRange(const sensor_msgs::LaserScan& scan, float range,
                  double max_range_m) {
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max &&
         range <= max_range_m;
}

}  // namespace

ScanWindowBuilder::ScanWindowBuilder(const ScanWindowBuilderConfig& cfg) : cfg_(cfg) {}

bool ScanWindowBuilder::Build(const sensor_msgs::LaserScan& scan,
                              const geometry_msgs::TransformStamped& tf_laser_to_base,
                              BeamCloud* out, std::string* error,
                              ScanWindowDebugInfo* debug_info) const {
  if (out == nullptr) {
    if (error) *error = "Build() failed: out is null";
    return false;
  }

  *out = BeamCloud();
  if (debug_info) *debug_info = ScanWindowDebugInfo();

  if (scan.ranges.empty()) {
    if (error) *error = "scan is empty";
    return false;
  }
  if (!scan.intensities.empty() && scan.intensities.size() != scan.ranges.size()) {
    if (error) *error = "scan intensity size mismatch";
    return false;
  }

  out->valid = true;
  out->angle_increment = scan.angle_increment;
  out->range_min = scan.range_min;
  out->range_max = scan.range_max;
  out->roi_max_range_m = cfg_.max_range_m;
  out->roi_max_abs_y_m = cfg_.max_abs_y_m;
  out->roi_front_min_x_m = cfg_.front_min_x;

  if (debug_info) {
    debug_info->scan_point_count = scan.ranges.size();
  }

  out->beams.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!IsValidRange(scan, range, cfg_.max_range_m)) continue;

    if (debug_info) {
      ++debug_info->valid_range_count;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const Point2 laser{static_cast<double>(range) * std::cos(angle),
                       static_cast<double>(range) * std::sin(angle)};
    const Point2 base = TransformLaserPointToBase(laser, scan, tf_laser_to_base);
    const double base_range = Norm(base);

    if (cfg_.front_only && base.x < cfg_.front_min_x) continue;
    if (cfg_.max_base_range_m > 0.0 && base_range > cfg_.max_base_range_m) continue;
    if (cfg_.max_abs_y_m > 0.0 && std::fabs(base.y) > cfg_.max_abs_y_m) continue;

    BeamSample sample;
    sample.scan_idx = static_cast<int>(i);
    sample.angle = angle;
    sample.range = static_cast<double>(range);
    sample.intensity =
        scan.intensities.empty() ? 0.0 : static_cast<double>(scan.intensities[i]);
    sample.laser = laser;
    sample.base = base;
    out->beams.push_back(sample);
  }

  if (debug_info) {
    debug_info->roi_beam_count = out->beams.size();
  }

  if (out->beams.empty()) {
    if (error) *error = "no beams remained in ROI";
    return false;
  }

  if (error) error->clear();
  return true;
}

}  // namespace reflector_localization
scene_prior.cpp:
#include "reflector_localization/scene_prior.hpp"

namespace reflector_localization {

ScenePrior ScenePriorProvider::Defaults(const std::string& name) {
  ScenePrior prior;
  prior.name = name;
  prior.mid_y_nominal_m = 0.0;
  prior.mid_y_sigma_m = 0.12;
  prior.width_nominal_m = 0.06;
  prior.width_sigma_m = 0.020;
  prior.yaw_prior_rad = 0.5 * M_PI;
  prior.yaw_prior_sigma_rad = 10.0 * M_PI / 180.0;
  prior.yaw_axis_prior = true;
  prior.bias_sigma_m = 0.030;
  prior.bias_diff_sigma_m = 0.020;
  prior.reject_fit_cost = 32.0;
  prior.min_quality = 0.10;
  prior.freeze_yaw_sigma_rad = 3.5 * M_PI / 180.0;

  if (name == "charger_front") {
    prior.mid_x_nominal_m = 1.8;
    prior.mid_x_sigma_m = 1.0;
  } else if (name == "shelf_front") {
    prior.mid_x_nominal_m = 2.2;
    prior.mid_x_sigma_m = 1.0;
  } else {
    prior.name = "auto";
    prior.mid_x_nominal_m = 0.0;
    prior.mid_x_sigma_m = 0.0;
  }

  return prior;
}

}  // namespace reflector_localization
strip_edge_extractor.cpp:
#include "reflector_localization/strip_edge_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace reflector_localization {
namespace {

struct BoundaryFitResult {
  bool valid = false;
  BoundaryFit fit;
};

double ComputeSegmentQuality(const StripEdgeExtractorConfig& cfg,
                             int point_count,
                             double mean_intensity,
                             double observed_width,
                             double asymmetry_score) {
  const double point_target = std::max(cfg.min_segment_points + 1, cfg.min_segment_points);
  const double point_score =
      Clamp(static_cast<double>(point_count) / static_cast<double>(point_target), 0.0, 1.0);
  const double intensity_span =
      std::max(1.0, cfg.intensity_high_threshold - cfg.intensity_low_threshold);
  const double intensity_score =
      Clamp((mean_intensity - cfg.intensity_low_threshold) / intensity_span, 0.0, 1.0);
  const double width_score =
      std::exp(-0.5 * std::pow((observed_width - 0.06) / 0.03, 2.0));
  const double asymmetry_penalty =
      Clamp(1.0 - 0.5 * asymmetry_score / std::max(observed_width, 1e-3), 0.3, 1.0);
  return Clamp((0.45 * point_score + 0.35 * intensity_score + 0.20 * width_score) *
                   asymmetry_penalty,
               0.0, 1.0);
}

bool FitWeightedLineIntensityAngle(const std::vector<const BeamSample*>& samples,
                                   double* slope_out,
                                   double* bias_out) {
  if (slope_out == nullptr || bias_out == nullptr || samples.size() < 2) {
    return false;
  }

  double sw = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  const double center_angle =
      0.5 * (samples.front()->angle + samples.back()->angle);

  for (size_t i = 0; i < samples.size(); ++i) {
    const double w = 1.0 / (1.0 + std::fabs(static_cast<double>(i) -
                                            0.5 * static_cast<double>(samples.size() - 1)));
    const double x = samples[i]->angle - center_angle;
    const double y = samples[i]->intensity;
    sw += w;
    sx += w * x;
    sy += w * y;
    sxx += w * x * x;
    sxy += w * x * y;
  }

  const double denom = sw * sxx - sx * sx;
  if (std::fabs(denom) < 1e-12) return false;

  const double slope = (sw * sxy - sx * sy) / denom;
  const double bias = (sy - slope * sx) / sw - slope * center_angle;
  if (!std::isfinite(slope) || !std::isfinite(bias)) return false;

  *slope_out = slope;
  *bias_out = bias;
  return true;
}

int FindOutsideLowSample(const std::vector<BeamSample>& all_samples,
                         int inside_pos,
                         int step,
                         double low_threshold) {
  for (int p = inside_pos + step;
       p >= 0 && p < static_cast<int>(all_samples.size()) &&
       std::abs(p - inside_pos) <= 2;
       p += step) {
    if (all_samples[p].intensity < low_threshold) return p;
  }
  return -1;
}

BoundaryFitResult FitSubBeamBoundary(const std::vector<BeamSample>& all_samples,
                                     int outside_pos,
                                     int inside_pos,
                                     double low_threshold,
                                     double angle_increment) {
  BoundaryFitResult result;
  if (outside_pos < 0 || inside_pos < 0 ||
      outside_pos >= static_cast<int>(all_samples.size()) ||
      inside_pos >= static_cast<int>(all_samples.size())) {
    return result;
  }

  const BeamSample& outside = all_samples[outside_pos];
  const BeamSample& inside = all_samples[inside_pos];
  const double intensity_delta = inside.intensity - outside.intensity;
  if (std::fabs(intensity_delta) < 1e-6) return result;

  double t = (low_threshold - outside.intensity) / intensity_delta;
  t = Clamp(t, 0.0, 1.0);

  double cross_angle = outside.angle + t * (inside.angle - outside.angle);
  const double min_angle = std::min(outside.angle, inside.angle);
  const double max_angle = std::max(outside.angle, inside.angle);

  std::vector<const BeamSample*> neighborhood;
  const int start = std::max(0, std::min(outside_pos, inside_pos) - 1);
  const int end = std::min(static_cast<int>(all_samples.size()) - 1,
                           std::max(outside_pos, inside_pos) + 2);
  neighborhood.reserve(end - start + 1);
  for (int i = start; i <= end; ++i) {
    neighborhood.push_back(&all_samples[i]);
  }

  double slope = 0.0;
  double bias = 0.0;
  if (FitWeightedLineIntensityAngle(neighborhood, &slope, &bias) &&
      std::fabs(slope) > 1e-3) {
    const double fit_angle = (low_threshold - bias) / slope;
    if (std::isfinite(fit_angle) &&
        fit_angle >= min_angle - 1.5 * std::fabs(angle_increment) &&
        fit_angle <= max_angle + 1.5 * std::fabs(angle_increment)) {
      cross_angle = Clamp(fit_angle, min_angle, max_angle);
    }
  }

  const double span = inside.angle - outside.angle;
  const double u = std::fabs(span) > 1e-9
                       ? Clamp((cross_angle - outside.angle) / span, 0.0, 1.0)
                       : t;
  const double cross_range = outside.range + u * (inside.range - outside.range);
  const Point2 cross_laser{cross_range * std::cos(cross_angle),
                           cross_range * std::sin(cross_angle)};
  const Point2 cross_base = outside.base + u * (inside.base - outside.base);

  const double mean_range = 0.5 * (outside.range + inside.range);
  const double beam_step =
      std::max(0.001, 0.5 * std::fabs(angle_increment) * mean_range);
  const double slope_mag =
      std::fabs(intensity_delta) /
      std::max(std::fabs(inside.angle - outside.angle), 1e-6);
  const double slope_scale =
      slope_mag > 1e-3 ? Clamp(20.0 / slope_mag, 0.20, 1.0) : 1.0;

  result.valid = true;
  result.fit.valid = true;
  result.fit.idx_lo = std::min(outside.scan_idx, inside.scan_idx);
  result.fit.idx_hi = std::max(outside.scan_idx, inside.scan_idx);
  result.fit.angle_cross = cross_angle;
  result.fit.range_cross = cross_range;
  result.fit.laser = cross_laser;
  result.fit.base = cross_base;
  result.fit.sigma_t = std::max(0.0015, 0.30 * beam_step * slope_scale);
  result.fit.sigma_n = std::max(0.0030, 0.70 * beam_step * slope_scale + 0.0030);
  result.fit.residual = std::fabs(low_threshold - (outside.intensity + u * intensity_delta));
  return result;
}

double EstimateCurvature(const std::vector<BeamSample>& segment) {
  if (segment.size() < 3) return 0.0;
  double acc = 0.0;
  for (size_t i = 1; i + 1 < segment.size(); ++i) {
    const Point2 a = Normalize(segment[i].base - segment[i - 1].base);
    const Point2 b = Normalize(segment[i + 1].base - segment[i].base);
    acc += std::fabs(std::atan2(a.x * b.y - a.y * b.x, Dot(a, b)));
  }
  return acc / static_cast<double>(segment.size() - 2);
}

}  // namespace

StripEdgeExtractor::StripEdgeExtractor(const StripEdgeExtractorConfig& cfg) : cfg_(cfg) {}

bool StripEdgeExtractor::Extract(const BeamCloud& cloud,
                                 std::vector<StripEdgeObservation>* out,
                                 std::string* error,
                                 StripEdgeDebugInfo* debug_info) const {
  if (out == nullptr) {
    if (error) *error = "Extract() failed: out is null";
    return false;
  }

  out->clear();
  if (debug_info) *debug_info = StripEdgeDebugInfo();

  if (cloud.beams.empty()) {
    if (error) *error = "beam cloud is empty";
    return false;
  }

  const std::vector<BeamSample>& all_samples = cloud.beams;
  std::vector<int> sample_pos_by_scan;
  int max_scan_idx = 0;
  for (const auto& beam : all_samples) {
    max_scan_idx = std::max(max_scan_idx, beam.scan_idx);
  }
  sample_pos_by_scan.assign(static_cast<size_t>(max_scan_idx + 1), -1);
  for (size_t i = 0; i < all_samples.size(); ++i) {
    sample_pos_by_scan[static_cast<size_t>(all_samples[i].scan_idx)] = static_cast<int>(i);
  }

  std::vector<std::vector<BeamSample>> segments;
  std::vector<BeamSample> current;
  bool saw_high = false;
  int gap_count = 0;

  const auto flush_segment = [&]() {
    if (!current.empty() && saw_high) {
      segments.push_back(current);
    }
    current.clear();
    saw_high = false;
    gap_count = 0;
  };

  for (const auto& beam : all_samples) {
    const bool is_low = beam.intensity >= cfg_.intensity_low_threshold;
    const bool is_high = beam.intensity >= cfg_.intensity_high_threshold;

    if (current.empty()) {
      if (is_high) {
        current.push_back(beam);
        saw_high = true;
      }
      continue;
    }

    const int expected_next = current.back().scan_idx + 1;
    const int gap = std::max(0, beam.scan_idx - expected_next);

    if (is_low && gap <= cfg_.max_gap_points) {
      current.push_back(beam);
      saw_high = saw_high || is_high;
      gap_count = 0;
      continue;
    }

    if (!is_low && gap == 0 && gap_count < cfg_.max_gap_points) {
      ++gap_count;
      continue;
    }

    flush_segment();
    if (is_high) {
      current.push_back(beam);
      saw_high = true;
    }
  }
  flush_segment();

  if (debug_info) {
    debug_info->segment_count = segments.size();
    debug_info->clusters.reserve(segments.size());
  }

  out->reserve(segments.size());

  for (const auto& segment : segments) {
    StripClusterDebug dbg;
    dbg.segment_start = segment.front().scan_idx;
    dbg.segment_end = segment.back().scan_idx;
    dbg.point_count = static_cast<int>(segment.size());

    if (dbg.point_count < cfg_.min_segment_points ||
        dbg.point_count > cfg_.max_segment_points) {
      dbg.rejection_reason = "segment_size";
      if (debug_info) {
        ++debug_info->rejected_size_count;
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }

    double sum_intensity = 0.0;
    for (const auto& beam : segment) {
      sum_intensity += beam.intensity;
    }

    const BeamSample& left_inside = segment.front();
    const BeamSample& right_inside = segment.back();
    const int left_inside_pos =
        sample_pos_by_scan[static_cast<size_t>(left_inside.scan_idx)];
    const int right_inside_pos =
        sample_pos_by_scan[static_cast<size_t>(right_inside.scan_idx)];
    const int left_outside_pos =
        FindOutsideLowSample(all_samples, left_inside_pos, -1, cfg_.intensity_low_threshold);
    const int right_outside_pos =
        FindOutsideLowSample(all_samples, right_inside_pos, +1, cfg_.intensity_low_threshold);

    const BoundaryFitResult left_fit =
        FitSubBeamBoundary(all_samples, left_outside_pos, left_inside_pos,
                           cfg_.intensity_low_threshold, cloud.angle_increment);
    const BoundaryFitResult right_fit =
        FitSubBeamBoundary(all_samples, right_outside_pos, right_inside_pos,
                           cfg_.intensity_low_threshold, cloud.angle_increment);
    if (!left_fit.valid || !right_fit.valid) {
      dbg.rejection_reason = "boundary_fit";
      if (debug_info) {
        debug_info->clusters.push_back(dbg);
      }
      continue;
    }

    StripEdgeObservation obs;
    obs.valid = true;
    obs.segment_start = dbg.segment_start;
    obs.segment_end = dbg.segment_end;
    obs.left = left_fit.fit;
    obs.right = right_fit.fit;
    obs.point_count = dbg.point_count;
    obs.mean_intensity = sum_intensity / static_cast<double>(dbg.point_count);
    obs.center_base = Midpoint(obs.left.base, obs.right.base);
    obs.center_laser = Midpoint(obs.left.laser, obs.right.laser);
    obs.observed_width = Distance(obs.left.base, obs.right.base);
    const double left_len = Distance(obs.center_base, obs.left.base);
    const double right_len = Distance(obs.center_base, obs.right.base);
    obs.asymmetry_score = std::fabs(left_len - right_len);
    obs.profile_curvature = EstimateCurvature(segment);
    obs.segment_quality = ComputeSegmentQuality(cfg_, dbg.point_count, obs.mean_intensity,
                                               obs.observed_width, obs.asymmetry_score);
    obs.sigma_s = std::max(0.003,
                           0.5 * (obs.left.sigma_t + obs.right.sigma_t) +
                               0.25 * obs.asymmetry_score);
    obs.sigma_n = std::max(0.006,
                           0.5 * (obs.left.sigma_n + obs.right.sigma_n) +
                               0.30 * obs.profile_curvature);

    dbg.accepted = true;
    dbg.center_base = obs.center_base;
    dbg.width = obs.observed_width;
    dbg.quality = obs.segment_quality;
    if (debug_info) {
      ++debug_info->accepted_count;
      debug_info->clusters.push_back(dbg);
    }
    out->push_back(obs);
  }

  std::sort(out->begin(), out->end(),
            [](const StripEdgeObservation& a, const StripEdgeObservation& b) {
              if (a.segment_quality == b.segment_quality) {
                return a.center_base.x < b.center_base.x;
              }
              return a.segment_quality > b.segment_quality;
            });

  if (out->empty()) {
    if (error) *error = "no strip edge observations extracted";
    return false;
  }

  if (error) error->clear();
  return true;
}

}  // namespace reflector_localization
strip_interval_projector.cpp:
#include "reflector_localization/strip_interval_projector.hpp"

#include <cmath>

namespace reflector_localization {

StripIntervalProjector::StripIntervalProjector(const StripIntervalProjectorConfig& cfg)
    : cfg_(cfg) {}

bool StripIntervalProjector::Project(const StripProjectorInput& in,
                                     StripIntervalObservation* out,
                                     StripProjectorDebug* dbg,
                                     std::string* error) const {
  if (out == nullptr) {
    if (error) *error = "Project() failed: out is null";
    return false;
  }

  *out = StripIntervalObservation();
  if (dbg != nullptr) *dbg = StripProjectorDebug();

  if (in.strip == nullptr || in.support_line == nullptr) {
    if (error) *error = "strip or support_line is null";
    return false;
  }
  if (!in.strip->valid || !in.support_line->valid) {
    if (error) *error = "strip or support_line invalid";
    return false;
  }

  const Point2 dl = in.strip->left.base - in.support_line->anchor_base;
  const Point2 dr = in.strip->right.base - in.support_line->anchor_base;

  double s_lo = Dot(in.support_line->t, dl);
  double s_hi = Dot(in.support_line->t, dr);
  double d_lo = Dot(in.support_line->n, dl);
  double d_hi = Dot(in.support_line->n, dr);

  if (s_lo > s_hi) {
    std::swap(s_lo, s_hi);
    std::swap(d_lo, d_hi);
  }

  out->valid = true;
  out->strip_id = 0;
  out->segment_start = in.strip->segment_start;
  out->segment_end = in.strip->segment_end;
  out->s_lo = s_lo;
  out->s_hi = s_hi;
  out->s_center = 0.5 * (s_lo + s_hi);
  out->width_s = s_hi - s_lo;
  out->d_lo = d_lo;
  out->d_hi = d_hi;
  out->d_center = 0.5 * (d_lo + d_hi);
  out->skew_n = d_hi - d_lo;
  out->asymmetry_s = in.strip->asymmetry_score;
  out->curvature_hint = in.strip->profile_curvature;
  out->left_base = in.strip->left.base;
  out->right_base = in.strip->right.base;

  const double sigma_s =
      std::max(0.004, in.strip->sigma_s + in.support_line->sigma_yaw_rad * out->width_s);
  out->sigma_s_lo = sigma_s;
  out->sigma_s_hi = sigma_s;
  out->sigma_s_center = std::max(0.004, sigma_s / std::sqrt(2.0));
  out->sigma_width_s = std::max(0.006, std::sqrt(2.0) * sigma_s);
  out->sigma_d_center =
      std::max(cfg_.d_center_sigma_m,
               std::sqrt(in.strip->sigma_n * in.strip->sigma_n +
                         in.support_line->sigma_offset_m * in.support_line->sigma_offset_m));
  out->sigma_skew_n =
      std::max(cfg_.skew_sigma_m, std::sqrt(2.0) * out->sigma_d_center);

  const double width_err = out->width_s - cfg_.width_nominal_m;
  const double width_score =
      std::exp(-0.5 * std::pow(width_err / std::max(cfg_.width_sigma_m, 1e-3), 2.0));
  const double d_score =
      std::exp(-0.5 * std::pow(out->d_center / std::max(cfg_.max_abs_d_center_m, 1e-3), 2.0));
  const double skew_score =
      std::exp(-0.5 * std::pow(out->skew_n / std::max(cfg_.max_abs_skew_n_m, 1e-3), 2.0));
  const double curvature_score =
      std::exp(-0.5 * std::pow(out->curvature_hint / 0.25, 2.0));

  out->quality = Clamp(in.strip->segment_quality * width_score * d_score *
                           skew_score * curvature_score,
                       0.0, 1.0);
  out->usable_for_position = std::fabs(out->d_center) <= 1.5 * cfg_.max_abs_d_center_m;
  out->usable_for_yaw_correction =
      in.support_line->quality >= cfg_.yaw_correction_quality_thresh &&
      std::fabs(out->skew_n) <= cfg_.yaw_correction_max_skew_n_m &&
      out->quality >= cfg_.yaw_correction_quality_thresh;

  if (dbg != nullptr) {
    dbg->raw_chord_length = Distance(in.strip->left.base, in.strip->right.base);
    dbg->projected_width_s = out->width_s;
    dbg->d_left = d_lo;
    dbg->d_right = d_hi;
  }

  if (error) error->clear();
  return true;
}

}  // namespace reflector_localization
support_line_fitter.cpp:
#include "reflector_localization/support_line_fitter.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace reflector_localization {
namespace {

double HuberWeight(double residual, double delta) {
  const double a = std::fabs(residual);
  if (a <= delta) return 1.0;
  return delta / std::max(a, 1e-9);
}

double SelectReferenceYaw(const SupportLineFitterInput& in) {
  if (in.prior_support_line != nullptr && in.prior_support_line->valid) {
    return in.prior_support_line->yaw;
  }
  if (in.raw_strips != nullptr && in.raw_strips->size() >= 2) {
    size_t best_i = 0;
    size_t best_j = 1;
    double best_dist = 0.0;
    for (size_t i = 0; i < in.raw_strips->size(); ++i) {
      for (size_t j = i + 1; j < in.raw_strips->size(); ++j) {
        const double d =
            SquaredDistance(in.raw_strips->at(i).center_base,
                            in.raw_strips->at(j).center_base);
        if (d > best_dist) {
          best_dist = d;
          best_i = i;
          best_j = j;
        }
      }
    }
    return Heading(in.raw_strips->at(best_i).center_base,
                   in.raw_strips->at(best_j).center_base);
  }
  if (in.scene_prior != nullptr) {
    return in.scene_prior->yaw_prior_rad;
  }
  return 0.0;
}

Point2 AverageStripCenter(const std::vector<StripEdgeObservation>& strips) {
  Point2 acc{0.0, 0.0};
  if (strips.empty()) return acc;
  for (const auto& strip : strips) {
    acc = acc + strip.center_base;
  }
  return acc / static_cast<double>(strips.size());
}

bool IsInsideStripNeighborhood(int scan_idx,
                               const std::vector<StripEdgeObservation>& strips) {
  for (const auto& strip : strips) {
    if (scan_idx >= strip.segment_start - 1 && scan_idx <= strip.segment_end + 1) {
      return true;
    }
  }
  return false;
}

void WeightedLineFit(const std::vector<Point2>& points,
                     const std::vector<double>& weights,
                     const Point2& reference_t,
                     Point2* anchor_out,
                     double* yaw_out,
                     double* eig_major_out,
                     double* eig_minor_out,
                     double* normal_rms_out) {
  double sum_w = 0.0;
  Point2 mean{0.0, 0.0};
  for (size_t i = 0; i < points.size(); ++i) {
    sum_w += weights[i];
    mean = mean + weights[i] * points[i];
  }
  if (sum_w <= 1e-9) sum_w = 1.0;
  mean = mean / sum_w;

  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < points.size(); ++i) {
    const Point2 d = points[i] - mean;
    const Eigen::Vector2d v(d.x, d.y);
    cov += weights[i] * (v * v.transpose());
  }
  cov /= sum_w;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  const Eigen::Vector2d tangent_eig = solver.eigenvectors().col(1);
  Point2 tangent{tangent_eig.x(), tangent_eig.y()};
  tangent = Normalize(tangent);
  if (Dot(tangent, reference_t) < 0.0) tangent = -1.0 * tangent;

  const Point2 normal{-tangent.y, tangent.x};
  double rms = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const double r = Dot(normal, points[i] - mean);
    rms += weights[i] * r * r;
  }
  rms = std::sqrt(rms / sum_w);

  if (anchor_out) *anchor_out = mean;
  if (yaw_out) *yaw_out = std::atan2(tangent.y, tangent.x);
  if (eig_major_out) *eig_major_out = solver.eigenvalues()(1);
  if (eig_minor_out) *eig_minor_out = solver.eigenvalues()(0);
  if (normal_rms_out) *normal_rms_out = rms;
}

double TangentSpan(const std::vector<Point2>& points,
                   const Point2& anchor,
                   const Point2& t) {
  if (points.empty()) return 0.0;
  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  for (const auto& p : points) {
    const double s = Dot(t, p - anchor);
    s_min = std::min(s_min, s);
    s_max = std::max(s_max, s);
  }
  return s_max - s_min;
}

double AxisBlendYaw(double current_yaw,
                    double prior_yaw,
                    double blend_ratio) {
  Point2 t_cur = TangentFromYaw(current_yaw);
  Point2 t_prior = TangentFromYaw(prior_yaw);
  if (Dot(t_cur, t_prior) < 0.0) t_prior = -1.0 * t_prior;
  const Point2 blended = Normalize((1.0 - blend_ratio) * t_cur + blend_ratio * t_prior);
  return std::atan2(blended.y, blended.x);
}

SupportLineEstimate MakePriorEstimate(const SupportLineFitterInput& in,
                                      double reference_yaw) {
  SupportLineEstimate out;
  if (in.prior_support_line != nullptr && in.prior_support_line->valid) {
    out = *in.prior_support_line;
    out.source = SupportLineEstimate::Source::kPriorPropagated;
    out.yaw_frozen = true;
    UpdateSupportLineFrame(&out);
    return out;
  }

  if (in.raw_strips == nullptr || in.raw_strips->empty()) return out;
  out.valid = true;
  out.anchor_base = AverageStripCenter(*in.raw_strips);
  out.yaw = reference_yaw;
  UpdateSupportLineFrame(&out);
  out.tangent_span_m = 0.0;
  out.inlier_count = 0;
  out.normal_rms = 0.05;
  out.condition_ratio = 0.0;
  out.quality = 0.15;
  out.sigma_yaw_rad =
      in.scene_prior != nullptr
          ? std::max(in.scene_prior->yaw_prior_sigma_rad, 8.0 * M_PI / 180.0)
          : 8.0 * M_PI / 180.0;
  out.sigma_offset_m = 0.05;
  out.source = SupportLineEstimate::Source::kPriorPropagated;
  out.yaw_frozen = true;
  out.fit_cost = 1e3;
  return out;
}

}  // namespace

SupportLineFitter::SupportLineFitter(const SupportLineFitterConfig& cfg) : cfg_(cfg) {}

bool SupportLineFitter::Fit(const SupportLineFitterInput& in,
                            SupportLineEstimate* out,
                            SupportLineFitterDebug* dbg,
                            std::string* error) const {
  if (out == nullptr) {
    if (error) *error = "Fit() failed: out is null";
    return false;
  }

  *out = SupportLineEstimate();
  if (dbg != nullptr) *dbg = SupportLineFitterDebug();

  if (in.cloud == nullptr || in.raw_strips == nullptr) {
    if (error) *error = "cloud or raw_strips is null";
    return false;
  }
  if (in.raw_strips->empty()) {
    if (error) *error = "no strips available for support line fit";
    return false;
  }

  const Point2 raw_mid = AverageStripCenter(*in.raw_strips);
  const double reference_yaw = SelectReferenceYaw(in);
  const Point2 t_ref = TangentFromYaw(reference_yaw);
  const Point2 n_ref = NormalFromYaw(reference_yaw);

  std::vector<Point2> candidates;
  candidates.reserve(in.cloud->beams.size());
  for (const auto& beam : in.cloud->beams) {
    if (IsInsideStripNeighborhood(beam.scan_idx, *in.raw_strips)) continue;

    const Point2 rel = beam.base - raw_mid;
    if (std::fabs(Dot(t_ref, rel)) > cfg_.support_line_window_m) continue;
    if (std::fabs(Dot(n_ref, rel)) > cfg_.support_line_neighbor_margin_m) continue;
    candidates.push_back(beam.base);
  }

  if (dbg != nullptr) {
    dbg->candidate_points_base = candidates;
  }

  if (static_cast<int>(candidates.size()) < cfg_.support_line_min_points) {
    *out = MakePriorEstimate(in, reference_yaw);
    if (!out->valid) {
      if (error) *error = "support line fallback failed";
      return false;
    }
    if (dbg != nullptr) {
      dbg->raw_yaw = reference_yaw;
      dbg->blended_yaw = out->yaw;
    }
    if (error) error->clear();
    return true;
  }

  std::vector<double> weights(candidates.size(), 1.0);
  Point2 anchor = raw_mid;
  double yaw = reference_yaw;
  double eig_major = 0.0;
  double eig_minor = 0.0;
  double normal_rms = 0.0;
  std::vector<Point2> inliers = candidates;

  for (int iter = 0; iter < 4; ++iter) {
    WeightedLineFit(candidates, weights, t_ref, &anchor, &yaw,
                    &eig_major, &eig_minor, &normal_rms);
    const Point2 normal = NormalFromYaw(yaw);
    inliers.clear();
    for (size_t i = 0; i < candidates.size(); ++i) {
      const double residual = Dot(normal, candidates[i] - anchor);
      weights[i] = HuberWeight(residual, cfg_.support_line_huber_delta);
      if (std::fabs(residual) <= 2.0 * cfg_.support_line_huber_delta) {
        inliers.push_back(candidates[i]);
      }
    }
  }

  SupportLineEstimate current;
  current.valid = true;
  current.anchor_base = anchor;
  current.yaw = yaw;
  UpdateSupportLineFrame(&current);
  current.tangent_span_m = TangentSpan(inliers.empty() ? candidates : inliers,
                                       anchor, current.t);
  current.inlier_count = static_cast<int>(inliers.size());
  current.normal_rms = normal_rms;
  current.condition_ratio = eig_major / std::max(eig_minor, 1e-6);
  current.sigma_yaw_rad =
      std::max(0.75 * M_PI / 180.0,
               std::atan2(std::max(normal_rms, 1e-4),
                          std::max(current.tangent_span_m, 0.05)) +
                   0.5 * M_PI / 180.0);
  current.sigma_offset_m = std::max(cfg_.support_line_sigma_n_m, normal_rms);

  const double span_score =
      Clamp(current.tangent_span_m / std::max(cfg_.support_line_min_span_m, 1e-3),
            0.0, 1.0);
  const double cond_score =
      Clamp(current.condition_ratio /
                std::max(cfg_.support_line_min_condition_ratio, 1e-3),
            0.0, 1.0);
  const double rms_score =
      Clamp(std::exp(-0.5 * std::pow(normal_rms /
                                     std::max(cfg_.support_line_sigma_n_m, 1e-3), 2.0)),
            0.0, 1.0);
  const double inlier_score =
      Clamp(static_cast<double>(current.inlier_count) /
                std::max(12.0, static_cast<double>(cfg_.support_line_min_points)),
            0.0, 1.0);
  current.quality = Clamp(0.35 * span_score + 0.25 * cond_score +
                              0.25 * rms_score + 0.15 * inlier_score,
                          0.0, 1.0);
  current.fit_cost =
      current.normal_rms / std::max(cfg_.support_line_sigma_n_m, 1e-3) +
      1.0 / std::max(current.condition_ratio, 1e-6) +
      1.0 / std::max(current.tangent_span_m, 1e-3);

  if (dbg != nullptr) {
    dbg->inlier_points_base = inliers;
    dbg->eig_major = eig_major;
    dbg->eig_minor = eig_minor;
    dbg->raw_yaw = yaw;
  }

  const bool current_good =
      current.tangent_span_m >= cfg_.support_line_min_span_m &&
      current.condition_ratio >= cfg_.support_line_min_condition_ratio &&
      current.inlier_count >= cfg_.support_line_min_points;

  if (!current_good) {
    *out = MakePriorEstimate(in, reference_yaw);
    if (dbg != nullptr) {
      dbg->blended_yaw = out->yaw;
    }
    if (!out->valid) {
      if (error) *error = "support line degraded and no prior fallback";
      return false;
    }
    if (error) error->clear();
    return true;
  }

  if (in.prior_support_line != nullptr && in.prior_support_line->valid) {
    current.yaw = AxisBlendYaw(current.yaw, in.prior_support_line->yaw,
                               Clamp(cfg_.support_line_blend_ratio, 0.0, 1.0));
    UpdateSupportLineFrame(&current);
    current.source = SupportLineEstimate::Source::kBlended;
  } else {
    current.source = SupportLineEstimate::Source::kCurrentFrameLine;
  }
  current.yaw_frozen = false;

  if (dbg != nullptr) {
    dbg->blended_yaw = current.yaw;
  }

  *out = current;
  if (error) error->clear();
  return true;
}

}  // namespace reflector_localization
visualizer.cpp:
#include "reflector_localization/visualizer.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>

#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

namespace reflector_localization {
namespace {

geometry_msgs::Point ToRosPoint(const Point2& p, double z = 0.05) {
  geometry_msgs::Point pt;
  pt.x = p.x;
  pt.y = p.y;
  pt.z = z;
  return pt;
}

std::vector<Point2> CovarianceEllipsePoints(const PairMeasurement& measurement) {
  std::vector<Point2> points;
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  cov(0, 0) = measurement.covariance(0, 0);
  cov(0, 1) = measurement.covariance(0, 1);
  cov(1, 0) = measurement.covariance(1, 0);
  cov(1, 1) = measurement.covariance(1, 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  Eigen::Vector2d eval = solver.eigenvalues().cwiseMax(1e-6).cwiseSqrt();
  Eigen::Matrix2d evec = solver.eigenvectors();

  constexpr int kPoints = 24;
  points.reserve(kPoints + 1);
  for (int i = 0; i <= kPoints; ++i) {
    const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kPoints);
    const Eigen::Vector2d unit(std::cos(theta), std::sin(theta));
    const Eigen::Vector2d d = evec * eval.asDiagonal() * unit;
    points.push_back(Point2{measurement.midpoint_base.x + d.x(),
                            measurement.midpoint_base.y + d.y()});
  }
  return points;
}

}  // namespace

Visualizer::Visualizer(ros::NodeHandle& nh, const std::string& topic,
                       const std::string& map_frame,
                       const std::string& base_frame)
    : map_frame_(map_frame), base_frame_(base_frame) {
  pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

visualization_msgs::Marker Visualizer::MakeSphere(
    int id, const std::string& ns, const std::string& frame, const Point2& p,
    double scale, float r, float g, float b, float a,
    const ros::Time& stamp) const {
  visualization_msgs::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position = ToRosPoint(p);
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  return m;
}

visualization_msgs::Marker Visualizer::MakeLineStrip(
    int id, const std::string& ns, const std::string& frame,
    const std::vector<Point2>& points, double width, float r, float g, float b,
    float a, const ros::Time& stamp) const {
  visualization_msgs::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.points.reserve(points.size());
  for (const auto& p : points) {
    m.points.push_back(ToRosPoint(p));
  }
  return m;
}

visualization_msgs::Marker Visualizer::MakeArrow(
    int id, const std::string& ns, const std::string& frame, const Point2& p,
    double yaw, double length, double width,
    float r, float g, float b, float a, const ros::Time& stamp) const {
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = frame;
  arrow.header.stamp = stamp;
  arrow.ns = ns;
  arrow.id = id;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.pose.position = ToRosPoint(p);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  arrow.pose.orientation = tf2::toMsg(q);
  arrow.scale.x = length;
  arrow.scale.y = width;
  arrow.scale.z = width;
  arrow.color.r = r;
  arrow.color.g = g;
  arrow.color.b = b;
  arrow.color.a = a;
  return arrow;
}

void Visualizer::Publish(const std::vector<StripEdgeObservation>& strips,
                         const SupportLineEstimate& line,
                         const PairMeasurement& measurement,
                         const PairAssociationResult& association,
                         const ReflectorMap& map,
                         const PoseEstimate& pose_estimate,
                         const ros::Time& stamp) {
  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(clear);
  int id = 0;

  for (const auto& strip : strips) {
    arr.markers.push_back(MakeLineStrip(
        id++, "strip_edges", base_frame_,
        std::vector<Point2>{strip.left.base, strip.center_base, strip.right.base},
        0.01, 0.2f, 0.9f, 0.2f, 0.9f, stamp));
    arr.markers.push_back(MakeSphere(id++, "strip_centers", base_frame_,
                                     strip.center_base, 0.05,
                                     0.9f, 0.9f, 0.2f, 0.9f, stamp));
  }

  if (line.valid) {
    const Point2 t = TangentFromYaw(line.yaw);
    const Point2 a = line.anchor_base - 0.6 * t;
    const Point2 b = line.anchor_base + 0.6 * t;
    arr.markers.push_back(MakeLineStrip(
        id++, "support_line", base_frame_, std::vector<Point2>{a, b},
        0.015, 0.2f, 0.6f, 1.0f, 0.9f, stamp));
    arr.markers.push_back(MakeArrow(id++, "support_line_yaw", base_frame_,
                                    line.anchor_base, line.yaw, 0.25, 0.04,
                                    0.2f, 0.6f, 1.0f, 0.9f, stamp));
  }

  if (measurement.valid) {
    arr.markers.push_back(MakeSphere(id++, "pair_midpoint", base_frame_,
                                     measurement.midpoint_base, 0.07,
                                     1.0f, 0.4f, 0.1f, 1.0f, stamp));
    arr.markers.push_back(MakeLineStrip(
        id++, "pair_geometry", base_frame_,
        std::vector<Point2>{measurement.strip_left_center_base, measurement.midpoint_base,
                            measurement.strip_right_center_base},
        0.02, 1.0f, 0.5f, 0.1f, 0.9f, stamp));
    arr.markers.push_back(MakeArrow(id++, "pair_heading", base_frame_,
                                    measurement.midpoint_base, measurement.heading_base,
                                    0.22, 0.05, 1.0f, 0.5f, 0.1f, 0.9f, stamp));
    arr.markers.push_back(MakeLineStrip(
        id++, "pair_covariance", base_frame_, CovarianceEllipsePoints(measurement),
        0.01, 1.0f, 0.1f, 0.1f, 0.8f, stamp));
  }

  if (association.success && association.map_idx_a >= 0 && association.map_idx_b >= 0 &&
      association.map_idx_a < static_cast<int>(map.reflectors().size()) &&
      association.map_idx_b < static_cast<int>(map.reflectors().size())) {
    const auto& ra = map.reflectors().at(static_cast<size_t>(association.map_idx_a));
    const auto& rb = map.reflectors().at(static_cast<size_t>(association.map_idx_b));
    arr.markers.push_back(MakeSphere(id++, "matched_map", map_frame_,
                                     Point2{ra.x, ra.y}, 0.10,
                                     0.1f, 0.4f, 1.0f, 1.0f, stamp));
    arr.markers.push_back(MakeSphere(id++, "matched_map", map_frame_,
                                     Point2{rb.x, rb.y}, 0.10,
                                     0.1f, 0.4f, 1.0f, 1.0f, stamp));
  }

  if (pose_estimate.pose_map_base.valid) {
    arr.markers.push_back(MakeArrow(id++, "ref_pose", map_frame_,
                                    Point2{pose_estimate.pose_map_base.x,
                                           pose_estimate.pose_map_base.y},
                                    pose_estimate.pose_map_base.yaw,
                                    0.40, 0.07, 1.0f, 0.2f, 0.2f, 1.0f, stamp));
  }

  pub_.publish(arr);
}

}  // namespace reflector_localization

