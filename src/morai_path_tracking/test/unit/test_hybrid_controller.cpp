#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/controllers/lateral/hybrid_controller.hpp"

namespace morai_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;

HybridConfig competitionConfig() {
  HybridConfig config;
  config.pure_pursuit.wheelbase_m = 3.0;
  config.pure_pursuit.lookahead_base_m = 4.0;
  config.pure_pursuit.lookahead_speed_gain_sec = 0.75;
  config.pure_pursuit.lookahead_curvature_gain_m = 8.0;
  config.pure_pursuit.lookahead_min_m = 4.0;
  config.pure_pursuit.lookahead_max_m = 16.0;
  config.pure_pursuit.minimum_target_distance_m = 0.5;
  config.pure_pursuit.maximum_steering_angle_rad =
      40.0 * kPi / 180.0;

  config.stanley.wheelbase_m = 3.0;
  config.stanley.gain = 2.0;
  config.stanley.softening_speed_mps = 2.0;
  config.stanley.minimum_control_speed_mps = 1.0;
  config.stanley.heading_window_m = 4.0;
  config.stanley.heading_error_gain = 0.5;
  config.stanley.curvature_feedforward_gain = 1.0;
  config.stanley.curvature_preview_distance_m = 8.0;
  config.stanley.yaw_rate_damping_gain_sec = 0.1;
  config.stanley.yaw_rate_damping_nonlinear_gain_sec2 = 0.5;
  config.stanley.maximum_steering_angle_rad =
      40.0 * kPi / 180.0;
  config.stanley.maximum_steering_rate_rad_per_sec =
      60.0 * kPi / 180.0;

  config.imm.mass_kg = 2000.0;
  config.imm.yaw_inertia_kgm2 = 4000.0;
  config.imm.front_cornering_stiffness_n_per_rad = 60000.0;
  config.imm.rear_cornering_stiffness_n_per_rad = 60000.0;
  config.imm.front_axle_to_cg_m = 1.5;
  config.imm.rear_axle_to_cg_m = 1.5;
  config.imm.process_noise_sideslip = 1.0e-4;
  config.imm.process_noise_yaw_rate = 1.0e-4;
  config.imm.measurement_noise_sideslip = 1.0e-4;
  config.imm.measurement_noise_yaw_rate = 1.0e-4;
  config.imm.initial_covariance_sideslip = 1.0e-4;
  config.imm.initial_covariance_yaw_rate = 1.0e-4;
  config.imm.initial_pure_pursuit_probability = 0.5;
  config.imm.initial_stanley_probability = 0.5;
  config.imm.stanley_probability_min = 0.001;
  config.imm.stanley_probability_max = 0.999;
  config.imm.transition_pure_pursuit_to_pure_pursuit = 1.0;
  config.imm.transition_pure_pursuit_to_stanley = 0.0;
  config.imm.transition_stanley_to_pure_pursuit = 0.0;
  config.imm.transition_stanley_to_stanley = 1.0;
  config.imm.transition_speed_gain = 0.0;
  config.imm.transition_reference_speed_mps = 60.0 / 3.6;
  config.imm.minimum_model_speed_mps = 1.0;

  config.maximum_steering_angle_rad = 40.0 * kPi / 180.0;
  config.maximum_steering_rate_rad_per_sec =
      60.0 * kPi / 180.0;
  return config;
}

std::vector<Point2d> straightPath(double lateral_offset_m) {
  return {
      {0.0, lateral_offset_m},
      {10.0, lateral_offset_m},
      {20.0, lateral_offset_m},
      {30.0, lateral_offset_m},
  };
}

std::vector<Point2d> leftCurve(double lateral_offset_m) {
  std::vector<Point2d> path;
  constexpr double radius_m = 12.0;
  for (int x_m = 0; x_m <= 10; ++x_m) {
    path.push_back(
        {static_cast<double>(x_m),
         lateral_offset_m + radius_m -
             std::sqrt(radius_m * radius_m - x_m * x_m)});
  }
  return path;
}

TEST(HybridController, ConvertsRearAxleVelocityToCgSideslip) {
  HybridConfig config = competitionConfig();
  config.imm.stanley_probability_min = 0.5;
  config.imm.stanley_probability_max = 0.5;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(0.0), 10.0, 0.2, 0.4,
                           0.0, 0.0, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.8, result.cg_lateral_velocity_mps, 1.0e-12);
  EXPECT_NEAR(std::atan2(0.8, 10.0),
              result.measured_sideslip_angle_rad, 1.0e-12);
}

TEST(HybridController, BlendsNativeControllerRequestsByImmProbability) {
  HybridConfig config = competitionConfig();
  config.imm.initial_pure_pursuit_probability = 0.75;
  config.imm.initial_stanley_probability = 0.25;
  config.imm.stanley_probability_min = 0.25;
  config.imm.stanley_probability_max = 0.25;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(0.5), 10.0, 0.0, 0.0,
                           0.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  const double expected =
      0.75 * result.pure_pursuit.steering_angle_rad +
      0.25 * result.stanley.requested_steering_angle_rad;
  EXPECT_NEAR(0.75, result.imm.pure_pursuit_probability, 1.0e-12);
  EXPECT_NEAR(0.25, result.imm.stanley_probability, 1.0e-12);
  EXPECT_NEAR(expected, result.requested_steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(expected, result.steering_angle_rad, 1.0e-12);
}

TEST(HybridController,
     AddsConfigurableCrossTrackFeedbackOnlyToHybridPurePursuitCandidate) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.imm.initial_pure_pursuit_probability = 0.75;
  config.imm.initial_stanley_probability = 0.25;
  config.imm.stanley_probability_min = 0.25;
  config.imm.stanley_probability_max = 0.25;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(0.5), 10.0, 0.0, 0.0,
                           0.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  const PurePursuitResult native_pure_pursuit =
      computePurePursuit(straightPath(0.5), 10.0, 0.0,
                         config.pure_pursuit);
  ASSERT_TRUE(native_pure_pursuit.valid);
  EXPECT_NEAR(native_pure_pursuit.steering_angle_rad,
              result.pure_pursuit.steering_angle_rad, 1.0e-12);
  const double expected_corrected_pure_pursuit =
      result.pure_pursuit.steering_angle_rad +
      0.5 * result.stanley.cross_track_feedback_steering_rad;
  EXPECT_NEAR(expected_corrected_pure_pursuit,
              result.corrected_pure_pursuit_steering_angle_rad,
              1.0e-12);
  const double expected_blend =
      0.75 * expected_corrected_pure_pursuit +
      0.25 * result.stanley.requested_steering_angle_rad;
  EXPECT_NEAR(expected_blend, result.requested_steering_angle_rad,
              1.0e-12);
}

TEST(HybridController,
     PrioritizesStrongerSameDirectionRecoveryAtCriticalWheelClearance) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 1.0;
  config.cross_track_recovery_full_scale_m = 2.0;
  config.lane_clearance_recovery_start_m = 0.18;
  config.lane_clearance_recovery_full_m = 0.05;
  config.imm.initial_pure_pursuit_probability = 0.10;
  config.imm.initial_stanley_probability = 0.90;
  config.imm.stanley_probability_min = 0.90;
  config.imm.stanley_probability_max = 0.90;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result = controller.calculate(
      straightPath(0.5), 10.0, 0.0, 0.0, 0.0,
      0.05, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_GT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_GT(result.stanley.requested_steering_angle_rad, 0.0);
  ASSERT_GT(std::abs(result.corrected_pure_pursuit_steering_angle_rad),
            std::abs(result.stanley.requested_steering_angle_rad));
  EXPECT_NEAR(0.90, result.imm.stanley_probability, 1.0e-12);
  EXPECT_TRUE(result.lane_clearance_recovery_active);
  EXPECT_NEAR(1.0, result.lane_clearance_recovery_urgency, 1.0e-12);
  EXPECT_NEAR(1.0, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(0.0, result.effective_stanley_weight, 1.0e-12);
}

TEST(HybridController,
     AnticipatesTightCurveWithStrongerSameDirectionStanleyRequest) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.0;
  config.curve_preview_stanley_weight_start_m_inv = 0.02;
  config.curve_preview_stanley_weight_full_m_inv = 0.06;
  config.curve_preview_stanley_minimum_weight = 0.85;
  config.imm.initial_pure_pursuit_probability = 0.85;
  config.imm.initial_stanley_probability = 0.15;
  config.imm.stanley_probability_min = 0.15;
  config.imm.stanley_probability_max = 0.15;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result = controller.calculate(
      straightPath(1.0), 10.0, 0.0, 0.0, 0.06,
      std::numeric_limits<double>::infinity(), 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_GT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_GT(result.stanley.requested_steering_angle_rad, 0.0);
  ASSERT_GT(std::abs(result.stanley.requested_steering_angle_rad),
            std::abs(result.corrected_pure_pursuit_steering_angle_rad));
  EXPECT_TRUE(result.curve_preview_stanley_recovery_active);
  EXPECT_NEAR(0.85, result.curve_preview_stanley_recovery_weight, 1.0e-12);
  EXPECT_NEAR(0.85, result.effective_stanley_weight, 1.0e-12);
  EXPECT_NEAR(0.15, result.effective_pure_pursuit_weight, 1.0e-12);
}

TEST(HybridController, PreservesImmBlendBelowCurveAnticipationThreshold) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.0;
  config.curve_preview_stanley_weight_start_m_inv = 0.02;
  config.curve_preview_stanley_weight_full_m_inv = 0.06;
  config.curve_preview_stanley_minimum_weight = 0.85;
  config.imm.stanley_probability_min = 0.15;
  config.imm.stanley_probability_max = 0.15;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result = controller.calculate(
      straightPath(1.0), 10.0, 0.0, 0.0, 0.01,
      std::numeric_limits<double>::infinity(), 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_FALSE(result.curve_preview_stanley_recovery_active);
  EXPECT_DOUBLE_EQ(0.0, result.curve_preview_stanley_recovery_weight);
  EXPECT_NEAR(result.imm.stanley_probability,
              result.effective_stanley_weight, 1.0e-12);
}

TEST(HybridController, PreservesImmBlendAtComfortableWheelClearance) {
  HybridConfig config = competitionConfig();
  config.lane_clearance_recovery_start_m = 0.18;
  config.lane_clearance_recovery_full_m = 0.05;
  config.imm.initial_pure_pursuit_probability = 0.75;
  config.imm.initial_stanley_probability = 0.25;
  config.imm.stanley_probability_min = 0.25;
  config.imm.stanley_probability_max = 0.25;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result = controller.calculate(
      straightPath(0.5), 10.0, 0.0, 0.0, 0.0,
      0.30, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_FALSE(result.lane_clearance_recovery_active);
  EXPECT_NEAR(0.0, result.lane_clearance_recovery_urgency, 1.0e-12);
  EXPECT_NEAR(0.75, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(0.25, result.effective_stanley_weight, 1.0e-12);
}

TEST(HybridController,
     PrioritizesPurePursuitWhenCandidatesConflictBeforeACurve) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.candidate_conflict_curvature_threshold_m_inv = 0.01;
  config.candidate_conflict_cross_track_threshold_m = 0.5;
  config.curve_preview_stanley_minimum_weight = 0.85;
  config.stanley.yaw_rate_damping_gain_sec = 1.0;
  config.stanley.yaw_rate_damping_nonlinear_gain_sec2 = 0.0;
  config.imm.stanley_probability_min = 0.9;
  config.imm.stanley_probability_max = 0.9;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(0.1), 10.0, 0.0, 1.0,
                           0.02, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_GT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_LT(result.stanley.requested_steering_angle_rad, 0.0);
  EXPECT_TRUE(result.candidate_conflict_guard_active);
  EXPECT_NEAR(1.0, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(0.0, result.effective_stanley_weight, 1.0e-12);
  EXPECT_NEAR(result.corrected_pure_pursuit_steering_angle_rad,
              result.requested_steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(0.9, result.imm.stanley_probability, 1.0e-12);
}

TEST(HybridController,
     UsesSignedStanleyCurvatureWhenPurePursuitConflictsInsideTightCurve) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.candidate_conflict_curvature_threshold_m_inv = 0.01;
  config.candidate_conflict_cross_track_threshold_m = 0.5;
  config.curve_preview_stanley_minimum_weight = 0.85;
  config.imm.stanley_probability_min = 0.15;
  config.imm.stanley_probability_max = 0.15;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result = controller.calculate(
      leftCurve(-0.8), 3.0, 0.0, 0.0, 0.08,
      std::numeric_limits<double>::infinity(), 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_LT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_GT(result.stanley.reference_curvature_m_inv, 0.01);
  ASSERT_GT(result.stanley.requested_steering_angle_rad, 0.0);
  EXPECT_TRUE(result.candidate_conflict_guard_active);
  EXPECT_TRUE(result.candidate_conflict_stanley_override_active);
  EXPECT_NEAR(0.0, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(1.0, result.effective_stanley_weight, 1.0e-12);
  EXPECT_NEAR(result.stanley.requested_steering_angle_rad,
              result.requested_steering_angle_rad, 1.0e-12);
}

TEST(HybridController,
     PrioritizesCrossTrackRecoveryDuringRecordedSCurveConflict) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.candidate_conflict_curvature_threshold_m_inv = 0.01;
  config.candidate_conflict_cross_track_threshold_m = 0.5;
  config.imm.stanley_probability_min = 0.15;
  config.imm.stanley_probability_max = 0.90;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);
  const std::vector<Point2d> recorded_s_curve = {
      {0.049903315, 0.203561034},
      {0.544345594, 0.277896956},
      {1.037313378, 0.361452375},
      {1.529415376, 0.449963503},
      {2.021517374, 0.538474630},
      {2.512810297, 0.631370930},
      {3.004299416, 0.723218548},
      {3.495788534, 0.815066166},
      {3.987762608, 0.904271923},
      {4.480738773, 0.987741496},
      {4.973714939, 1.071211069},
      {5.468823274, 1.140836830},
      {5.967294213, 1.179366610},
      {6.466853855, 1.199401432},
      {6.966843329, 1.196315531},
      {7.465902291, 1.166125951},
      {7.963738346, 1.120022066},
      {8.458604199, 1.048788482},
      {8.949955971, 0.956300687},
      {9.440252918, 0.858319630},
      {9.766650796, 0.790282095},
      {10.238920980, 0.626137681},
      {10.700990707, 0.435118575},
      {11.149116816, 0.213348389},
      {11.592582369, -0.017595157},
      {12.021638882, -0.274317721},
      {12.443544360, -0.542632248},
      {12.858225647, -0.821984084},
      {13.272906935, -1.101335920},
      {13.672070391, -1.402447500},
      {14.071233848, -1.703559080},
      {14.464940811, -2.011771241},
      {14.851857518, -2.328465823},
      {15.238774226, -2.645160406},
      {15.618130994, -2.970872320},
  };

  const HybridResult result = controller.calculate(
      recorded_s_curve, 5.584721088, 0.0, 0.125691637,
      0.083654219, 0.213053600, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_GT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_LT(result.stanley.requested_steering_angle_rad, 0.0);
  ASSERT_LT(result.stanley.reference_curvature_m_inv, 0.0);
  ASSERT_GT(result.stanley.cross_track_error_m, 0.15);
  EXPECT_TRUE(result.candidate_conflict_guard_active);
  EXPECT_NEAR(1.0, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(0.0, result.effective_stanley_weight, 1.0e-12);
  EXPECT_NEAR(result.corrected_pure_pursuit_steering_angle_rad,
              result.requested_steering_angle_rad, 1.0e-12);
}

TEST(HybridController,
     KeepsPurePursuitForTinyErrorAtStartOfCandidateConflict) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.candidate_conflict_curvature_threshold_m_inv = 0.01;
  config.candidate_conflict_cross_track_threshold_m = 0.5;
  config.imm.stanley_probability_min = 0.15;
  config.imm.stanley_probability_max = 0.90;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);
  const std::vector<Point2d> recorded_curve_entry = {
      {0.150015788, 0.106507881},
      {0.648865575, 0.072621424},
      {1.147715363, 0.038734967},
      {1.647075220, 0.013536085},
      {2.146435077, -0.011662796},
      {2.645794934, -0.036861677},
      {3.145154790, -0.062060559},
      {3.644514647, -0.087259440},
      {4.143874504, -0.112458321},
      {4.643234361, -0.137657203},
      {4.772154021, -0.144162794},
      {5.272148799, -0.142378919},
      {5.772143577, -0.140595044},
      {6.272138355, -0.138811169},
      {6.770881575, -0.103446989},
      {7.269624795, -0.068082809},
      {7.768368015, -0.032718629},
      {8.262012254, 0.046691638},
      {8.755656493, 0.126101904},
      {9.239383843, 0.252595884},
      {9.723111193, 0.379089863},
      {10.198209331, 0.534899849},
      {10.673307469, 0.690709834},
      {11.132210482, 0.889205882},
      {11.591113495, 1.087701930},
      {12.037039252, 1.313843515},
      {12.482965009, 1.539985101},
      {12.906404535, 1.805876167},
      {13.329844062, 2.071767232},
      {13.753283588, 2.337658297},
      {14.150702186, 2.641061416},
      {14.548120783, 2.944464535},
      {14.914526050, 3.284672243},
      {15.280931317, 3.624879951},
      {15.625884806, 3.986822467},
  };

  const HybridResult result = controller.calculate(
      recorded_curve_entry, 9.315444946, 0.0, 0.077970408,
      0.058345456, 0.149211041, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_LT(std::abs(result.stanley.cross_track_error_m), 0.15);
  ASSERT_LT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_GT(result.stanley.requested_steering_angle_rad, 0.0);
  EXPECT_TRUE(result.candidate_conflict_guard_active);
  EXPECT_NEAR(1.0, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(0.0, result.effective_stanley_weight, 1.0e-12);
}

TEST(HybridController,
     RaisesPurePursuitWeightForLargeSameDirectionCrossTrackRecovery) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.candidate_conflict_curvature_threshold_m_inv = 0.01;
  config.cross_track_recovery_full_scale_m = 0.75;
  config.imm.stanley_probability_min = 0.90;
  config.imm.stanley_probability_max = 0.90;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);
  const std::vector<Point2d> recorded_s_curve_excursion = {
      {0.151595559, 0.101842685},
      {0.644227295, 0.187358588},
      {1.135176093, 0.282058226},
      {1.625147082, 0.381692692},
      {2.115118070, 0.481327158},
      {2.604180868, 0.585327345},
      {3.093463563, 0.688283563},
      {3.582746258, 0.791239781},
      {4.072573621, 0.891565800},
      {4.563532743, 0.986179804},
      {5.054491865, 1.080793808},
      {5.547896171, 1.161615845},
      {6.045366538, 1.211426040},
      {6.544344236, 1.242770682},
      {7.044275337, 1.251010268},
      {7.543890060, 1.232132051},
      {8.042642646, 1.197315914},
      {8.538994977, 1.137309250},
      {9.032315533, 1.055974233},
      {9.524705957, 0.969123473},
      {9.852561142, 0.908496261},
      {10.328428018, 0.755090814},
      {10.794705762, 0.574586529},
      {11.247739973, 0.363023233},
      {11.696322599, 0.142183369},
      {12.131083773, -0.104755262},
      {12.558958304, -0.363444853},
      {12.979860490, -0.633332548},
      {13.400762677, -0.903220243},
      {13.806643869, -1.195213577},
      {14.212525060, -1.487206910},
      {14.613111985, -1.786422592},
      {15.007102523, -2.094272317},
      {15.401093061, -2.402122043},
      {15.787729839, -2.719158018},
      {16.174366617, -3.036193993},
      {16.559655229, -3.354866910},
      {16.944943841, -3.673539826},
      {17.330232453, -3.992212743},
      {17.716041262, -4.309955771},
  };

  const HybridResult result = controller.calculate(
      recorded_s_curve_excursion, 5.585116386, 0.0, -0.3,
      0.091556620, -0.149362685, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_LT(result.stanley.reference_curvature_m_inv, 0.0);
  ASSERT_GT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_GT(result.stanley.requested_steering_angle_rad, 0.0);
  ASSERT_GT(std::abs(result.corrected_pure_pursuit_steering_angle_rad),
            std::abs(result.stanley.requested_steering_angle_rad));
  ASSERT_GT(std::abs(result.stanley.cross_track_error_m),
            0.5 * config.cross_track_recovery_full_scale_m);
  EXPECT_TRUE(result.cross_track_recovery_active);
  EXPECT_GT(result.cross_track_recovery_weight, 0.10);
  EXPECT_GT(result.effective_pure_pursuit_weight, 0.10);
  EXPECT_LT(result.effective_stanley_weight, 0.90);
}

TEST(HybridController,
     KeepsImmHeadingCorrectionForRecordedLargeHeadingExcursion) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.60;
  config.cross_track_recovery_full_scale_m = 0.55;
  config.cross_track_recovery_heading_error_suppression_start_rad =
      15.0 * kPi / 180.0;
  config.cross_track_recovery_heading_error_suppression_full_rad =
      17.5 * kPi / 180.0;
  config.cross_track_recovery_heading_error_maximum_suppression_ratio =
      0.30;
  config.imm.stanley_probability_min = 0.10;
  config.imm.stanley_probability_max = 0.10;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);
  const std::vector<Point2d> recorded_front_axle_heading_excursion = {
      {-0.825893016, 0.145950200},
      {0.171099255, 0.074266126},
      {1.155179127, -0.103137986},
      {2.121126444, -0.361490531},
      {3.076060077, -0.658296583},
      {4.013773198, -1.005705127},
      {4.941048314, -1.379927804},
      {5.859607377, -1.775035881},
      {6.772651687, -2.182889642},
      {7.684831023, -2.592679623},
      {8.597519431, -3.000400298},
      {9.510207838, -3.408120972},
      {10.422896246, -3.815841646},
      {11.335584653, -4.223562321},
      {12.250700008, -4.626241661},
      {13.168242309, -5.023879667},
      {14.794477151, -5.712932632},
      {16.640629637, -6.481578740},
      {18.486782124, -7.250224848},
      {20.332934611, -8.018870956},
      {22.179087098, -8.787517064},
      {24.025239585, -9.556163172},
      {25.868861703, -10.331118697},
      {27.709953453, -11.112383639},
      {29.551045202, -11.893648582},
      {31.392136952, -12.674913524},
      {33.233228702, -13.456178467},
      {35.074320451, -14.237443409},
      {36.915412201, -15.018708352},
      {38.756503951, -15.799973294},
      {40.597595700, -16.581238237},
  };

  const HybridResult result = controller.calculate(
      recorded_front_axle_heading_excursion, 4.283557892,
      0.007158718, -0.485238016, 0.054122066,
      -0.422158705, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_GT(std::abs(result.stanley.cross_track_error_m), 0.55);
  ASSERT_GT(std::abs(result.stanley.heading_error_rad),
            config
                .cross_track_recovery_heading_error_suppression_full_rad);
  ASSERT_FALSE(result.candidate_conflict_guard_active);
  ASSERT_GT(
      std::abs(result.corrected_pure_pursuit_steering_angle_rad),
      std::abs(result.stanley.requested_steering_angle_rad));
  EXPECT_FALSE(result.cross_track_recovery_active);
  EXPECT_NEAR(0.70, result.cross_track_recovery_weight, 1.0e-12);
  EXPECT_TRUE(
      result.cross_track_recovery_heading_suppression_active);
  EXPECT_NEAR(
      0.30,
      result.cross_track_recovery_heading_suppression_weight,
      1.0e-12);
  EXPECT_NEAR(0.70, result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(0.30, result.effective_stanley_weight, 1.0e-12);
}

TEST(HybridController,
     RaisesStanleyWeightOnlyForRecordedSameDirectionHeadingLag) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.60;
  config.heading_lag_stanley_weight_start_rad = 8.0 * kPi / 180.0;
  config.heading_lag_stanley_weight_full_rad = 14.0 * kPi / 180.0;
  config.heading_lag_stanley_minimum_weight = 0.55;
  config.imm.stanley_probability_min = 0.15;
  config.imm.stanley_probability_max = 0.15;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);
  const std::vector<Point2d> recorded_curve_entry = {
      {-0.092193724, 0.211059099}, {0.407760340, 0.212996862},
      {0.907446334, 0.196409594},  {1.405868543, 0.156732165},
      {1.901378716, 0.090091886},  {2.394504317, 0.007669515},
      {2.882828162, -0.099597714}, {3.366091520, -0.227804899},
      {3.847900771, -0.361413395}, {4.168442562, -0.453161099},
      {4.627430177, -0.651435868}, {5.074277443, -0.875766545},
      {5.504967484, -1.129745113}, {5.930337973, -1.392531273},
      {6.339451024, -1.679972532}, {6.740583680, -1.978451192},
      {7.133703582, -2.287408731}, {7.526823485, -2.596366270},
      {7.902874349, -2.925889252}, {8.278925214, -3.255412234},
      {8.649014453, -3.591617326}, {9.011710730, -3.935785029},
      {9.374407008, -4.279952731}, {9.728903548, -4.632560177},
      {10.083400088, -4.985167623}, {10.436397887, -5.339275370},
      {10.789395686, -5.693383117}, {11.142393485, -6.047490863},
      {11.495998146, -6.400722817}, {11.849602807, -6.753954770},
      {12.203207469, -7.107186724}, {12.556812130, -7.460418677},
      {12.910416791, -7.813650631}, {13.264021452, -8.166882584},
      {13.617626114, -8.520114537}};

  const HybridResult result = controller.calculate(
      recorded_curve_entry, 5.030441761, 0.0, -0.315771878,
      0.083654932, 0.336278611, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_LT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_LT(result.stanley.requested_steering_angle_rad, 0.0);
  ASSERT_GT(std::abs(result.stanley.requested_steering_angle_rad),
            std::abs(result.corrected_pure_pursuit_steering_angle_rad));
  ASSERT_GT(std::abs(result.stanley.heading_error_rad),
            config.heading_lag_stanley_weight_start_rad);
  EXPECT_TRUE(result.heading_lag_stanley_recovery_active);
  EXPECT_GT(result.heading_lag_stanley_recovery_weight, 0.54);
  EXPECT_NEAR(result.heading_lag_stanley_recovery_weight,
              result.effective_stanley_weight, 1.0e-12);
  EXPECT_NEAR(1.0 - result.heading_lag_stanley_recovery_weight,
              result.effective_pure_pursuit_weight, 1.0e-12);
}

TEST(HybridController,
     LeavesImmBlendUnchangedForSmallStraightCandidateConflict) {
  HybridConfig config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 0.5;
  config.candidate_conflict_curvature_threshold_m_inv = 0.01;
  config.candidate_conflict_cross_track_threshold_m = 0.5;
  config.stanley.yaw_rate_damping_gain_sec = 1.0;
  config.stanley.yaw_rate_damping_nonlinear_gain_sec2 = 0.0;
  config.imm.stanley_probability_min = 0.9;
  config.imm.stanley_probability_max = 0.9;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(0.1), 10.0, 0.0, 1.0,
                           0.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  ASSERT_GT(result.corrected_pure_pursuit_steering_angle_rad, 0.0);
  ASSERT_LT(result.stanley.requested_steering_angle_rad, 0.0);
  EXPECT_FALSE(result.candidate_conflict_guard_active);
  EXPECT_NEAR(result.imm.pure_pursuit_probability,
              result.effective_pure_pursuit_weight, 1.0e-12);
  EXPECT_NEAR(result.imm.stanley_probability,
              result.effective_stanley_weight, 1.0e-12);
}

TEST(HybridController, UsesStanleyRequestBeforeItsStandaloneRateLimit) {
  HybridConfig config = competitionConfig();
  config.imm.stanley_probability_min = 1.0;
  config.imm.stanley_probability_max = 1.0;
  config.stanley.maximum_steering_rate_rad_per_sec =
      1.0 * kPi / 180.0;
  config.maximum_steering_rate_rad_per_sec =
      1000.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(10.0), 5.0, 0.0, 0.0,
                           0.0, 0.0, 0.02);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(result.stanley.requested_steering_angle_rad,
              result.requested_steering_angle_rad, 1.0e-12);
  EXPECT_GT(std::abs(result.requested_steering_angle_rad),
            100.0 * std::abs(result.stanley.steering_angle_rad));
}

TEST(HybridController, AppliesFinalRateLimitOnceAfterBlend) {
  HybridConfig config = competitionConfig();
  config.imm.stanley_probability_min = 0.5;
  config.imm.stanley_probability_max = 0.5;
  config.maximum_steering_rate_rad_per_sec =
      60.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(10.0), 5.0, 0.0, 0.0,
                           0.0, 0.0, 0.02);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.requested_steering_angle_rad,
            1.2 * kPi / 180.0);
  EXPECT_NEAR(1.2 * kPi / 180.0, result.steering_angle_rad,
              1.0e-12);
}

TEST(HybridController, UsesLowerSteeringRateOnLowCurvatureSegments) {
  HybridConfig config = competitionConfig();
  config.imm.stanley_probability_min = 0.5;
  config.imm.stanley_probability_max = 0.5;
  config.low_curvature_maximum_steering_rate_rad_per_sec =
      30.0 * kPi / 180.0;
  config.full_steering_rate_curvature_m_inv = 0.02;
  config.maximum_steering_rate_rad_per_sec =
      60.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(10.0), 5.0, 0.0, 0.0,
                           0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(3.0 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(30.0 * kPi / 180.0,
              result.applied_maximum_steering_rate_rad_per_sec, 1.0e-12);
}

TEST(HybridController, RestoresFullSteeringRateForTightCurves) {
  HybridConfig config = competitionConfig();
  config.imm.stanley_probability_min = 0.5;
  config.imm.stanley_probability_max = 0.5;
  config.low_curvature_maximum_steering_rate_rad_per_sec =
      30.0 * kPi / 180.0;
  config.full_steering_rate_curvature_m_inv = 0.02;
  config.maximum_steering_rate_rad_per_sec =
      60.0 * kPi / 180.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(10.0), 5.0, 0.0, 0.0,
                           0.02, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(6.0 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(60.0 * kPi / 180.0,
              result.applied_maximum_steering_rate_rad_per_sec, 1.0e-12);
}

TEST(HybridController, AllowsFasterSteeringReturnTowardCenter) {
  HybridConfig config = competitionConfig();
  config.imm.initial_pure_pursuit_probability = 0.999;
  config.imm.initial_stanley_probability = 0.001;
  config.imm.stanley_probability_min = 0.001;
  config.imm.stanley_probability_max = 0.001;
  config.maximum_steering_rate_rad_per_sec =
      60.0 * kPi / 180.0;
  config.steering_return_rate_multiplier = 2.0;
  HybridController controller(config);

  const HybridResult result =
      controller.calculate(straightPath(0.0), 5.0, 0.0, 0.0,
                           0.0, -20.0 * kPi / 180.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.0, result.requested_steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(-8.0 * kPi / 180.0, result.steering_angle_rad,
              1.0e-12);
}

TEST(HybridController, KeepsBothControllerReferencePoints) {
  HybridController controller(competitionConfig());

  const HybridResult result =
      controller.calculate(straightPath(0.3), 8.0, 0.0, 0.0,
                           0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.pure_pursuit.lookahead_m, 0.0);
  EXPECT_NEAR(result.pure_pursuit.target.x,
              result.pure_pursuit_target.x, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit.target.y,
              result.pure_pursuit_target.y, 1.0e-12);
  EXPECT_NEAR(result.stanley.target.x,
              result.stanley_projection.x, 1.0e-12);
  EXPECT_NEAR(result.stanley.target.y,
              result.stanley_projection.y, 1.0e-12);
}

TEST(HybridController, RejectsCycleWhenEitherCandidateIsInvalid) {
  HybridController controller(competitionConfig());

  const HybridResult result =
      controller.calculate({{-20.0, 0.0}, {-10.0, 0.0}},
                           5.0, 0.0, 0.0, 0.0, 0.0, 0.1);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.pure_pursuit.valid);
  EXPECT_TRUE(result.stanley.valid);
}

TEST(HybridController, ResetRestoresConfiguredImmPrior) {
  HybridConfig config = competitionConfig();
  config.imm.initial_pure_pursuit_probability = 0.7;
  config.imm.initial_stanley_probability = 0.3;
  HybridController controller(config);
  ASSERT_TRUE(controller
                  .calculate(straightPath(1.0), 10.0, 0.0, 0.4,
                             0.0, 0.0, 0.1)
                  .valid);

  controller.reset();
  const HybridResult result =
      controller.calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                           0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.7, result.imm.pure_pursuit_probability, 1.0e-12);
  EXPECT_NEAR(0.3, result.imm.stanley_probability, 1.0e-12);
}

TEST(HybridController, RejectsInvalidConfigurationAndRuntimeInput) {
  HybridConfig config = competitionConfig();
  config.maximum_steering_rate_rad_per_sec = 0.0;
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  config = competitionConfig();
  config.maximum_steering_angle_rad =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  HybridController controller(competitionConfig());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller
                   .calculate(straightPath(0.0), nan, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);
  EXPECT_FALSE(controller
                   .calculate(straightPath(0.0), 10.0, nan, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);
  EXPECT_FALSE(controller
                   .calculate(straightPath(0.0), 10.0, 0.0, nan,
                              0.0, 0.0, 0.1)
                   .valid);
  EXPECT_FALSE(controller
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.0)
                   .valid);

  config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = -0.1;
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  config = competitionConfig();
  config.pure_pursuit_cross_track_correction_gain = 1.1;
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  config = competitionConfig();
  config.candidate_conflict_curvature_threshold_m_inv = -0.01;
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  config = competitionConfig();
  config.candidate_conflict_cross_track_threshold_m =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  config = competitionConfig();
  config.cross_track_recovery_full_scale_m = 0.0;
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);

  config = competitionConfig();
  config.steering_return_rate_multiplier = 0.5;
  EXPECT_FALSE(HybridController(config)
                   .calculate(straightPath(0.0), 10.0, 0.0, 0.0,
                              0.0, 0.0, 0.1)
                   .valid);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
