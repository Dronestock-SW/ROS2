// Deterministic EKF tests using upstream PX4 sensor simulation, no aircraft I/O.
#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfLidarIndependentTest : public ::testing::Test {
protected:
    std::shared_ptr<Ekf> ekf{std::make_shared<Ekf>()};
    SensorSimulator sim{ekf};
    EkfWrapper wrapper{ekf};
    void SetUp() override {
        ekf->init(0);
        sim.runSeconds(.1f);
        ekf->set_in_air_status(false);
        ekf->set_vehicle_at_rest(true);
        sim.runSeconds(7.f);
        ekf->set_vehicle_at_rest(false);
        ekf->setAuxVelIndependent(true);
        ASSERT_TRUE(ekf->control_status_flags().yaw_align);
        ASSERT_TRUE(ekf->control_status_flags().tilt_align);
    }
    void sendFor(float seconds, Vector2f velocity = Vector2f{0.f, 0.f}) {
        for (unsigned i = 0; i < unsigned(seconds * 10.f); ++i) {
            ekf->setAuxVelData(auxVelSample{sim.getTime(), velocity, Vector2f{.01f, .01f}});
            sim.runSeconds(.1f);
        }
    }
    void expectIndependent() {
        EXPECT_TRUE(ekf->control_status_flags().aux_vel);
        EXPECT_TRUE(ekf->aid_src_aux_vel().fused);
        EXPECT_GT(ekf->aid_src_aux_vel().time_last_fuse, 0u);
        EXPECT_FALSE(ekf->control_status_flags().inertial_dead_reckoning);
        EXPECT_FALSE(ekf->control_status_flags().fake_pos);
        EXPECT_TRUE(ekf->isLocalHorizontalPositionValid());
    }
};

TEST_F(EkfLidarIndependentTest, StartsWithoutAnotherHorizontalSource) {
    const auto vz_resets = ekf->get_velD_reset_count();
    const auto z_resets = ekf->get_posD_reset_count();
    const auto xy_resets = ekf->get_posNE_reset_count();
    sendFor(3.f);
    expectIndependent();
    EXPECT_FALSE(ekf->control_status_flags().ev_pos);
    EXPECT_FALSE(ekf->control_status_flags().ev_vel);
    EXPECT_FALSE(ekf->control_status_flags().gnss_vel);
    EXPECT_FALSE(ekf->control_status_flags().opt_flow);
    EXPECT_EQ(ekf->get_velD_reset_count(), vz_resets);
    EXPECT_EQ(ekf->get_posD_reset_count(), z_resets);
    EXPECT_EQ(ekf->get_posNE_reset_count(), xy_resets);
}

TEST_F(EkfLidarIndependentTest, LegacyAuxiliaryModeStillNeedsAnotherSource) {
    ekf->setAuxVelIndependent(false);
    sendFor(3.f);
    EXPECT_FALSE(ekf->control_status_flags().aux_vel);
    EXPECT_EQ(ekf->aid_src_aux_vel().time_last_fuse, 0u);
}

TEST_F(EkfLidarIndependentTest, GapStopsAidingAndRecoveryStartsAgain) {
    sendFor(3.f);
    expectIndependent();
    sim.runSeconds(.8f);
    EXPECT_FALSE(ekf->control_status_flags().aux_vel);
    EXPECT_FALSE(ekf->aid_src_aux_vel().fused);
    EXPECT_TRUE(ekf->control_status_flags().inertial_dead_reckoning);
    sendFor(2.f);
    expectIndependent();
}

TEST_F(EkfLidarIndependentTest, ReceiverResetWithdrawsAidingImmediately) {
    sendFor(3.f);
    const auto xy = ekf->get_posNE_reset_count();
    const auto vz = ekf->get_velD_reset_count();
    ekf->resetAuxVelData();
    EXPECT_FALSE(ekf->control_status_flags().aux_vel);
    EXPECT_FALSE(ekf->aid_src_aux_vel().fused);
    EXPECT_EQ(ekf->get_posNE_reset_count(), xy);
    EXPECT_EQ(ekf->get_velD_reset_count(), vz);
    sendFor(2.f);
    expectIndependent();
}

TEST_F(EkfLidarIndependentTest, HorizontalVelocityTracksMeasuredMotion) {
    ekf->set_in_air_status(true);
    sendFor(10.f, Vector2f{.2f, -.1f});
    expectIndependent();
    const auto velocity = ekf->getVelocity();
    EXPECT_NEAR(velocity(0), .2f, .025f);
    EXPECT_NEAR(velocity(1), -.1f, .025f);
}

TEST_F(EkfLidarIndependentTest, UwbArrivalLossAndReturnKeepLidarAiding) {
    sendFor(3.f);
    wrapper.enableExternalVisionPositionFusion();
    sim._vio.setPositionFrameToLocalNED();
    sim.startExternalVision();
    sendFor(3.f);
    EXPECT_TRUE(ekf->control_status_flags().ev_pos);
    expectIndependent();
    sim.stopExternalVision();
    sendFor(5.f);
    EXPECT_FALSE(ekf->control_status_flags().ev_pos);
    expectIndependent();
    sim.startExternalVision();
    sendFor(3.f);
    EXPECT_TRUE(ekf->control_status_flags().ev_pos);
    expectIndependent();
}

TEST_F(EkfLidarIndependentTest, RejectedVelocityDoesNotKeepAidingAlive) {
    sendFor(3.f);
    sendFor(1.f, Vector2f{2.f, -2.f});
    EXPECT_FALSE(ekf->control_status_flags().aux_vel);
    EXPECT_FALSE(ekf->aid_src_aux_vel().fused);
}

TEST_F(EkfLidarIndependentTest, HeightLossIsNotReplacedByLidar) {
    sendFor(3.f);
    sim.stopBaro();
    sim.stopRangeFinder();
    sendFor(20.f);
    expectIndependent();
    EXPECT_FALSE(ekf->isLocalVerticalPositionValid());
    EXPECT_FALSE(ekf->isLocalVerticalVelocityValid());
    EXPECT_FALSE(ekf->control_status_flags().ev_hgt);
    sim.startBaro();
    sendFor(5.f);
    EXPECT_TRUE(ekf->isLocalVerticalPositionValid());
    expectIndependent();
}
