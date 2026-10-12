// Offline call-path probe linked to the exact built PX4 EKF library.
// No flight process, sensor device, parameter store or network is accessed.
#include <cstdio>
#include <memory>
#include "EKF/ekf.h"
#include "EKF/aid_sources/external_vision/ev_vel.h"
#include "test/sensor_simulator/sensor_simulator.h"

int main()
{
    for (int axis : {0, 1}) {
      for (float angle : {0.f, 5.f, 10.f, 20.f}) {
        for (float vz_var : {1.f, 100.f, 10000.f, 1000000.f}) {
            auto ekf = std::make_shared<Ekf>();
            ekf->init(0);
            ekf->set_in_air_status(false);
            ekf->set_vehicle_at_rest(true);
            SensorSimulator sim(ekf);
            sim.simulateOrientation(Quatf(Eulerf(axis == 0 ? math::radians(angle) : 0.f,
                                                axis == 1 ? math::radians(angle) : 0.f, 0.f)));
            sim.runSeconds(10.f);
            imuSample imu{};
            imu.delta_ang_dt = .01f;
            extVisionSample sample{};
            sample.vel = Vector3f(0.f, 0.f, .4f);
            sample.velocity_var = Vector3f(.01f, .01f, .01f);
            EvVelBodyFrameFrd before(*ekf, sample, .1f, imu);
            before.resetVelocity();
            const Vector3f old_vel = ekf->getVelocity();
            sample.vel = Vector3f(0.f, 0.f, 0.f);
            sample.velocity_var = Vector3f(.01f, .01f, vz_var);
            EvVelBodyFrameFrd candidate(*ekf, sample, .1f, imu);
            candidate.resetVelocity();
            const Vector3f new_vel = ekf->getVelocity();
            const Vector3f variance = ekf->getVelocityVariance();
            const Eulerf actual(ekf->getQuaternion());
            std::printf("RESULT {\"axis\":\"%s\",\"angle_deg\":%.9g,\"aligned_angle_deg\":%.9g,\"supplied_vz_variance\":%.9g,"
                        "\"before_velocity\":[%.9g,%.9g,%.9g],\"after_velocity\":[%.9g,%.9g,%.9g],"
                        "\"after_variance\":[%.9g,%.9g,%.9g]}\n",
                        axis == 0 ? "roll" : "pitch", double(angle),
                        double(math::degrees(axis == 0 ? actual.phi() : actual.theta())), double(vz_var),
                        double(old_vel(0)), double(old_vel(1)), double(old_vel(2)),
                        double(new_vel(0)), double(new_vel(1)), double(new_vel(2)),
                        double(variance(0)), double(variance(1)), double(variance(2)));
            if (vz_var == 10000.f) {
                // Match the static type used by controlEvVelFusion(). This
                // probes dispatch only; it does not force a height-failure event.
                sample.vel = Vector3f(.5f, .2f, 0.f);
                EvVelBodyFrameFrd vertical(*ekf, sample, .1f, imu);
                ExternalVisionVel &base_reference = vertical;
                base_reference.resetVerticalVelocity();
                const float base_vz = ekf->getVelocity()(2);
                const float base_var = ekf->getVelocityVariance()(2);
                vertical.resetVerticalVelocity();
                const float derived_vz = ekf->getVelocity()(2);
                const float derived_var = ekf->getVelocityVariance()(2);
                std::printf("VERTICAL_DISPATCH {\"axis\":\"%s\",\"angle_deg\":%.9g,"
                            "\"base_vz\":%.9g,\"derived_vz\":%.9g,\"base_variance\":%.9g,"
                            "\"derived_variance\":%.9g,\"automatic_height_failure_tested\":false}\n",
                            axis == 0 ? "roll" : "pitch", double(angle), double(base_vz), double(derived_vz),
                            double(base_var), double(derived_var));
            }
        }
      }
    }
    // Hypothetical geometry only. Neither vector is a measured real mount.
    for (float height : {.05f, .10f, .15f}) {
      for (int axis : {0, 1, 2}) {
        for (float sign : {-1.f, 1.f}) {
            auto ekf = std::make_shared<Ekf>();
            ekf->init(0);
            const Vector3f imu_to_uwb(.02f, -.03f, .04f);
            const Vector3f uwb_to_lidar(0.f, 0.f, height);
            ekf->getParamHandle()->ev_pos_body = imu_to_uwb;
            ekf->getParamHandle()->imu_pos_body = Vector3f(0.f, 0.f, 0.f);
            Vector3f omega{};
            omega(axis) = math::radians(sign * 10.f);
            const Vector3f lidar_velocity(.5f, -.2f, .1f);
            const Vector3f uwb_velocity = lidar_velocity - omega % uwb_to_lidar;
            extVisionSample sample{};
            sample.vel = uwb_velocity;
            sample.velocity_var = Vector3f(.01f, .01f, 10000.f);
            imuSample imu{};
            imu.delta_ang_dt = .01f;
            imu.delta_ang = omega * imu.delta_ang_dt;
            EvVelBodyFrameFrd corrected(*ekf, sample, .1f, imu);
            const Vector3f expected = lidar_velocity - omega % (imu_to_uwb + uwb_to_lidar);
            const float error = (corrected._measurement - expected).norm();
            std::printf("LEVER {\"height_m\":%.9g,\"axis\":%d,\"rate_deg_s\":%.9g,"
                        "\"correction_mps\":%.9g,\"composition_error_mps\":%.9g,\"mount_is_hypothetical\":true}\n",
                        double(height), axis, double(sign * 10.f),
                        double((lidar_velocity - uwb_velocity).norm()), double(error));
            if (error > 1e-6f) { return 2; }
        }
      }
    }
    return 0;
}
