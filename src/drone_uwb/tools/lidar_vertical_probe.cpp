// Isolated EKF-library experiment. No vehicle process, ROS, or device access.
// Inputs are declared synthetic and are never derived from the EKF output.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include "EKF/ekf.h"
#include "test/sensor_simulator/sensor_simulator.h"

int main(int argc, char **argv)
{
    std::setvbuf(stdout,nullptr,_IONBF,0);
    const char *mode = argc > 1 ? argv[1] : "stock";
    const unsigned seed = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 11;
    const float tilt = argc > 3 ? std::strtof(argv[3], nullptr) : 0.f;
    const bool position_only = std::strcmp(mode, "position_only") == 0;
    const bool aux2d = std::strcmp(mode, "aux2d") == 0;
    const bool ned3d = std::strcmp(mode, "ned3d") == 0;
    if (!position_only && !aux2d && !ned3d && std::strcmp(mode,"stock") != 0
        && std::strcmp(mode,"no_vreset") != 0 && std::strcmp(mode,"rotated_vreset") != 0
        && std::strcmp(mode,"no_output_reset") != 0) { return 2; }
    setenv("LIDAR_PROBE_MODE", mode, 1);
    auto ekf = std::make_shared<Ekf>();
    ekf->init(0);
    auto *params = ekf->getParamHandle();
    params->ekf2_ev_ctrl = (position_only || aux2d) ? 1 : 5;
    params->ekf2_ev_delay = 0.f;
    params->ekf2_evv_noise = .1f;
    params->ekf2_evp_noise = .1f;
    params->ev_pos_body.setZero();
    params->imu_pos_body.setZero();
    ekf->set_in_air_status(false);
    ekf->set_vehicle_at_rest(true);
    SensorSimulator sim(ekf);
    const Quatf attitude(Eulerf(0.f, math::radians(tilt), 0.f));
    sim.simulateOrientation(attitude);
    sim._baro.setData(0.f);
    sim._vio.setRateHz(10);
    extVisionSample ev{};
    ev.pos.setZero(); ev.vel.setZero(); ev.quat = attitude;
    ev.position_var = Vector3f(.01f,.01f,1e6f);
    ev.velocity_var = Vector3f(.01f,.01f,1e4f);
    ev.orientation_var = Vector3f(1e6f,1e6f,1e6f);
    ev.pos_frame = PositionFrame::LOCAL_FRAME_NED;
    ev.vel_frame = ned3d ? VelocityFrame::LOCAL_FRAME_NED : VelocityFrame::BODY_FRAME_FRD;
    ev.quality = 100;
    std::mt19937 random(seed);
    std::normal_distribution<float> noise(0.f,1.f);
    const Vector3f gravity_body = Dcmf(attitude).transpose() * Vector3f(0.f,0.f,-CONSTANTS_ONE_G);
    const uint64_t start = sim.getTime();
    bool started = false, removed = false;
    for (unsigned step=0; step<24000; ++step) {
        const float t = (sim.getTime()-start)*1e-6f;
        if (t >= 20.f && !started) { sim.startExternalVision(); started=true; }
        if (t >= 40.f && !removed) {
            sim.stopBaro(); sim.stopRangeFinder();
            params->ekf2_baro_ctrl=0; params->ekf2_rng_ctrl=0; removed=true;
        }
        // Independent 200 Hz accelerometer/gyro noise; not a SIH sensor replay.
        const Vector3f accel_noise(.02f*noise(random),.02f*noise(random),.02f*noise(random));
        const Vector3f gyro_noise(.001f*noise(random),.001f*noise(random),.001f*noise(random));
        sim._imu.setData(gravity_body+accel_noise,gyro_noise);
        if (step%20 == 0) {
            ev.pos = Vector3f(.01f*noise(random),.01f*noise(random),0.f);
            ev.vel = Vector3f(.01f*noise(random),.01f*noise(random),0.f);
            sim._vio.setData(ev);
            if (aux2d && started) {
                // True synthetic NE observations, not a rotated 2D body scan.
                // Missing body-z cannot be recovered by a frame relabel.
                auxVelSample sample{sim.getTime(),Vector2f(ev.vel(0),ev.vel(1)),Vector2f(.01f,.01f)};
                ekf->setAuxVelData(sample);
            }
        }
        sim.runMicroseconds(5000);
        if (step%2 == 0) {
            const auto &s=ekf->state();
            const auto &flags=ekf->control_status().flags;
            const auto output=ekf->getPosition();
            const auto velocity=ekf->getVelocity();
            const auto &aux=ekf->aid_src_aux_vel();
            const auto &aid=ekf->aid_src_ev_vel();
            std::printf("SAMPLE {\"t\":%.9g,\"delayed_us\":%llu,\"ekf_z\":%.9g,\"ekf_vz\":%.9g,"
                        "\"out_z\":%.9g,\"out_vz\":%.9g,\"ekf_vx\":%.9g,\"ekf_vy\":%.9g,"
                        "\"vz_var\":%.9g,\"z_var\":%.9g,\"vz_reset\":%u,\"z_reset\":%u,"
                        "\"internal_z_valid\":%d,\"internal_vz_valid\":%d,\"ev_vel\":%d,\"fake_hgt\":%d,"
                        "\"baro\":%d,\"aux_last_fuse\":%llu,\"ev_last_fuse\":%llu}\n",
                        double(t),static_cast<unsigned long long>(ekf->time_delayed_us()),
                        double(s.pos(2)+ekf->getEkfGlobalOriginAltitude()),double(s.vel(2)),
                        double(output(2)),double(velocity(2)),double(s.vel(0)),double(s.vel(1)),
                        double(ekf->getVelocityVariance()(2)),double(ekf->getPositionVariance()(2)),
                        unsigned(ekf->state_reset_status().reset_count.velD),unsigned(ekf->get_posD_reset_count()),
                        int(ekf->isLocalVerticalPositionValid()),int(ekf->isLocalVerticalVelocityValid()),
                        int(flags.ev_vel),int(flags.fake_hgt),int(flags.baro_hgt),
                        static_cast<unsigned long long>(aux.time_last_fuse),
                        static_cast<unsigned long long>(aid.time_last_fuse));
        }
    }
    return 0;
}
