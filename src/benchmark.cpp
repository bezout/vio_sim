#include "viosim/config.hpp"
#include "viosim/math.hpp"

namespace vio {
const std::vector<BenchmarkPreset>& benchmark_presets() {
    static const auto presets=[] {
        std::vector<BenchmarkPreset> result;
        Config base;
        base.trajectory="random_long";
        base.motion_mode="controller";
        base.difficulty="ideal";
        base.duration=30;
        base.warmup=2;
        base.gravity=9.80665; // Same reference as SLAM: isolate bias estimation.
        base.samples=1;
        base.preview_count=0;
        base.initial_gyro_bias_set=base.initial_accel_bias_set=true;
        auto add=[&](const char* name,const char* category,const char* description,Config config) {
            result.push_back({name,category,description,std::move(config)});
        };
        add("test-reference","accuracy","6DoF, ideal sensors, zero biases; reference for paired tests",base);
        Config c=base;
        c.duration=120;c.motion_mode="scanner";c.scene="warehouse";
        add("test-drift-long","accuracy","120 s in warehouse; drift and local relative errors",c);
        c=base;c.scene="corridor";c.motion_mode="rectilinear";
        c.motion_fixed_distance=10;c.motion_move_duration=5;c.motion_final_hold=2;c.duration=9.00001;
        add("test-corridor-10m","accuracy","10 m forward, two static seconds at each end; weak excitation",c);
        c=base;c.trajectory="figure8";c.period=8;c.duration=35;
        add("test-loop","accuracy","Repeated closed figure-eight; closure residual and cumulative drift",c);
        c=base;c.motion_time_scale=2;
        add("test-fast","robustness","Same generator at twice the speed; acceleration multiplied by four",c);
        c=base;c.scene="warehouse";c.motion_mode="scanner";c.difficulty="hard";
        add("test-warehouse-hard","robustness","Warehouse with hard image and IMU noise profile",c);
        c=base;c.scene="lab_room";c.motion_mode="scanner";c.difficulty="hard";
        add("test-lab-hard","robustness","Plain surfaces, highlights, vignette and hard sensor profile",c);
        c=base;c.brightness=.35;c.contrast=.35;c.image_noise=2;
        add("test-low-contrast","robustness","Reduced brightness/contrast with image noise; ideal IMU",c);
        c=base;c.extra_blur=2;
        add("test-blur","robustness","Gaussian optical blur (not rolling shutter or motion blur)",c);
        c=base;c.camera_hz=20;
        add("test-low-camera-rate","robustness","20 Hz stereo with 500 Hz IMU",c);
        c=base;c.trajectory="static";c.duration=10;
        add("test-static","degenerate","No motion: absence of initialization can be legitimate",c);
        c=base;c.trajectory="pure_rotation";c.roll_deg=25;c.pitch_deg=20;c.yaw_deg=80;
        add("test-rotation-only","degenerate","Fixed IMU center; rotation and weak translation observability",c);
        c=base;c.initial_gyro_bias={.005,-.003,.002};c.initial_accel_bias={.08,-.06,.04};
        add("test-bias-known","bias","Known nonzero constant biases on all axes; no sensor noise",c);
        for(auto& value:c.initial_gyro_bias) value=-value;
        for(auto& value:c.initial_accel_bias) value=-value;
        add("test-bias-negative","bias","Opposite biases on the same trajectory; sign sensitivity",c);
        c.difficulty="easy";c.gyro_bias_rw=.0001;c.accel_bias_rw=.001;
        add("test-bias-walk","bias","Known evolving biases and white noise; ground truth at every IMU sample",c);
        c=base;c.motion_time_offset=4;c.warmup=0;
        add("test-moving-start","initialization","Starts inside a controller gesture, without a stationary prefix",c);
        c.motion_time_scale=2;c.difficulty="hard";
        add("test-moving-start-hard","initialization","Moving start, doubled motion rate and hard sensor profile",c);
        c=base;c.duration=60;c.reported_focal_scale=1.03;
        add("test-recalib-focal","recalibration","Reported fx/fy +3%; physical camera unchanged; paired online on/off",c);
        c=base;c.duration=60;c.reported_baseline_scale=1.05;
        add("test-recalib-baseline","recalibration","Reported camera translations +5%; physical rig unchanged; paired on/off",c);
        c=base;c.duration=90;c.scene="warehouse";c.motion_mode="scanner";
        add("test-map-budget","map","Long trajectory with a 50-keyframe budget versus unlimited map",c);
        c=base;c.duration=40;c.blackout_start=12;c.blackout_duration=2;
        add("test-camera-blackout","relocalization","All cameras black from 12 to 14 s; IMU continues; measure tracking recovery",c);
        // The first full loop is visible before the blackout. The known case
        // returns to an already observed view; the new case turns toward the
        // previously unseen rear half of the room while all cameras are black.
        c=base;c.trajectory="figure8";c.duration=24;c.period=8;c.size=.35;
        c.camera_hz=30;c.roll_deg=c.pitch_deg=c.yaw_deg=0;
        c.blackout_start=12;c.blackout_duration=2;
        c.yaw_turn_start=12;c.yaw_turn_duration=2;
        add("test-camera-blackout-known","relocalization","Blackout 12-14 s; return to an already observed view on a repeated loop",c);
        c.yaw_turn_deg=180;
        add("test-camera-blackout-new","relocalization","Blackout 12-14 s with a half-turn; recover on previously unseen landmarks",c);
        c=base;c.imu_hz=1000;
        add("test-imu-1000hz","throughput","50 Hz stereo and 1000 Hz IMU; compare reference and scheduler drops",c);
        c=base;c.imu_hz=1000;c.camera_count=4;
        c.width=640;c.height=480;c.alpha=.6085298657;c.beta=1.083426476;
        c.camera_extrinsics.resize(4);
        const std::array<std::array<double,3>,4> translations{{
            {{-.02,.01,.01}},
            {{ .11,.01,.01}},
            {{-.04,.01,-.01}},
            {{ .14,.01,-.01}}
        }};
        const std::array<double,4> yaw_degrees{{-10.,10.,-70.,70.}};
        for(size_t i=0;i<4;++i) {
            auto& camera=c.camera_extrinsics[i];
            // Representative four-fisheye calibration: two forward cameras
            // and two lateral cameras, all expressed camera-to-IMU (R_BC,T_BC).
            camera.rotation=ry(radians(yaw_degrees[i])).a;
            camera.translation=translations[i];
            camera.rotation_set=camera.translation_set=true;
        }
        add("test-quad-1000hz","throughput","Four calibrated fisheyes at 50 Hz and IMU at 1000 Hz",c);
        return result;
    }();
    return presets;
}
} // namespace vio
