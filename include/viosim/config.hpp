#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
namespace vio {
struct CameraExtrinsic {
    std::array<double,9> rotation{{1,0,0,0,1,0,0,0,1}};
    std::array<double,3> translation{{0,0,0}};
    bool rotation_set=false,translation_set=false;
};
struct Config {
    std::string scene="office",trajectory="figure8",difficulty="easy",output="output/sequence",assets;
    std::string camera_model="seucm";
    // Default IMU axes match camera 0: X right, Y down, Z forward.
    // legacy_z_up retains the original X right, Y forward, Z up body frame.
    std::string imu_frame="camera";
    double alpha=4.0/9.0,beta=1.0;
    std::string texture_filter="mipmap",format="png",renderer="auto";
    std::string motion_mode="scanner",motion_config;
    double motion_time_offset=0, motion_time_scale=1;
    double motion_fixed_distance=0,motion_final_hold=0,motion_move_duration=0;
    double duration=8,period=8,size=1,speed_scale=1,warmup=1,ramp=1,cooldown=0;
    double roll_deg=4,pitch_deg=2.5,yaw_deg=18,yaw_offset_deg=0;
    double yaw_turn_start=0,yaw_turn_duration=0,yaw_turn_deg=0;
    double camera_hz=50,imu_hz=500,baseline=.08,fov_deg=120,xi=.8;
    double right_yaw_deg=1,right_roll_deg=.6,clearance=.10;
    double image_noise=-1,extra_blur=-1,brightness=-1,contrast=-1;
    double gyro_noise_density=-1,accel_noise_density=-1,gyro_bias_rw=-1,accel_bias_rw=-1;
    // Explicit biases override difficulty defaults, including in ideal mode.
    std::array<double,3> initial_gyro_bias{},initial_accel_bias{};
    bool initial_gyro_bias_set=false,initial_accel_bias_set=false;
    double gravity=9.81;
    // Live diagnostic export; no images are saved. Calibration errors affect only
    // what the device reports to SLAM, never the physical rendering rig.
    std::string truth_output;
    double reported_focal_scale=1,reported_baseline_scale=1;
    double blackout_start=0,blackout_duration=0;
    double sensor_blur=.45;
    bool lab_reflections=true;
    double lab_vignette=.78;
    int width=640,height=400,threads=4,samples=2,png_compression=1,preview_count=5,depth_every=0;
    int motion_reference_camera=0;
    uint64_t seed=1234;
    uint64_t motion_seed=1234;
    int64_t start_ns=1700000000000000000LL;
    bool dry_run=false,distance_shading=false;
    bool profile_render=false,gpu_precompute_rays=true;
    bool gpu_postprocess=true;
    int camera_count=0;
    std::vector<CameraExtrinsic> camera_extrinsics;
};
const std::vector<std::string>& scene_names();
const std::vector<std::string>& trajectory_names();
const std::vector<std::string>& preset_names();
Config preset_config(const std::string& name);
struct BenchmarkPreset {
    std::string name,category,description;
    Config config;
};
const std::vector<BenchmarkPreset>& benchmark_presets();
void set_option(Config&,const std::string& key,const std::string& value);
void load_config(Config&,const std::filesystem::path&);
void validate_config(Config&);
std::string dump_config(const Config&);
int64_t sample_count(double duration,double rate);
int64_t timestamp(int64_t start,int64_t index,double rate);
std::filesystem::path locate_assets(const Config&,const std::filesystem::path& executable);
} // namespace vio
