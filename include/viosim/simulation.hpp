#pragma once

#include "camera.hpp"
#include "renderer.hpp"
#include "trajectory.hpp"

#include <filesystem>
#include <memory>
#include <vector>

namespace vio {

class GpuRenderer;

struct TrajectoryStats {
    double distance=0,max_speed=0,max_angular_speed=0;
    int64_t checks=0,certified_segments=0;
};

struct ImuSample {
    int64_t timestamp_ns=0;
    Vec3 gyro,accel,gyro_bias,accel_bias;
    State ideal_state;
};

struct CameraGroup {
    int64_t timestamp_ns=0;
    std::vector<Pose> poses;
    std::vector<uint64_t> image_seeds;
    std::vector<Image8> images;
};

TrajectoryStats preflight(const Config&,const Scene&,const Rig&);

class Simulation {
public:
    Simulation(Config config,const std::filesystem::path& executable);

    const Config& config() const { return config_; }
    const Scene& scene() const { return scene_; }
    const Rig& rig() const { return rig_; }
    const Camera& camera() const { return camera_; }
    const TrajectoryStats& trajectory_stats() const { return stats_; }
    const std::string& renderer_backend() const { return renderer_backend_; }
    const std::string& renderer_note() const { return renderer_note_; }
    const std::string& trajectory_algorithm() const { return camera_trajectory_->algorithm(); }
    const std::string& trajectory_signature() const { return camera_trajectory_->signature(); }
    const std::string& resolved_motion_config() const { return camera_trajectory_->resolved_config(); }

    ImuSample next_imu();
    std::string render_profile() const;
    CameraGroup prepare_camera_group(int64_t index,const std::vector<size_t>& cameras={});
    CameraGroup render_camera_group(CameraGroup group) const;

private:
    Config config_;
    Scene scene_;
    Rig rig_;
    Camera camera_;
    TextureBank textures_;
    std::unique_ptr<TrajectoryCursor> imu_trajectory_,camera_trajectory_;
    std::shared_ptr<GpuRenderer> gpu_renderer_;
    TrajectoryStats stats_;
    std::string renderer_backend_="cpu",renderer_note_;
    int64_t next_imu_index_=0;
    Noise gyro_noise_,accel_noise_,gyro_bias_noise_,accel_bias_noise_;
    Vec3 gyro_bias_,accel_bias_;
};

} // namespace vio
