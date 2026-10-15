#include "viosim/simulation.hpp"
#include "viosim/gpu_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vio {
namespace {
Config prepare(Config config,const std::filesystem::path& executable) {
    validate_config(config);
    config.assets=locate_assets(config,executable).string();
    return config;
}

}

TrajectoryStats preflight(const Config& config,const Scene& scene,const Rig& rig) {
    TrajectoryCursor trajectory(config,scene,rig);
    TrajectoryStats stats;
    Vec3 previous;
    bool first=true;
    const auto check=[&](double t) {
        const auto state=trajectory.at(t);
        if(!scene.safe(state.p,config.clearance))
            throw std::runtime_error("Trajectory clearance failure for IMU center at t="+std::to_string(t)+" s. Reduce --size, change --scene, or edit scene geometry. No output has been deleted.");
        for(size_t camera=0;camera<rig.size();++camera)
            if(!scene.safe(rig.camera_pose(state.p,state.R,camera).p,config.clearance))
                throw std::runtime_error("Trajectory clearance failure for camera "+std::to_string(camera)+" at t="+std::to_string(t)+" s.");
        if(!first) stats.distance+=norm(state.p-previous);
        first=false;
        previous=state.p;
        stats.max_speed=std::max(stats.max_speed,norm(state.v));
        stats.max_angular_speed=std::max(stats.max_angular_speed,norm(state.omega));
        ++stats.checks;
    };

    if(trajectory.continuously_certified()) {
        double begin=0;
        while(begin<config.duration) {
            trajectory.at(begin);
            const double end=std::min(config.duration,trajectory.current_segment_end());
            constexpr int samples_per_primitive=20;
            for(int sample=0;sample<=samples_per_primitive;++sample)
                check(begin+(end-begin)*double(sample)/samples_per_primitive);
            ++stats.certified_segments;
            if(end>=config.duration) break;
            begin=end;
        }
    } else {
        const double rate=std::max({config.imu_hz,config.camera_hz,200.0});
        const int64_t steps=sample_count(config.duration,rate);
        for(int64_t i=0;i<=steps;++i)
            check(std::min(config.duration,double(i)/rate));
    }
    return stats;
}

Simulation::Simulation(Config config,const std::filesystem::path& executable)
    : config_(prepare(std::move(config),executable)),
      scene_(make_scene(config_.scene)),rig_(config_),camera_(config_,config_.renderer=="cpu"),textures_(config_.assets,config_.scene=="lab_room"),
      imu_trajectory_(std::make_unique<TrajectoryCursor>(config_,scene_,rig_)),
      camera_trajectory_(std::make_unique<TrajectoryCursor>(config_,scene_,rig_)),
      gyro_noise_(mix_seed(config_.seed^0x696d75677972ULL)),
      accel_noise_(mix_seed(config_.seed^0x696d75616363ULL)),
      gyro_bias_noise_(mix_seed(config_.seed^0x626961736779ULL)),
      accel_bias_noise_(mix_seed(config_.seed^0x626961736163ULL)) {
    stats_=preflight(config_,scene_,rig_);
    if(config_.renderer!="cpu") {
        try {
            gpu_renderer_=std::make_shared<GpuRenderer>(camera_,scene_,textures_,config_);
            renderer_backend_="gpu";
            renderer_note_=gpu_renderer_->device_name();
        } catch(const std::exception& error) {
            if(config_.renderer=="gpu") throw;
            renderer_note_=error.what();
            camera_=Camera(config_);
        }
    }
    const bool ideal_imu=config_.gyro_noise_density==0 && config_.accel_noise_density==0 &&
                         config_.gyro_bias_rw==0 && config_.accel_bias_rw==0;
    if(config_.difficulty!="ideal" && !ideal_imu) {
        gyro_bias_={.0015,-.0008,.0012};
        accel_bias_={.025,-.015,.035};
    }
    if(config_.initial_gyro_bias_set) gyro_bias_={config_.initial_gyro_bias[0],config_.initial_gyro_bias[1],config_.initial_gyro_bias[2]};
    if(config_.initial_accel_bias_set) accel_bias_={config_.initial_accel_bias[0],config_.initial_accel_bias[1],config_.initial_accel_bias[2]};
}

std::string Simulation::render_profile() const {
    return gpu_renderer_ ? gpu_renderer_->profile() : std::string();
}

ImuSample Simulation::next_imu() {
    const double dt=1/config_.imu_hz,sdt=std::sqrt(dt);
    const int64_t index=next_imu_index_++;
    const State state=imu_trajectory_->at(double(index)/config_.imu_hz);
    const Vec3 force=specific_force(state,config_.gravity);
    if(index) {
        gyro_bias_+=gyro_bias_noise_.normal3()*(config_.gyro_bias_rw*sdt);
        accel_bias_+=accel_bias_noise_.normal3()*(config_.accel_bias_rw*sdt);
    }
    return {timestamp(config_.start_ns,index,config_.imu_hz),
            state.omega+gyro_bias_+gyro_noise_.normal3()*(config_.gyro_noise_density/sdt),
            force+accel_bias_+accel_noise_.normal3()*(config_.accel_noise_density/sdt),
            gyro_bias_,accel_bias_,state};
}

CameraGroup Simulation::prepare_camera_group(int64_t index,const std::vector<size_t>& selected) {
    CameraGroup group; group.timestamp_ns=timestamp(config_.start_ns,index,config_.camera_hz);
    std::vector<size_t> cameras;
    if(selected.empty()) { cameras.resize(rig_.size()); for(size_t camera=0;camera<rig_.size();++camera) cameras[camera]=camera; }
    else cameras=selected;
    const State body=camera_trajectory_->at(double(index)/config_.camera_hz);
    group.poses.reserve(cameras.size()); group.image_seeds.reserve(cameras.size());
    for(size_t camera:cameras) {
        if(camera>=rig_.size()) throw std::out_of_range("Camera index is outside the VioSim rig");
        group.poses.push_back(rig_.camera_pose(body.p,body.R,camera));
        group.image_seeds.push_back(mix_seed(config_.seed^mix_seed(uint64_t(index)+0x4652414d45ULL)^mix_seed(uint64_t(camera)+0x43414d455241ULL)));
    }
    return group;
}

CameraGroup Simulation::render_camera_group(CameraGroup group) const {
    const double seconds=double(group.timestamp_ns-config_.start_ns)*1e-9;
    if(config_.blackout_duration>0 && seconds>=config_.blackout_start &&
       seconds<config_.blackout_start+config_.blackout_duration) {
        group.images.resize(group.poses.size());
        for(auto& image:group.images) {
            image.width=config_.width; image.height=config_.height;
            image.data.assign(size_t(image.width)*image.height,0);
        }
        return group;
    }
    if(gpu_renderer_) {
        group.images=gpu_renderer_->render(group.poses,group.image_seeds);
        return group;
    }
    group.images.reserve(group.poses.size());
    for(size_t camera=0;camera<group.poses.size();++camera)
        group.images.push_back(render(camera_,scene_,textures_,group.poses[camera],config_,group.image_seeds[camera]));
    return group;
}

} // namespace vio
