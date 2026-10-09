#include "viosim/dataset.hpp"
#include "viosim/simulation.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#ifdef _OPENMP
#include <omp.h>
#endif
namespace vio {
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
static double elapsed(Clock::time_point start) { return std::chrono::duration<double>(Clock::now()-start).count(); }
static std::ofstream text(const fs::path& path) {
    std::ofstream f(path); f.exceptions(std::ios::failbit|std::ios::badbit); f<<std::setprecision(17); return f;
}
static std::string json_string(const std::string& s) {
    std::ostringstream out; out<<'"';
    for(unsigned char c:s) {
        if(c=='"' || c=='\\') out<<'\\'<<char(c);
        else if(c<32) out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
        else out<<char(c);
    }
    out<<'"'; return out.str();
}
static void vec(std::ostream& out,Vec3 v) { out<<v.x<<','<<v.y<<','<<v.z; }
static void quat(std::ostream& out,const Mat3& R) { const auto q=quaternion(R); out<<q[0]<<','<<q[1]<<','<<q[2]<<','<<q[3]; }
static void calibration(const Config& c,const Camera& camera,const Rig& rig,const fs::path& root) {
    auto f=text(root/"calibration.yaml");
    f<<"# All lengths in meters. T_BC maps camera points into the IMU/body frame.\n"
      <<"camera_model: "<<(camera.seucm?"seucm":"unified")<<"\nresolution: ["<<c.width<<", "<<c.height<<"]\ncamera_rate_hz: "<<c.camera_hz<<"\nimu_rate_hz: "<<c.imu_hz;
    if(c.camera_count==0) f<<"\nstereo_baseline_m: "<<c.baseline;
    else f<<"\ncamera_count: "<<rig.size();
    f<<"\nhorizontal_fov_deg: "<<c.fov_deg<<"\ntime_offset_camera_imu_s: 0\nshutter: global_instantaneous\n";
    for(size_t k=0;k<rig.size();++k) {
        const auto& p=rig.BC[k];
        f<<"cam"<<k<<":\n  intrinsics: ["<<camera.fx<<", "<<camera.fy<<", "<<camera.cx<<", "<<camera.cy;
        if(camera.seucm) f<<", "<<camera.cx<<", "<<camera.cy<<", "<<camera.alpha<<", "<<camera.beta;
        f<<"]\n";
        if(!camera.seucm) f<<"  xi: "<<c.xi<<"\n";
        f<<"  distortion_model: none\n  T_BC:\n    R: [";
        for(int i=0;i<3;++i) { if(i) f<<", "; f<<'['<<p.R(i,0)<<", "<<p.R(i,1)<<", "<<p.R(i,2)<<']'; }
        f<<"]\n    t: ["; vec(f,p.p); f<<"]\n";
    }
    f<<"imu:\n  gravity_world_mps2: [0, 0, -"<<c.gravity<<"]\n"
      <<"  gyro_noise_density_rad_s_sqrtHz: "<<c.gyro_noise_density<<"\n  accel_noise_density_m_s2_sqrtHz: "<<c.accel_noise_density
      <<"\n  gyro_bias_random_walk: "<<c.gyro_bias_rw<<"\n  accel_bias_random_walk: "<<c.accel_bias_rw
      <<"\n  white_noise_discrete_std_rule: 'density / sqrt(dt)'\n  bias_increment_std_rule: 'random_walk_density * sqrt(dt)'\n"
      <<"  quaternion_order: wxyz\n  rotation_direction: body_to_world\n";
}
static void scene_metadata(const Scene& s,const fs::path& root) {
    auto f=text(root/"scene.json");
    f<<"{\n  \"name\": "<<json_string(s.name)<<",\n  \"room_min\": ["; vec(f,s.room.lo); f<<"],\n  \"room_max\": ["; vec(f,s.room.hi);
    f<<"],\n  \"trajectory_anchor\": ["; vec(f,s.anchor); f<<"],\n  \"surface_id_rule\": \"1 + 6*object + face; object=0 is the room; faces=xmin,xmax,ymin,ymax,zmin,zmax\",\n  \"boxes\": [\n";
    for(size_t i=0;i<s.boxes.size();++i) { const auto& b=s.boxes[i]; if(i) f<<",\n"; f<<"    {\"object\":"<<i+1<<",\"min\":["; vec(f,b.bounds.lo); f<<"],\"max\":["; vec(f,b.bounds.hi); f<<"],\"texture_id\":"<<b.texture<<'}'; }
    f<<"\n  ]";
    if(s.name=="office_tag" || s.name=="office_tag_near")
        f<<",\n  \"static_tags\": [{\"family\":\"36h11\",\"id\":0,\"size_m\":0.8,"
          "\"center_world_m\":[-1.7,7,2],\"normal_world\":[0,-1,0]}]";
    f<<"\n}\n";
}
static int64_t imu_files(Simulation& simulation,const fs::path& root) {
    const auto& c=simulation.config();
    // Include one final sample to bracket all camera times, including non-integer rate ratios.
    const int64_t n=sample_count(c.duration,c.imu_hz)+1;
    auto clean=text(root/"imu_clean.csv"),noisy=text(root/"imu.csv"),gt=text(root/"groundtruth.csv");
    clean<<"timestamp_ns,wx,wy,wz,ax,ay,az\n";
    noisy<<"timestamp_ns,wx,wy,wz,ax,ay,az,bgx,bgy,bgz,bax,bay,baz\n";
    gt<<"timestamp_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n";
    for(int64_t i=0;i<n;++i) {
        const auto sample=simulation.next_imu(); const auto ns=sample.timestamp_ns;
        const State& s=sample.ideal_state; const Vec3 force=specific_force(s,c.gravity);
        clean<<ns<<','; vec(clean,s.omega); clean<<','; vec(clean,force); clean<<'\n';
        noisy<<ns<<','; vec(noisy,sample.gyro); noisy<<','; vec(noisy,sample.accel); noisy<<','; vec(noisy,sample.gyro_bias); noisy<<','; vec(noisy,sample.accel_bias); noisy<<'\n';
        gt<<ns<<','; vec(gt,s.p); gt<<','; quat(gt,s.R); gt<<','; vec(gt,s.v); gt<<','; vec(gt,sample.gyro_bias); gt<<','; vec(gt,sample.accel_bias); gt<<'\n';
    }
    return n;
}
static void camera_files(const Config& c,const Rig& rig,int64_t n,const fs::path& root) {
    std::vector<std::ofstream> streams;
    for(size_t camera=0;camera<rig.size();++camera) { streams.push_back(text(root/("cam"+std::to_string(camera))/"data.csv")); streams.back()<<"timestamp_ns,filename\n"; }
    for(int64_t i=0;i<n;++i) {
        const auto ns=timestamp(c.start_ns,i,c.camera_hz); const std::string row=std::to_string(ns)+","+std::to_string(ns)+"."+c.format+"\n";
        for(auto& stream:streams) stream<<row;
    }
}
static std::string dataset_readme() {
    return R"(# Synthetic multi-camera inertial sequence

Generated by viosim_cpp 1.0.0. See config_used.yaml for the resolved settings.

- cam*/data: synchronized global-shutter grayscale images (two or four cameras).
- cam*/data.csv: signed integer nanosecond timestamps and relative filenames.
- imu.csv: angular velocity in rad/s and specific force in m/s^2, then true biases.
- imu_clean.csv: exact analytic instantaneous measurements before bias/noise.
- groundtruth.csv: IMU-origin position, Hamilton quaternion (qw,qx,qy,qz), velocity, biases.
- camera_poses.csv: camera-to-world poses, one row per camera per timestamp.
- calibration.yaml: selected camera model intrinsics and T_BC extrinsics (camera -> body).
- scene.json: room/box bounds and surface-ID definitions.
- motion_resolved.yaml: resolved random_long motion settings, when selected.
- motion_primitives.csv: exact accepted random_long primitive tape.
- debug/preview.png: camera strips; debug/preview_rows.csv gives row timestamps.
- Optional debug/cam*_range/*.pfm: metric range along unit rays at pixel centers.
- Optional debug/cam*_surface_id/*.png: exact hit surface IDs at pixel centers.

World/body axes: X right, Y forward, Z up at the reference orientation.
Camera axes: x right, y down, z forward. Rotations map local vectors to world.
World gravity is specified in calibration.yaml (default magnitude 9.81 m/s2).
The IMU is at the body origin; its acceleration is not the camera-center acceleration.
Read timestamp_ns as int64. Subtract the first timestamp before converting to seconds.

Camera samples cover [0,duration). IMU/GT include an additional endpoint sample.
range is not optical-axis Z depth. PFM stores bottom-to-top little-endian float32.
Surface IDs are per pixel center, not an averaged supersampled semantic label.
The optional depth is ground truth for center rays, whereas image antialiasing blends
subpixel rays; do not expect a unique depth for every mixed edge pixel.

White-noise sample sigma = density/sqrt(dt); bias-walk increment sigma = RW*sqrt(dt).
The stored true biases are for evaluation, not extra estimator input measurements.
The ideal profile removes image read noise and IMU noise/bias, but retains configured optics.

This is a procedural geometric renderer, not a photorealistic physics simulation.
There is no rolling shutter, temporal motion blur, automatic exposure or moving geometry.
A .incomplete marker means generation failed or was interrupted: do not use that sequence.
)";
}
void generate(Config c,const fs::path& executable) {
    const auto start=Clock::now(); validate_config(c);
#ifndef _OPENMP
    if(c.threads!=1) { std::cerr<<"OpenMP not enabled in this build; using one thread.\n"; c.threads=1; }
#endif
    const int64_t n=sample_count(c.duration,c.camera_hz);
    if(c.dry_run) {
        const Scene scene=make_scene(c.scene); const Rig rig(c);
        const auto stats=preflight(c,scene,rig);
        std::cout<<std::fixed<<std::setprecision(3)<<"Preflight: "<<n<<" camera groups ("<<rig.size()<<" cameras), "<<stats.distance<<" m traveled, peak "<<stats.max_speed<<" m/s, "<<stats.max_angular_speed<<" rad/s.\n";
        std::cout<<"Clearance passed at "<<stats.checks<<" sampled poses";
        if(stats.certified_segments) std::cout<<" plus "<<stats.certified_segments<<" continuously certified primitives";
        std::cout<<". No files written.\n";
        return;
    }
    Simulation simulation(c,executable); c=simulation.config();
    c.renderer=simulation.renderer_backend(); c.renderer_explicit=true;
    const auto& camera=simulation.camera(); const auto& scene=simulation.scene(); const auto& rig=simulation.rig();
    const auto& stats=simulation.trajectory_stats();
    std::cout<<std::fixed<<std::setprecision(3)<<"Preflight: "<<n<<" camera groups ("<<rig.size()<<" cameras), "<<stats.distance<<" m traveled, peak "<<stats.max_speed<<" m/s, "<<stats.max_angular_speed<<" rad/s.\n";
    const fs::path root=fs::absolute(c.output).lexically_normal();
    if(fs::exists(root) && (!fs::is_directory(root) || !fs::is_empty(root))) throw std::runtime_error("Output exists and is not empty: "+root.string()+". Choose a new directory; nothing will be deleted.");
    fs::create_directories(root); { auto f=text(root/".incomplete"); f<<"Generation is not complete.\n"; }
    for(size_t k=0;k<rig.size();++k) fs::create_directories(root/("cam"+std::to_string(k))/"data");
    fs::create_directories(root/"debug");
    if(c.depth_every>0) for(size_t k=0;k<rig.size();++k) {
        fs::create_directories(root/"debug"/("cam"+std::to_string(k)+"_range"));
        fs::create_directories(root/"debug"/("cam"+std::to_string(k)+"_surface_id"));
    }
    { auto f=text(root/"config_used.yaml"); f<<dump_config(c); }
    size_t motion_primitive_count=0;
    MotionDiagnostics motion_diagnostics;
    if(!simulation.resolved_motion_config().empty()) {
        { auto f=text(root/"motion_resolved.yaml"); f<<simulation.resolved_motion_config(); }
        auto f=text(root/"motion_primitives.csv");
        TrajectoryCursor tape(c,scene,rig);
        motion_primitive_count=tape.write_primitive_tape(f,c.duration);
        motion_diagnostics=tape.diagnostics();
    }
    { auto f=text(root/"README.md"); f<<dataset_readme(); }
    calibration(c,camera,rig,root); scene_metadata(scene,root);
    const int64_t ni=imu_files(simulation,root); camera_files(c,rig,n,root);
    const int count=std::min<int64_t>(c.preview_count,n);
    std::vector<int64_t> indices;
    for(int j=0;j<count;++j) indices.push_back(count==1?0:int64_t(std::llround(double(j)*(n-1)/(count-1))));
    std::vector<Image8> previews(size_t(count)*rig.size());
    std::atomic<int64_t> completed{0}; std::atomic<bool> failed{false}; std::mutex mutex; std::string error;
    const auto render_start=Clock::now();
    auto poses=text(root/"camera_poses.csv"); poses<<"timestamp_ns,camera,px,py,pz,qw,qx,qy,qz\n";
    constexpr int64_t camera_chunk_size=256;
    for(int64_t chunk_begin=0;chunk_begin<n && !failed.load();chunk_begin+=camera_chunk_size) {
        const int64_t chunk_end=std::min(n,chunk_begin+camera_chunk_size);
        std::vector<CameraGroup> groups;
        groups.reserve(size_t(chunk_end-chunk_begin));
        for(int64_t i=chunk_begin;i<chunk_end;++i) {
            groups.push_back(simulation.prepare_camera_group(i));
            const auto& group=groups.back();
            for(size_t k=0;k<group.poses.size();++k) {
                poses<<group.timestamp_ns<<",cam"<<k<<','; vec(poses,group.poses[k].p); poses<<','; quat(poses,group.poses[k].R); poses<<'\n';
            }
        }
#ifdef _OPENMP
        omp_set_dynamic(0);
#pragma omp parallel for schedule(dynamic,1) num_threads(c.threads)
#endif
        for(int64_t local=0;local<chunk_end-chunk_begin;++local) {
            if(failed.load()) continue;
            try {
                const int64_t i=chunk_begin+local;
                const auto ns=std::to_string(timestamp(c.start_ns,i,c.camera_hz));
                auto group=simulation.render_camera_group(std::move(groups[size_t(local)]));
                for(size_t k=0;k<rig.size();++k) {
                    const Pose& pose=group.poses[k];
                    Image8 image=std::move(group.images[k]);
                    const fs::path target=root/("cam"+std::to_string(k))/"data"/(ns+"."+c.format),temp=target.string()+".tmp";
                    if(c.format=="png") write_png(temp,image,c.png_compression); else write_pgm(temp,image);
                    fs::rename(temp,target);
                    for(size_t j=0;j<indices.size();++j) if(i==indices[j]) previews[rig.size()*j+k]=image;
                    if(c.depth_every>0 && i%c.depth_every==0) {
                        const auto g=render_geometry(camera,scene,pose);
                        write_pfm(root/"debug"/("cam"+std::to_string(k)+"_range")/(ns+".pfm"),c.width,c.height,g.range);
                        write_png(root/"debug"/("cam"+std::to_string(k)+"_surface_id")/(ns+".png"),g.surface_id,c.png_compression);
                    }
                }
                const auto done=completed.fetch_add(1)+1;
                if(done%50==0 || done==n) { std::lock_guard<std::mutex> guard(mutex); std::cout<<"Rendered "<<done<<'/'<<n<<" camera groups ("<<elapsed(render_start)<<" s)\n"<<std::flush; }
            } catch(const std::exception& e) {
                std::lock_guard<std::mutex> guard(mutex); if(!failed.exchange(true)) error=e.what();
            }
        }
    }
    if(failed.load()) throw std::runtime_error("Generation failed; partial output retained with .incomplete marker: "+error);
    const double render_seconds=elapsed(render_start);
    if(count>0) {
        Image8 montage{c.width*int(rig.size()),c.height*count,std::vector<uint8_t>(size_t(c.width)*rig.size()*c.height*count)};
        auto rows=text(root/"debug/preview_rows.csv"); rows<<"row,frame_index,timestamp_ns,time_s\n";
        for(int j=0;j<count;++j) {
            rows<<j<<','<<indices[size_t(j)]<<','<<timestamp(c.start_ns,indices[size_t(j)],c.camera_hz)<<','<<double(indices[size_t(j)])/c.camera_hz<<'\n';
            for(size_t k=0;k<rig.size();++k) for(int y=0;y<c.height;++y) {
                const auto& src=previews[rig.size()*size_t(j)+k];
                std::copy_n(src.data.data()+size_t(y)*c.width,c.width,montage.data.data()+size_t(j*c.height+y)*montage.width+k*c.width);
            }
        }
        write_png(root/"debug/preview.png",montage,c.png_compression);
    }
    const double seconds=elapsed(start);
    const auto profile=simulation.render_profile();
    if(!profile.empty()) std::cout<<profile<<'\n';
    { auto f=text(root/"manifest.json");
      f<<"{\n  \"generator\": \"viosim_cpp\",\n  \"version\": \"1.0.0\",\n  \"complete\": true,\n  \"scene\": "<<json_string(c.scene)<<",\n  \"trajectory\": "<<json_string(c.trajectory)<<",\n  \"difficulty\": "<<json_string(c.difficulty)
       <<",\n  \"duration_s\": "<<c.duration<<",\n  \"camera_groups\": "<<n;
      if(rig.size()==2) f<<",\n  \"stereo_pairs\": "<<n;
      f<<",\n  \"imu_samples\": "<<ni<<",\n  \"resolution\": ["<<c.width<<','<<c.height<<"],\n  \"camera_hz\": "<<c.camera_hz<<",\n  \"imu_hz\": "<<c.imu_hz
       <<",\n  \"camera_count\": "<<rig.size()
       <<",\n  \"renderer\": "<<json_string(c.renderer);
      if(c.camera_count==0) f<<",\n  \"baseline_m\": "<<c.baseline;
      f<<",\n  \"camera_model\": "<<json_string(c.camera_model);
      if(camera.seucm) f<<",\n  \"alpha\": "<<camera.alpha<<",\n  \"beta\": "<<camera.beta;
      else f<<",\n  \"xi\": "<<c.xi;
      f<<",\n  \"horizontal_fov_deg\": "<<c.fov_deg<<",\n  \"seed\": "<<c.seed
       <<",\n  \"motion_seed\": "<<json_string(std::to_string(c.motion_seed))
       <<",\n  \"trajectory_algorithm\": "<<json_string(simulation.trajectory_algorithm())
       <<",\n  \"trajectory_signature\": "<<json_string(simulation.trajectory_signature())
       <<",\n  \"supersampling_per_axis\": "<<c.samples<<",\n  \"texture_filter\": "<<json_string(c.texture_filter)<<",\n  \"threads_requested\": "<<c.threads
       <<",\n  \"trajectory_distance_m\": "<<stats.distance<<",\n  \"peak_speed_mps\": "<<stats.max_speed<<",\n  \"peak_angular_speed_rad_s\": "<<stats.max_angular_speed
       <<",\n  \"clearance_check_samples\": "<<stats.checks
       <<",\n  \"continuously_certified_primitives\": "<<stats.certified_segments
       <<",\n  \"exported_motion_primitives\": "<<motion_primitive_count
       <<",\n  \"motion_events\": "<<motion_diagnostics.events
       <<",\n  \"motion_candidates\": "<<motion_diagnostics.candidates
       <<",\n  \"motion_rejected_candidates\": "<<motion_diagnostics.rejected
       <<",\n  \"motion_blocked_holds\": "<<motion_diagnostics.blocked_holds
       <<",\n  \"motion_safe_retreats\": "<<motion_diagnostics.safe_retreats
       <<",\n  \"render_and_image_io_seconds\": "<<render_seconds<<",\n  \"generation_seconds\": "<<seconds<<",\n  \"camera_groups_per_second_total\": "<<double(n)/seconds<<"\n}\n";
    }
    fs::remove(root/".incomplete");
    std::cout<<"Generated "<<root.string()<<"\nTotal: "<<seconds<<" s, "<<double(n)/seconds<<" camera groups/s including export.\n";
}
} // namespace vio
