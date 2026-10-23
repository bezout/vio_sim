#include "viosim/truth.hpp"
#include <iomanip>
#include <stdexcept>

namespace vio {
namespace {
std::ofstream output(const std::filesystem::path& path) {
    std::ofstream stream;
    stream.exceptions(std::ios::failbit|std::ios::badbit);
    stream.open(path);
    stream<<std::setprecision(17);
    return stream;
}
void vector(std::ostream& out,Vec3 v) { out<<v.x<<','<<v.y<<','<<v.z; }
}
LiveTruthWriter::LiveTruthWriter(const Simulation& simulation,const std::string& optitrack_path) {
    if(!optitrack_path.empty()) {
        optitrack_=output(optitrack_path);
        optitrack_<<"timestamp,x,y,z,qx,qy,qz,qw\n";
    }
    const auto& c=simulation.config();
    if(c.truth_output.empty()) return;
    const std::filesystem::path root=c.truth_output;
    if(std::filesystem::exists(root) && !std::filesystem::is_empty(root))
        throw std::runtime_error("VioSim truth output must be empty: "+root.string());
    std::filesystem::create_directories(root);
    output(root/"config_used.yaml")<<dump_config(c);
    output(root/"motion_resolved.yaml")<<simulation.resolved_motion_config();
    auto calibration=output(root/"calibration.csv");
    calibration<<"camera,fx,fy,cx,cy,alpha,beta,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22,eu,ev\n";
    for(size_t i=0;i<simulation.rig().size();++i) {
        const auto& camera=simulation.camera(i);
        const auto& pose=simulation.rig().BC[i];
        calibration<<i<<','<<camera.fx<<','<<camera.fy<<','<<camera.cx<<','<<camera.cy<<','<<camera.alpha<<','<<camera.beta<<',';
        vector(calibration,pose.p);
        for(double r:pose.R.a) calibration<<','<<r;
        calibration<<','<<camera.eu<<','<<camera.ev<<'\n';
    }
    poses_=output(root/"groundtruth.csv");
    scene_=&simulation.scene();
    floors_=output(root/"floor_truth.csv");
    floors_<<"timestamp_us,simulation_ns,floor_z,kind,body_clearance\n";
    poses_<<"timestamp_us,simulation_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,wx,wy,wz,ax,ay,az\n";
}
void LiveTruthWriter::imu(const ImuSample& sample,int64_t output_timestamp_us) {
    if(optitrack_.is_open()) {
        const auto& state=sample.ideal_state;
        const auto q=quaternion(state.R);
        optitrack_<<double(output_timestamp_us)*1e-6<<',';
        vector(optitrack_,state.p);
        optitrack_<<','<<q[1]<<','<<q[2]<<','<<q[3]<<','<<q[0]<<'\n';
    }
    if(!poses_.is_open()) return;
    const auto& state=sample.ideal_state;
    int kind=0;
    const double height=scene_->support_height(state.p,kind);
    floors_<<output_timestamp_us<<','<<sample.timestamp_ns<<','<<height<<','<<kind<<','<<state.p.z-height<<'\n';
    poses_<<output_timestamp_us<<','<<sample.timestamp_ns<<',';
    vector(poses_,state.p);
    for(double q:quaternion(state.R)) poses_<<','<<q;
    poses_<<',';vector(poses_,state.v);
    poses_<<',';vector(poses_,sample.gyro_bias);
    poses_<<',';vector(poses_,sample.accel_bias);
    poses_<<',';vector(poses_,state.omega);
    poses_<<',';vector(poses_,state.a);poses_<<'\n';
}
void LiveTruthWriter::finish() {
    if(poses_.is_open()) poses_.close();
    if(floors_.is_open()) floors_.close();
    if(optitrack_.is_open()) optitrack_.close();
}
}
