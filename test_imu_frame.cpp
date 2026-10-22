#include <viosim/simulation.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
double difference(const vio::Mat3& a,const vio::Mat3& b) {
    double result=0;
    for(size_t i=0;i<9;++i) result=std::max(result,std::abs(a.a[i]-b.a[i]));
    return result;
}
void compare(const vio::State& a,const vio::State& b,const vio::Rig& rig,const vio::Rig& old_rig,const vio::Mat3& basis) {
    using namespace vio;
    check(norm(a.p-b.p)<1e-8 && norm(a.v-b.v)<1e-8 && norm(a.a-b.a)<1e-7,"World trajectory changed with IMU frame");
    check(difference(a.R,b.R*basis)<1e-8,"Body orientation is not a change of basis");
    check(norm(a.omega-basis.transpose()*b.omega)<1e-8,"Gyroscope axes are inconsistent");
    check(norm(specific_force(a)-basis.transpose()*specific_force(b))<1e-7,"Accelerometer axes are inconsistent");
    for(size_t camera=0;camera<rig.size();++camera) {
        const auto x=rig.camera_pose(a.p,a.R,camera),y=old_rig.camera_pose(b.p,b.R,camera);
        check(norm(x.p-y.p)<1e-8 && difference(x.R,y.R)<1e-8,"Physical camera view changed");
    }
}
}

int main() {
    using namespace vio;
    try {
        Config config;
        check(config.imu_frame=="camera","Default IMU frame must match camera axes");
        const Mat3 basis=imu_basis_rotation(config);
        const Rig rig(config);
        check(difference(rig.BC[0].R,Mat3{})==0,"Camera zero is not aligned with the IMU");
        for(const auto& name:trajectory_names()) {
            if(name=="random_long") continue;
            Config c=config; c.trajectory=name;c.duration=72;
            c.scene=name=="multilevel_walk"?"multilevel":name=="tables_walk"?"flat_tables":"office";
            validate_config(c);
            Config old=c; old.imu_frame="legacy_z_up";
            const Rig new_rig(c),old_rig(old);
            const auto scene=make_scene(c.scene);
            for(double t=0;t<72;t+=.37)
                compare(state_at(t,c,scene.anchor),state_at(t,old,scene.anchor),new_rig,old_rig,basis);
        }
        for(const char* mode:{"rectilinear","circular","scanner","xr_headset","controller","mower"})
            for(int reference:{-1,0}) {
                Config c=config; c.scene="warehouse";c.trajectory="random_long";c.motion_mode=mode;
                c.motion_reference_camera=reference;c.duration=8;validate_config(c);
                Config old=c;old.imu_frame="legacy_z_up";
                const Rig new_rig(c),old_rig(old);const auto scene=make_scene(c.scene);
                TrajectoryCursor a(c,scene,new_rig),b(old,scene,old_rig);
                for(double t=0;t<8;t+=.1) compare(a.at(t),b.at(t),new_rig,old_rig,basis);
            }
        Config still=config;still.trajectory="static";
        check(norm(specific_force(state_at(0,still,{}))-Vec3{0,-9.81,0})<1e-12,"Default gravity must appear on IMU -Y");
        set_option(still,"imu-frame","legacy_z_up");
        check(norm(specific_force(state_at(0,still,{}))-Vec3{0,0,9.81})<1e-12,"Legacy +Z gravity convention was not preserved");
        check(dump_config(still).find("imu_frame: 'legacy_z_up'")!=std::string::npos,"IMU frame missing from saved config");
        still.imu_frame="invalid";bool rejected=false;
        try { validate_config(still); } catch(const std::invalid_argument&) { rejected=true; }
        check(rejected,"Invalid IMU frame accepted");
        std::cout<<"Default/legacy IMU axes, camera views, all trajectories and six motion profiles passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
