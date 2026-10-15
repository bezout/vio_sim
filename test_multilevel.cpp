#include <viosim/simulation.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool condition,const char* message) {
    if(!condition) throw std::runtime_error(message);
}
int main() {
    try {
        for(const auto* name:{"multilevel","multilevel_tables","flat_tables"}) {
            vio::Config c; c.scene=name;
            c.trajectory=c.scene=="flat_tables"?"tables_walk":"multilevel_walk";
            c.warmup=2; c.duration=72; c.period=70; c.imu_hz=300;
            vio::validate_config(c);
            const auto scene=vio::make_scene(c.scene);
            const vio::Rig rig(c);
            const auto stats=vio::preflight(c,scene,rig);
            require(stats.checks==21601,"incomplete collision preflight");
            for(double t:{3.5,20.0,28.0,39.0,48.0,61.0}) {
                constexpr double dt=1e-4;
                const auto a=vio::state_at(t-dt,c,scene.anchor),b=vio::state_at(t+dt,c,scene.anchor);
                const auto s=vio::state_at(t,c,scene.anchor);
                require(vio::norm((b.p-a.p)/(2*dt)-s.v)<1e-5,"position/velocity mismatch");
                require(vio::norm((b.v-a.v)/(2*dt)-s.a)<1e-5,"velocity/acceleration mismatch");
                const auto rotation=a.R.transpose()*b.R;
                const vio::Vec3 omega{rotation(2,1)-rotation(1,2),rotation(0,2)-rotation(2,0),rotation(1,0)-rotation(0,1)};
                require(vio::norm(omega/(4*dt)-s.omega)<1e-4,"rotation/angular velocity mismatch");
            }
            int kind=0;
            require(scene.support_height({0,2,1.6},kind)==0 && kind==1,"lower floor truth");
            if(c.scene!="flat_tables") {
                require(std::abs(scene.support_height({0,5,2.6},kind)-.6)<1e-9 && kind==2,"tread truth");
                require(scene.support_height({0,12,4.6},kind)==3 && kind==1,"upper floor truth");
            } else {
                require(vio::state_at(21,c,scene.anchor).p.z<1.2,"crouch control missing");
                require(scene.support_height({0,12,1.6},kind)==0,"flat floor gained a level");
            }
            if(c.scene!="multilevel")
                require(scene.support_height({2,2.2,1.6},kind)==0,"table labelled as floor in truth");
        }
        std::cout<<"multilevel geometry, clearance, IMU derivatives and floor truth passed\n";
        return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
