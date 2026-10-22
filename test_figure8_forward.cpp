#include <viosim/simulation.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("Expected scenario path");
        vio::Config c; vio::load_config(c,argv[1]); vio::validate_config(c);
        const auto dataset_config=c;
        for(int variant=0;variant<4;++variant) {
            c=dataset_config;
            if(variant>=2) { c.camera_count=0; c.camera_extrinsics.clear(); }
            c.motion_reference_camera=variant%2;
            vio::validate_config(c);
            const auto scene=vio::make_scene(c.scene);
            const vio::Rig rig(c);
            vio::preflight(c,scene,rig);
            const double end=c.warmup+.5*c.ramp+3*c.period/c.speed_scale;
            if(std::abs(vio::motion_clock(end,c).v/c.period-3)>1e-12)
                throw std::runtime_error("Not three complete laps");
            if(vio::norm(vio::state_at(end,c,scene.anchor).p-scene.anchor)>1e-10)
                throw std::runtime_error("Eight did not close");
            const auto point=[&](double phase) {
                return vio::state_at(c.warmup+.5*c.ramp+phase*c.period/(2*vio::pi*c.speed_scale),c,scene.anchor).p-scene.anchor;
            };
            if(c.figure8_height>0 && std::abs(vio::dot(vio::cross(point(vio::pi/6),point(5*vio::pi/6)),point(vio::pi/2)))<1e-4)
                throw std::runtime_error("Eight is still contained in a tilted plane");
            for(double t=0;t<end;t+=.007) {
                const auto s=vio::state_at(t,c,scene.anchor);
                if(std::abs(s.p.z-scene.anchor.z)>c.figure8_height*c.size+1e-12)
                    throw std::runtime_error("Height excursion exceeds the configured handheld envelope");
                const auto forward=s.R*rig.BC[c.motion_reference_camera].R*vio::Vec3{0,0,1};
                if(vio::norm(s.v)>1e-10 && vio::dot(vio::normalized(forward),vio::normalized(s.v))<1-1e-12)
                    throw std::runtime_error("Camera is not looking along the velocity");
                const double dt=1e-5;
                const auto a=vio::state_at(t-dt,c,scene.anchor),b=vio::state_at(t+dt,c,scene.anchor);
                const auto rotation=a.R.transpose()*b.R;
                const vio::Vec3 omega{rotation(2,1)-rotation(1,2),rotation(0,2)-rotation(2,0),rotation(1,0)-rotation(0,1)};
                if(vio::norm(omega/(4*dt)-s.omega)>1e-6)
                    throw std::runtime_error("Gyroscope does not match camera rotation");
                if(vio::norm((b.p-a.p)/(2*dt)-s.v)>1e-7 || vio::norm((b.v-a.v)/(2*dt)-s.a)>1e-6)
                    throw std::runtime_error("Inconsistent translation derivatives");
            }
        }
        std::cout<<"Three closed eights, both reference cameras in dataset/default rigs, IMU derivatives and collision preflight passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
