#include "viosim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition,const char* message) {
    if(!condition) throw std::runtime_error(message);
}
int views(const vio::Rig& rig,const vio::Camera& camera,vio::Vec3 point) {
    int count=0;
    for(const auto& pose:rig.BC) {
        const auto pixel=camera.project(pose.R.transpose()*(point-pose.p));
        count+=std::isfinite(pixel[0]) && std::isfinite(pixel[1]) &&
               pixel[0]>=0 && pixel[0]<=camera.width-1 && pixel[1]>=0 && pixel[1]<=camera.height-1;
    }
    return count;
}
}

int main(int argc,char** argv) {
    try {
        if(argc<3) throw std::runtime_error("Expected helmet6 configuration and texture directory [gpu]");
        vio::Config config;vio::load_config(config,argv[1]);
        vio::validate_config(config);
        require(config.camera_count==6 && config.width==640 && config.height==480,"Wrong helmet camera count or resolution");
        const vio::Rig rig(config);const vio::Camera camera(config,false);
        const auto left=camera.unproject(0,camera.cy),right=camera.unproject(camera.width-1,camera.cy);
        const double fov=std::atan2(right.x,right.z)-std::atan2(left.x,left.z);
        require(std::abs(fov-vio::radians(125))<1e-12,"Helmet FOV must be 125 degrees");
        for(size_t k=0;k<rig.size();++k) {
            const auto& pose=rig.BC[k];const auto& next=rig.BC[(k+1)%rig.size()];
            const auto forward=pose.R*vio::Vec3{0,0,1};
            require(std::abs(vio::norm(pose.p)-.12)<1e-12 && std::abs(pose.p.y)<1e-12,"Camera is outside the horizontal 24 cm ring");
            require(vio::norm(forward-vio::normalized(pose.p))<1e-12,"Camera does not point radially outward");
            require(vio::norm(pose.R*vio::Vec3{0,1,0}-vio::Vec3{0,1,0})<1e-12,"Camera image is not upright");
            require(std::abs(vio::norm(next.p-pose.p)-.12)<1e-12,"Adjacent baseline must be 12 cm, including closing pair");
            const double angle=std::acos(std::clamp(vio::dot(forward,next.R*vio::Vec3{0,0,1}),-1.,1.));
            require(std::abs(fov-angle-vio::radians(65))<1e-12,"Neighbor overlap must be 65 degrees");
        }
        bool near_monocular=false;
        for(int sample=0;sample<1440;++sample) {
            const double angle=2*vio::pi*(sample+.5)/1440;
            const vio::Vec3 direction{std::sin(angle),0,std::cos(angle)};
            require(views(rig,camera,direction*1e6)>=2,"Far-field horizon has a stereo coverage gap");
            near_monocular|=views(rig,camera,direction)==1;
        }
        require(near_monocular,"Near-field model unexpectedly ignored camera separation");

        config.assets=argv[2];config.renderer=argc>3?"gpu":"cpu";
        config.difficulty="ideal";config.sensor_blur=0;config.duration=.1;
        vio::Simulation simulation(config,argv[0]);
        const auto full=simulation.render_camera_group(simulation.prepare_camera_group(0));
        const auto selected=simulation.render_camera_group(simulation.prepare_camera_group(0,{5,0,3}));
        const auto repeat=simulation.render_camera_group(simulation.prepare_camera_group(0));
        require(full.images.size()==6 && selected.images.size()==3,"Rendered camera count is wrong");
        require(selected.images[0].data==full.images[5].data && selected.images[1].data==full.images[0].data &&
                selected.images[2].data==full.images[3].data,"Camera selection lost physical indices or image seeds");
        for(size_t k=0;k<6;++k) {
            require(repeat.images[k].data==full.images[k].data,"Repeated camera group changed pixels");
            require(full.images[k].data.size()==640*480,"Rendered helmet image has wrong size");
            require(full.images[k].data!=full.images[(k+1)%6].data,"Neighbor cameras rendered identical views");
        }
        std::cout<<"PASS helmet6: 24 cm ring, 12 cm baselines, 125 degree FOV, 65 degree overlap, six images and selection; renderer="
                 <<simulation.renderer_backend()<<simulation.render_profile()<<'\n';
        return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
