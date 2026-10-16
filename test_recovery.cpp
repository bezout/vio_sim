#include "viosim/camera.hpp"
#include "viosim/scene.hpp"
#include "viosim/trajectory.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if(!condition) throw std::runtime_error(message);
}
// Test visibility of sampled surfaces, not image similarity or SLAM features.
// A sample counts as previously seen only if its projection is inside a camera
// and a ray trace confirms the same unoccluded world point.
double known_fraction(const vio::Config& c,double time) {
    const auto scene=vio::make_scene(c.scene);
    const vio::Rig rig(c);
    const vio::Camera camera(c,false);
    std::vector<vio::Pose> previous;
    for(double t=0;t<c.blackout_start;t+=.25) {
        const auto state=vio::state_at(t,c,scene.anchor);
        for(size_t k=0;k<rig.size();++k)
            previous.push_back(rig.camera_pose(state.p,state.R,k));
    }
    int seen=0,total=0;
    const auto state=vio::state_at(time,c,scene.anchor);
    for(size_t k=0;k<rig.size();++k) {
        const auto pose=rig.camera_pose(state.p,state.R,k);
        for(int v=10;v<c.height;v+=20) for(int u=10;u<c.width;u+=20) {
            const auto hit=scene.trace(pose.p,pose.R*camera.unproject(u,v));
            if(hit.object<0) continue;
            ++total;
            for(const auto& prior:previous) {
                const auto local=prior.R.transpose()*(hit.point-prior.p);
                const auto uv=camera.project(local);
                if(!std::isfinite(uv[0]) || !std::isfinite(uv[1])
                   || uv[0]<0 || uv[0]>=c.width || uv[1]<0 || uv[1]>=c.height) continue;
                const auto old=scene.trace(prior.p,prior.R*camera.unproject(uv[0],uv[1]));
                if(old.object==hit.object && old.face==hit.face
                   && vio::norm(old.point-hit.point)<.001) {++seen;break;}
            }
        }
    }
    require(total>0,"No visible surfaces");
    return double(seen)/total;
}
}
int main() {
    try {
        auto known=vio::preset_config("test-camera-blackout-known");
        auto unseen=vio::preset_config("test-camera-blackout-new");
        vio::validate_config(known);vio::validate_config(unseen);
        const auto scene=vio::make_scene(known.scene);
        require(known.blackout_start==12 && known.blackout_duration==2,"Unexpected blackout interval");
        for(double t=0;t<=12;t+=.1) {
            const auto a=vio::state_at(t,known,scene.anchor),b=vio::state_at(t,unseen,scene.anchor);
            require(vio::norm(a.p-b.p)<1e-10 && vio::norm(a.omega-b.omega)<1e-10,
                    "Cases differ before visual loss");
            for(size_t i=0;i<a.R.a.size();++i)
                require(std::abs(a.R.a[i]-b.R.a[i])<1e-10,"Different initial attitudes");
        }
        const auto before=vio::state_at(6,known,scene.anchor);
        const auto after=vio::state_at(14,known,scene.anchor);
        require(vio::norm(before.p-after.p)<1e-10,"Known view does not repeat");
        for(double t:{14.,16.,22.}) {
            const auto a=known_fraction(known,t),b=known_fraction(unseen,t);
            require(a>.99,"Known recovery lacks prior visual coverage");
            require(b<.01,"New recovery still sees old surfaces");
            std::cout<<"time="<<t<<" known_fraction="<<a<<" new_fraction="<<b<<'\n';
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
