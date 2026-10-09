#include "viosim/lab_room.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vio {
namespace {
Material texture(int id,double us=1,double vs=1,double uo=0,double vo=0,bool repeat=true) {
    return {id,0,us,vs,uo,vo,repeat};
}
Material solid(double value) { return {-1,value,1,1,0,0,true}; }
void rectangle(Surface& wall,double u0,double u1,double v0,double v1,Material material) {
    wall.decals.push_back({u0,u1,v0,v1,material});
}
void decal(Surface& wall,double u0,double u1,double v0,double v1,int id) {
    rectangle(wall,u0,u1,v0,v1,texture(id,1/(u1-u0),1/(v1-v0),-u0/(u1-u0),-v0/(v1-v0),false));
}
void box(Scene& scene,Vec3 lo,Vec3 hi,int base,int front_face=-1,int front=-1) {
    Box item{{lo,hi},base};
    if(front_face>=0) item.face_textures[size_t(front_face)]=front;
    scene.boxes.push_back(item);
}
struct Light { double x,y,half_x,half_y; };
constexpr std::array<Light,5> lights={{{-1.3,2.0,.65,.105},{1.3,2.5,.65,.105},
    {-1.25,4.65,.65,.105},{1.25,5.35,.65,.105},{0.0,7.0,.65,.105}}};
constexpr double ceiling_z=3.15;
} // namespace

Scene make_lab_room() {
    Scene scene; scene.name="lab_room";
    scene.room={{-3.8,0,0},{3.8,8.8,ceiling_z}};
    scene.anchor={0,2.30,1.55};
    scene.walls[0].base=scene.walls[1].base=texture(LAB_PLASTER,1/8.8,1/ceiling_z,0,0,false);
    scene.walls[2].base=scene.walls[3].base=texture(LAB_PLASTER,1/7.6,1/ceiling_z,.5,0,false);
    scene.walls[4].base=texture(LAB_FLOOR,1/7.6,1/8.8,.5,0,false);
    scene.walls[5].base=solid(140);

    // Furniture remains peripheral so the central aisle stays completely free.
    for(int i=0;i<6;++i) {
        const double y=1.05+.91*i;
        box(scene,{-3.67,y,0},{-2.87,y+.87,2.19+.045*(i%2)},LAB_METAL,1,(i%2)?LAB_LOCKERS_B:LAB_LOCKERS_A);
    }
    for(int i=0;i<4;++i) {
        const double y=.80+1.12*i;
        box(scene,{2.88,y,0},{3.68,y+1.07,2.60+.025*(i%2)},LAB_METAL,0,LAB_CABINET);
    }
    box(scene,{2.96,7.65,0},{3.7,8.52,2.59},LAB_METAL,2,LAB_CABINET);

    box(scene,{-.42,7.91,.04},{-.35,8.52,2.52},LAB_METAL);
    box(scene,{1.12,7.91,.04},{1.19,8.52,2.52},LAB_METAL);
    for(double z:{.12,.73,1.34,1.96}) box(scene,{-.42,7.88,z},{1.19,8.53,z+.055},LAB_METAL);
    box(scene,{-.27,7.95,.18},{.25,8.40,.69},LAB_CARDBOARD);
    box(scene,{.37,8.04,.18},{1.04,8.45,.59},LAB_CARDBOARD);
    box(scene,{-.25,7.95,.79},{.55,8.40,1.20},LAB_METAL,2,LAB_INSTRUMENT);
    box(scene,{.63,8.04,1.40},{1.04,8.42,1.91},LAB_CARDBOARD);
    box(scene,{-.23,8.00,2.02},{.52,8.42,2.38},LAB_CARDBOARD);
    box(scene,{-3.46,7.92,0},{-2.13,8.48,2.34},LAB_METAL,2,LAB_SHELVES);

    box(scene,{1.43,7.32,.87},{3.47,8.05,.945},LAB_METAL);
    for(double x:{1.49,3.31}) for(double y:{7.40,7.91})
        box(scene,{x,y,0},{x+.065,y+.065,.87},LAB_METAL);
    box(scene,{2.54,7.47,0},{3.19,8.00,.87},LAB_METAL,2,LAB_DRAWERS);
    box(scene,{1.60,7.56,.945},{2.19,7.98,1.37},LAB_METAL,2,LAB_INSTRUMENT);
    box(scene,{2.42,7.56,.945},{2.85,7.97,1.24},LAB_CARDBOARD);

    box(scene,{2.34,6.21,1.09},{3.30,6.28,2.08},LAB_METAL,2,LAB_WHITEBOARD);
    for(double x:{2.40,3.15}) {
        box(scene,{x,6.22,.12},{x+.045,6.265,1.10},LAB_METAL);
        box(scene,{x-.14,5.99,.04},{x+.19,6.48,.10},LAB_METAL);
    }
    box(scene,{-2.01,7.28,0},{-1.63,7.67,.65},LAB_METAL);
    box(scene,{-3.57,4.16,2.19},{-2.95,4.73,2.53},LAB_CARDBOARD);

    // The bright rear opening is a decal, not traversable geometry.
    decal(scene.walls[3],-1.72,-.62,0,2.51,LAB_OPENING);
    rectangle(scene.walls[3],-1.69,-.65,2.53,2.64,solid(69));
    decal(scene.walls[2],-2.8,-1.8,0,2.25,LAB_CABINET);
    for(const auto& light:lights) {
        rectangle(scene.walls[5],light.x-light.half_x-.04,light.x+light.half_x+.04,
                  light.y-light.half_y-.04,light.y+light.half_y+.04,solid(94));
        rectangle(scene.walls[5],light.x-light.half_x,light.x+light.half_x,
                  light.y-light.half_y,light.y+light.half_y,solid(251));
    }
    if(6*(scene.boxes.size()+1)>255)
        throw std::logic_error("lab_room exceeds the 8-bit surface-ID capacity");
    return scene;
}

double lab_room_radiance(const Hit& hit,Vec3 eye,double albedo,bool reflections) {
    const Vec3 point=hit.point;
    if(hit.object==0 && hit.face==5) return albedo>240?albedo:albedo*.63;
    if(hit.object==0 && hit.face==3 && point.x>-1.73 && point.x<-.61 && point.z<2.52) return albedo;
    double gain=.82;
    if(hit.object==0 && hit.face==4) gain=.95;
    else if(hit.face==5) gain=1.06;
    else if(hit.face==2) gain=.84;
    else if(hit.face==0 || hit.face==1) gain=.75;
    gain*=.87+.13*std::clamp(point.y/8.8,0.0,1.0);
    double value=albedo*gain;
    if(reflections && hit.object==0 && hit.face==4 && eye.z>.05) {
        const double qx=point.x+(point.x-eye.x)*ceiling_z/eye.z;
        const double qy=point.y+(point.y-eye.y)*ceiling_z/eye.z;
        for(const auto& light:lights) {
            const double dx=std::max(0.0,std::abs(qx-light.x)-light.half_x);
            const double dy=std::max(0.0,std::abs(qy-light.y)-light.half_y);
            const double exponent=(dx*dx+dy*dy)/(.34*.34);
            if(exponent<18) value+=76*std::exp(-.5*exponent);
        }
    }
    return value;
}

double lab_room_vignette(const Camera& camera,int x,int y,double strength) {
    if(strength<=0) return 1.0;
    const double dx=x-camera.cx,dy=y-camera.cy;
    const double r2=(dx*dx+dy*dy)/(camera.cx*camera.cx+camera.cy*camera.cy);
    return std::clamp(1.0-strength*r2,.05,1.0);
}
} // namespace vio
