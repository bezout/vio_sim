#include "viosim/scene.hpp"
#include "viosim/lab_room.hpp"
#include <stdexcept>
namespace vio {
static Material tex(int id,double us=1,double vs=1,double uo=0,double vo=0,bool repeat=true) { return {id,0,us,vs,uo,vo,repeat}; }
static Material gray(double value) { return {-1,value,1,1,0,0,true}; }
static void decal(Surface& s,double u0,double u1,double v0,double v1,int texture) {
    s.decals.push_back({u0,u1,v0,v1,tex(texture,1/(u1-u0),1/(v1-v0),-u0/(u1-u0),-v0/(v1-v0),false)});
}
static void solid(Surface& s,double u0,double u1,double v0,double v1,double value) { s.decals.push_back({u0,u1,v0,v1,gray(value)}); }
Scene make_scene(const std::string& name) {
    if(name=="lab_room") return make_lab_room();
    Scene s; s.name=name;
    if(name=="office" || name=="office_tag" || name=="office_tag_near") {
        s.room={{-3,0,0},{3,7,3}};
        // Keep the scene and motion identical; translate the two device paths
        // towards the fixed tag for a controlled range comparison.
        s.anchor=name=="office_tag_near" ? Vec3{-.25,3.0,1.25}
                                         : Vec3{0,1.6,1.25};
        s.boxes={{{{-1.9,2.1,0},{-1.15,2.9,1.3}},CRATE_A},{{{1,3.35,0},{1.8,4.15,1.55}},CRATE_B},{{{-.35,4.95,0},{.35,5.55,1.95}},CRATE_C}};
        s.walls[0].base=s.walls[1].base=tex(WALL,1/3.2,1/1.6);
        s.walls[2].base=s.walls[3].base=tex(WALL,1/3.4,1/1.6,10/3.4);
        decal(s.walls[0],1.1,1.95,.85,1.75,POSTER_LIGHT); decal(s.walls[0],4,4.75,1,1.7,POSTER_DARK);
        decal(s.walls[1],2,2.95,0,2.05,DOOR); decal(s.walls[1],4.35,5.2,1.05,1.75,POSTER_LIGHT);
        solid(s.walls[2],-.8,.8,1.2,2.35,215); solid(s.walls[2],-.03,.03,1.2,2.35,100); solid(s.walls[2],-.8,.8,1.745,1.805,100);
        solid(s.walls[3],-1.1,1.1,1,1.8,35); solid(s.walls[3],-1.04,1.04,1.06,1.74,88);
    } else if(name=="multilevel" || name=="multilevel_tables" || name=="flat_tables") {
        const bool flat=name=="flat_tables";
        s.room={{-3,0,0},{3,16,flat?3.2:6.4}}; s.anchor={0,1.6,1.6};
        if(!flat) {
            // Twenty 15 cm risers, 30 cm treads, then a 3 m upper landing.
            for(int i=0;i<20;++i) {
                Box step{{{-1.5,4+.3*i,0},{1.5,4+.3*(i+1),.15*(i+1)}},FLOOR};
                step.support_kind=2; s.boxes.push_back(step);
            }
            Box landing{{{-3,10,0},{3,16,3}},FLOOR};
            landing.support_kind=1; s.boxes.push_back(landing);
        }
        if(name!="multilevel") {
            for(double y:{2.2,3.3,11.3,12.6,14.2}) for(double x:{-2.0,2.0}) {
                const double level=!flat && y>10?3.0:0.0;
                Box table{{{x-.6,y-.45,level+.65},{x+.6,y+.45,level+.8}},CRATE_A};
                s.boxes.push_back(table);
            }
        }
        s.walls[0].base=s.walls[1].base=tex(CORRIDOR_WALL,1/2.4,1/1.5);
        s.walls[2].base=s.walls[3].base=tex(WALL,1/2.2,1/1.5);
    } else if(name=="corridor") {
        s.room={{-2.1,0,0},{2.1,16,3}}; s.anchor={0,4,1.25};
        s.boxes={{{{-1.7,2.2,0},{-1.2,2.7,1.5}},CRATE_A},{{{1.2,5.3,0},{1.7,5.8,1.5}},CRATE_B},{{{-1.7,8.6,0},{-1.2,9.1,1.5}},CRATE_C},{{{1.2,11.8,0},{1.7,12.3,1.5}},CRATE_A}};
        s.walls[0].base=s.walls[1].base=tex(CORRIDOR_WALL,1/2.4,1/1.5);
        s.walls[2].base=s.walls[3].base=tex(CORRIDOR_WALL,1/2.2,1/1.5,10/2.2);
    } else if(name=="warehouse") {
        s.room={{-5,0,0},{5,12,4.2}}; s.anchor={0,6.5,1.35};
        s.boxes={{{{-3.8,1.6,0},{-2.7,3.4,2.2}},CRATE_A},{{{2.7,2,0},{3.8,4,2.4}},CRATE_B},{{{-1,5,0},{1,5.8,2.8}},CRATE_C},{{{-4,7.8,0},{-2.3,9.8,2.2}},CRATE_B},{{{2.2,8.2,0},{4.2,10.2,2.2}},CRATE_A}};
        s.walls[0].base=s.walls[1].base=tex(WAREHOUSE_WALL,1/4.5,.5);
        s.walls[2].base=s.walls[3].base=tex(WAREHOUSE_WALL,.25,.5,2.5);
    } else throw std::invalid_argument("Unknown scene: "+name);
    s.walls[4].base=tex(FLOOR,1/3.2,1/3.2,20/3.2);
    s.walls[5].base=tex(CEILING,1/3.5,1/3.5,20/3.5);
    if(name=="office" || name=="office_tag" || name=="office_tag_near") { solid(s.walls[4],-.9,.9,1.8,3.5,45); solid(s.walls[4],-.82,.82,1.9,3.4,90); }
    if(name=="office_tag" || name=="office_tag_near") {
        // Rear wall y=7: 1 m square including the white margin, hence an
        // AprilTag black-border side of 0.8 m. Center: (-1.7, 7, 2) m.
        decal(s.walls[3],-2.2,-1.2,1.5,2.5,APRILTAG_36H11_0);
        // Texture rows run down; world Z runs up. Avoid mirroring the code.
        auto& material=s.walls[3].decals.back().material;
        material.vs=-1;
        material.vo=2.5;
    }
    return s;
}
bool intersect_box(const AABB& b,Vec3 o,Vec3 d,double& near,double& far,int& nf,int& ff) {
    near=-std::numeric_limits<double>::infinity(); far=std::numeric_limits<double>::infinity(); nf=ff=-1;
    for(int k=0;k<3;++k) {
        if(std::abs(d[k])<1e-14) { if(o[k]<b.lo[k] || o[k]>b.hi[k]) return false; continue; }
        double t0=(b.lo[k]-o[k])/d[k],t1=(b.hi[k]-o[k])/d[k]; int f0=2*k,f1=2*k+1;
        if(t0>t1) { std::swap(t0,t1); std::swap(f0,f1); }
        if(t0>near) { near=t0; nf=f0; } if(t1<far) { far=t1; ff=f1; }
        if(near>far) return false;
    }
    return far>1e-7;
}
Hit Scene::trace(Vec3 origin,Vec3 ray) const {
    Hit h; double near,far; int nf,ff;
    if(intersect_box(room,origin,ray,near,far,nf,ff)) { h.distance=far; h.object=0; h.face=ff; }
    for(size_t i=0;i<boxes.size();++i) if(intersect_box(boxes[i].bounds,origin,ray,near,far,nf,ff)) {
        const bool outside=near>1e-7; const double t=outside?near:far;
        if(t<h.distance) { h.distance=t; h.object=int(i)+1; h.face=outside?nf:ff; }
    }
    if(h.object>=0) h.point=origin+ray*h.distance;
    return h;
}
bool Scene::safe(Vec3 p,double margin) const {
    for(int k=0;k<3;++k) if(p[k]<=room.lo[k]+margin || p[k]>=room.hi[k]-margin) return false;
    for(const auto& b:boxes) {
        bool inside=true; for(int k=0;k<3;++k) if(p[k]<b.bounds.lo[k]-margin || p[k]>b.bounds.hi[k]+margin) inside=false;
        if(inside) return false;
    }
    return true;
}
double Scene::support_height(Vec3 point,int& kind) const {
    double height=room.lo.z; kind=1;
    for(const auto& box:boxes) {
        if(!box.support_kind || box.bounds.hi.z>point.z) continue;
        if(point.x>=box.bounds.lo.x && point.x<=box.bounds.hi.x &&
           point.y>=box.bounds.lo.y && point.y<=box.bounds.hi.y && box.bounds.hi.z>height) {
            height=box.bounds.hi.z; kind=box.support_kind;
        }
    }
    return height;
}
std::array<int,2> Scene::uv_axes(int face) { const int k=face/2; return k==0?std::array<int,2>{1,2}:(k==1?std::array<int,2>{0,2}:std::array<int,2>{0,1}); }
Material Scene::material(const Hit& h) const {
    const auto axes=uv_axes(h.face); const double u=h.point[axes[0]],v=h.point[axes[1]];
    if(h.object==0) {
        const auto& s=walls[size_t(h.face)]; Material m=s.base;
        for(const auto& d:s.decals) if(u>=d.u0 && u<=d.u1 && v>=d.v0 && v<=d.v1) m=d.material;
        return m;
    }
    const auto& b=boxes[size_t(h.object-1)]; const double us=1/(b.bounds.hi[axes[0]]-b.bounds.lo[axes[0]]),vs=1/(b.bounds.hi[axes[1]]-b.bounds.lo[axes[1]]);
    const int texture=b.face_textures[size_t(h.face)]>=0?b.face_textures[size_t(h.face)]:b.texture;
    // Walkable tops use isotropic world-space tiles. Stretching a full texture
    // across a 3 m by 30 cm tread squashes corners into nearly collinear rows.
    if(h.face==5 && b.support_kind)
        return tex(texture,1/1.2,1/1.2,0,0,true);
    return tex(texture,us,vs,-b.bounds.lo[axes[0]]*us,-b.bounds.lo[axes[1]]*vs,false);
}
} // namespace vio
