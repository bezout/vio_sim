#pragma once
#include "math.hpp"
#include "image.hpp"
#include <array>
#include <limits>
#include <string>
#include <vector>
namespace vio {
struct AABB { Vec3 lo,hi; };
struct Material { int texture=-1; double value=160,us=1,vs=1,uo=0,vo=0; bool repeat=true; };
struct Decal { double u0,u1,v0,v1; Material material; };
struct Surface { Material base; std::vector<Decal> decals; };
struct Box {
    AABB bounds; int texture;
    std::array<int,6> face_textures{{-1,-1,-1,-1,-1,-1}};
};
struct Hit { double distance=std::numeric_limits<double>::infinity(); int object=-1,face=-1; Vec3 point; };
struct Scene {
    std::string name; AABB room; Vec3 anchor;
    std::array<Surface,6> walls;
    std::vector<Box> boxes;
    bool safe(Vec3 point,double margin) const;
    Hit trace(Vec3 origin,Vec3 ray) const;
    Material material(const Hit&) const;
    static std::array<int,2> uv_axes(int face);
};
Scene make_scene(const std::string&);
bool intersect_box(const AABB&,Vec3 origin,Vec3 ray,double& near,double& far,int& near_face,int& far_face);
} // namespace vio
