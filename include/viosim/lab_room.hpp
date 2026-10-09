#pragma once
#include "camera.hpp"
#include "scene.hpp"
namespace vio {
Scene make_lab_room();
double lab_room_radiance(const Hit&,Vec3 camera_origin,double albedo,bool reflections);
double lab_room_vignette(const Camera&,int x,int y,double strength);
} // namespace vio
