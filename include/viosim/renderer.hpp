#pragma once
#include "camera.hpp"
#include "scene.hpp"
namespace vio {
struct GeometryImage { std::vector<float> range; Image8 surface_id; };
Image8 render(const Camera&,const Scene&,const TextureBank&,const Pose&,const Config&,uint64_t seed);
GeometryImage render_geometry(const Camera&,const Scene&,const Pose&);
} // namespace vio
