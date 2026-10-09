#pragma once
#include "math.hpp"
#include "config.hpp"
#include <vector>
namespace vio {
struct Float3 {
    float x=0,y=0,z=0;
    Float3()=default;
    explicit Float3(Vec3 v):x(float(v.x)),y(float(v.y)),z(float(v.z)) {}
    operator Vec3() const { return {x,y,z}; }
};
struct RaySample { Float3 d,dx,dy; };
struct Camera {
    int width,height,samples;
    double fx,fy,cx,cy,xi,alpha,beta;
    bool seucm;
    std::vector<RaySample> rays;
    explicit Camera(const Config&,bool precompute=true);
    Vec3 unproject(double u,double v) const;
    std::array<double,2> project(Vec3 p) const;
};
struct Pose { Vec3 p; Mat3 R; };
struct Rig {
    std::vector<Pose> BC;
    explicit Rig(const Config&);
    size_t size() const { return BC.size(); }
    Pose camera_pose(Vec3 p,const Mat3& R,size_t camera) const { return {p+R*BC.at(camera).p,R*BC.at(camera).R}; }
};
} // namespace vio
