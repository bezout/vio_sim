#include "viosim/camera.hpp"
#include <stdexcept>
namespace vio {
Camera::Camera(const Config& c,bool precompute):Camera(c,CameraExtrinsic{},precompute) {}
Camera::Camera(const Config& c,const CameraExtrinsic& calibration,bool precompute):width(c.width),height(c.height),samples(c.samples),cx((width-1)*.5),cy((height-1)*.5),eu(cx),ev(cy),xi(c.xi),alpha(c.alpha),beta(c.beta),seucm(c.camera_model=="seucm") {
    const double theta=radians(c.fov_deg*.5);
    const double st=std::sin(theta),ct=std::cos(theta);
    fx=fy=cx*(seucm ? alpha*std::sqrt(beta*st*st+ct*ct)+(1-alpha)*ct : ct+xi)/st;
    if(calibration.intrinsics_set) {
        const auto& k=calibration.intrinsics;
        fx=k[0];fy=k[1];cx=k[2];cy=k[3];eu=k[4];ev=k[5];alpha=k[6];beta=k[7];
    }
    prepare_rays(precompute);
}
void Camera::prepare_rays(bool precompute) {
    // Reject configurations whose image corners lie outside the inverse domain.
    for(double u:{-.5,double(width)-.5}) for(double v:{-.5,double(height)-.5}) {
        const auto ray=unproject(u,v);
        if(!std::isfinite(ray.z)) throw std::invalid_argument("Camera image exceeds projection domain; reduce FOV or alpha/beta");
    }
    // FOV is measured between the outermost pixel centers on the horizontal midline.
    if(!precompute) return;
    const int sw=width*samples,sh=height*samples; rays.resize(size_t(sw)*sh);
    const double step=1.0/samples;
    for(int y=0;y<sh;++y) for(int x=0;x<sw;++x) {
        const double u=(x+.5)*step-.5,v=(y+.5)*step-.5;
        const Vec3 d=unproject(u,v);
        const Vec3 dx=(unproject(u+.5*step,v)-unproject(u-.5*step,v));
        const Vec3 dy=(unproject(u,v+.5*step)-unproject(u,v-.5*step));
        rays[size_t(y)*sw+x]={Float3(d),Float3(dx),Float3(dy)};
    }
}
Vec3 Camera::unproject(double u,double v) const {
    const double x=(u-(seucm?eu:cx))/fx,y=(v-(seucm?ev:cy))/fy,r2=x*x+y*y;
    if(seucm) {
        const double z=(1-beta*alpha*alpha*r2)/(alpha*std::sqrt(1-(2*alpha-1)*beta*r2)+1-alpha);
        return normalized({x+(eu-cx)/fx*z,y+(ev-cy)/fy*z,z});
    }
    const double l=(xi+std::sqrt(1+(1-xi*xi)*r2))/(1+r2);
    return normalized({l*x,l*y,l-xi});
}
std::array<double,2> Camera::project(Vec3 p) const {
    if(seucm) {
        p.x-=(eu-cx)/fx*p.z;
        p.y-=(ev-cy)/fy*p.z;
        const double d=std::sqrt(beta*(p.x*p.x+p.y*p.y)+p.z*p.z);
        const double w=alpha>.5 ? (1-alpha)/alpha : alpha/(1-alpha);
        if(p.z<=-w*d) return {NAN,NAN};
    }
    const double denominator=seucm ? (1-alpha)*p.z+alpha*std::sqrt(beta*(p.x*p.x+p.y*p.y)+p.z*p.z) : p.z+xi*norm(p);
    if(denominator<=0) return {NAN,NAN};
    return {fx*p.x/denominator+(seucm?eu:cx),fy*p.y/denominator+(seucm?ev:cy)};
}
Rig::Rig(const Config& c) {
    if(c.camera_count>0) {
        BC.reserve(size_t(c.camera_count));
        for(const auto& camera:c.camera_extrinsics) {
            Mat3 R; R.a=camera.rotation;
            BC.push_back({{camera.translation[0],camera.translation[1],camera.translation[2]},R});
        }
        return;
    }
    const Mat3 R=imu_basis_rotation(c).transpose()*Mat3{{1,0,0,0,0,1,0,-1,0}};
    BC={{{-c.baseline*.5,0,0},R},{{c.baseline*.5,0,0},R*ry(radians(c.right_yaw_deg))*rz(radians(c.right_roll_deg))}};
}
} // namespace vio
