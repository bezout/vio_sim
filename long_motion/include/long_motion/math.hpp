#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace long_motion {
constexpr double pi = 3.141592653589793238462643383279502884;
inline double rad(double deg) { return deg * pi / 180.; }
struct Vec3 {
    double x=0, y=0, z=0;
    double operator[](int k) const { return k==0?x:(k==1?y:z); }
    double& operator[](int k) { return k==0?x:(k==1?y:z); }
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
    Vec3 operator/(double s) const { return *this*(1./s); }
    Vec3& operator+=(Vec3 b) { *this=*this+b; return *this; }
};
inline Vec3 operator*(double s,Vec3 a) { return a*s; }
inline double dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a,Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline double norm(Vec3 a) { return std::sqrt(dot(a,a)); }
inline bool finite(Vec3 a) { return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z); }
inline Vec3 unit(Vec3 a) { double n=norm(a); if(!(n>1e-12)) throw std::invalid_argument("Cannot normalize a zero vector"); return a/n; }
inline Vec3 vmin(Vec3 a,Vec3 b) { return {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)}; }
inline Vec3 vmax(Vec3 a,Vec3 b) { return {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)}; }
struct Mat3 {
    std::array<double,9> a{1,0,0,0,1,0,0,0,1};
    double operator()(int i,int j) const { return a[3*i+j]; }
    double& operator()(int i,int j) { return a[3*i+j]; }
    Vec3 operator*(Vec3 p) const { return {a[0]*p.x+a[1]*p.y+a[2]*p.z,a[3]*p.x+a[4]*p.y+a[5]*p.z,a[6]*p.x+a[7]*p.y+a[8]*p.z}; }
    Mat3 operator*(const Mat3& b) const { Mat3 c; c.a.fill(0); for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k) c(i,j)+=(*this)(i,k)*b(k,j); return c; }
    Mat3 transpose() const { Mat3 b; for(int i=0;i<3;++i) for(int j=0;j<3;++j) b(i,j)=(*this)(j,i); return b; }
};
inline Mat3 columns(Vec3 a,Vec3 b,Vec3 c) { return {{a.x,b.x,c.x,a.y,b.y,c.y,a.z,b.z,c.z}}; }
inline Mat3 rz(double t) { double c=std::cos(t),s=std::sin(t); return {{c,-s,0,s,c,0,0,0,1}}; }
inline Mat3 exp_so3(Vec3 w) {
    double a=norm(w),a2=a*a;
    double A=a<1e-6?1-a2/6+a2*a2/120:std::sin(a)/a;
    double B=a<1e-6?.5-a2/24+a2*a2/720:(1-std::cos(a))/a2;
    Mat3 K{{0,-w.z,w.y,w.z,0,-w.x,-w.y,w.x,0}},K2=K*K,R;
    for(int i=0;i<9;++i) R.a[i]+=A*K.a[i]+B*K2.a[i];
    return R;
}
inline std::array<double,4> quaternion(const Mat3& R) {
    double w,x,y,z,tr=R(0,0)+R(1,1)+R(2,2);
    if(tr>0) { double s=2*std::sqrt(std::max(0.,1+tr)); w=s/4; x=(R(2,1)-R(1,2))/s; y=(R(0,2)-R(2,0))/s; z=(R(1,0)-R(0,1))/s; }
    else if(R(0,0)>R(1,1)&&R(0,0)>R(2,2)) { double s=2*std::sqrt(std::max(0.,1+R(0,0)-R(1,1)-R(2,2))); x=s/4; w=(R(2,1)-R(1,2))/s; y=(R(0,1)+R(1,0))/s; z=(R(0,2)+R(2,0))/s; }
    else if(R(1,1)>R(2,2)) { double s=2*std::sqrt(std::max(0.,1+R(1,1)-R(0,0)-R(2,2))); y=s/4; w=(R(0,2)-R(2,0))/s; x=(R(0,1)+R(1,0))/s; z=(R(1,2)+R(2,1))/s; }
    else { double s=2*std::sqrt(std::max(0.,1+R(2,2)-R(0,0)-R(1,1))); z=s/4; w=(R(1,0)-R(0,1))/s; x=(R(0,2)+R(2,0))/s; y=(R(1,2)+R(2,1))/s; }
    double n=std::sqrt(w*w+x*x+y*y+z*z),sgn=w<0?-1.:1.;
    return {sgn*w/n,sgn*x/n,sgn*y/n,sgn*z/n};
}
inline Vec3 log_so3(const Mat3& R) {
    auto q=quaternion(R); Vec3 v{q[1],q[2],q[3]}; double n=norm(v);
    return n<1e-10?v*2:v*(2*std::atan2(n,q[0])/n);
}
inline bool rotation_valid(const Mat3& R) {
    for(double x:R.a) if(!std::isfinite(x)) return false;
    Mat3 E=R.transpose()*R;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) if(std::abs(E(i,j)-(i==j?1.:0.))>1e-7) return false;
    return dot(cross(R*Vec3{1,0,0},R*Vec3{0,1,0}),R*Vec3{0,0,1})>.999999;
}
// Map the configured reference optical direction to forward_world. Roll is zero
// with respect to world +Z, unless a nearly vertical look needs an alternate up.
inline Mat3 look_rotation(Vec3 forward_world,Vec3 forward_body,Vec3 up_body) {
    Vec3 fw=unit(forward_world),fb=unit(forward_body),ub=unit(up_body-fb*dot(up_body,fb));
    Vec3 uw={0,0,1}; if(std::abs(dot(fw,uw))>.98) uw={0,1,0};
    Vec3 rw=unit(cross(fw,uw)); uw=cross(rw,fw);
    Vec3 rb=cross(fb,ub);
    return columns(rw,fw,uw)*columns(rb,fb,ub).transpose();
}
inline uint64_t mix64(uint64_t z) { z=(z^(z>>30))*0xbf58476d1ce4e5b9ULL; z=(z^(z>>27))*0x94d049bb133111ebULL; return z^(z>>31); }
// Counter-keyed integer RNG. Explicit top-53-bit mapping, no std distributions.
inline double random01(uint64_t seed,uint64_t event,uint64_t stream,uint64_t draw) {
    uint64_t h=mix64(seed^0xa0761d6478bd642fULL);
    h=mix64(h^mix64(event+0xe7037ed1a0b428dbULL));
    h=mix64(h^mix64(stream+0x8ebc6af09c88c6e3ULL));
    h=mix64(h^mix64(draw+0x589965cc75374cc3ULL));
    return double(h>>11)*0x1.0p-53;
}
} // namespace long_motion
