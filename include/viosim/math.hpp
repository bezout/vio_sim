#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace vio {
constexpr double pi = 3.1415926535897932384626433832795;
inline double radians(double degrees) { return degrees * pi / 180.0; }
struct Vec3 {
    double x=0, y=0, z=0;
    double operator[](int i) const { return i==0 ? x : (i==1 ? y : z); }
    double& operator[](int i) { return i==0 ? x : (i==1 ? y : z); }
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
    Vec3 operator/(double s) const { return *this*(1.0/s); }
    Vec3& operator+=(Vec3 b) { *this=*this+b; return *this; }
};
inline double dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a,Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline double norm(Vec3 v) { return std::sqrt(dot(v,v)); }
inline Vec3 normalized(Vec3 v) { return v/norm(v); }
struct Mat3 {
    std::array<double,9> a{1,0,0,0,1,0,0,0,1};
    double operator()(int r,int c) const { return a[3*r+c]; }
    double& operator()(int r,int c) { return a[3*r+c]; }
    Vec3 operator*(Vec3 v) const {
        return {a[0]*v.x+a[1]*v.y+a[2]*v.z,a[3]*v.x+a[4]*v.y+a[5]*v.z,a[6]*v.x+a[7]*v.y+a[8]*v.z};
    }
    Mat3 operator*(const Mat3& b) const {
        Mat3 r; r.a.fill(0);
        for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k) r(i,j)+=(*this)(i,k)*b(k,j);
        return r;
    }
    Mat3 transpose() const { Mat3 r; for(int i=0;i<3;++i) for(int j=0;j<3;++j) r(i,j)=(*this)(j,i); return r; }
};
inline Mat3 rx(double a) { const double c=std::cos(a),s=std::sin(a); return {{1,0,0,0,c,-s,0,s,c}}; }
inline Mat3 ry(double a) { const double c=std::cos(a),s=std::sin(a); return {{c,0,s,0,1,0,-s,0,c}}; }
inline Mat3 rz(double a) { const double c=std::cos(a),s=std::sin(a); return {{c,-s,0,s,c,0,0,0,1}}; }
inline Mat3 exp_so3(Vec3 w) {
    const double a=norm(w),a2=a*a;
    const double A=a<1e-7 ? 1-a2/6 : std::sin(a)/a;
    const double B=a<1e-7 ? 0.5-a2/24 : (1-std::cos(a))/a2;
    Mat3 K{{0,-w.z,w.y,w.z,0,-w.x,-w.y,w.x,0}},K2=K*K,r;
    for(int i=0;i<9;++i) r.a[i]+=A*K.a[i]+B*K2.a[i];
    return r;
}
// Hamilton quaternion, scalar first, rotating body vectors into world coordinates.
inline std::array<double,4> quaternion(const Mat3& R) {
    double w,x,y,z;
    const double tr=R(0,0)+R(1,1)+R(2,2);
    if(tr>0) {
        const double s=2*std::sqrt(1+tr); w=.25*s;
        x=(R(2,1)-R(1,2))/s; y=(R(0,2)-R(2,0))/s; z=(R(1,0)-R(0,1))/s;
    } else if(R(0,0)>R(1,1) && R(0,0)>R(2,2)) {
        const double s=2*std::sqrt(1+R(0,0)-R(1,1)-R(2,2)); x=.25*s;
        w=(R(2,1)-R(1,2))/s; y=(R(0,1)+R(1,0))/s; z=(R(0,2)+R(2,0))/s;
    } else if(R(1,1)>R(2,2)) {
        const double s=2*std::sqrt(1+R(1,1)-R(0,0)-R(2,2)); y=.25*s;
        w=(R(0,2)-R(2,0))/s; x=(R(0,1)+R(1,0))/s; z=(R(1,2)+R(2,1))/s;
    } else {
        const double s=2*std::sqrt(1+R(2,2)-R(0,0)-R(1,1)); z=.25*s;
        w=(R(1,0)-R(0,1))/s; x=(R(0,2)+R(2,0))/s; y=(R(1,2)+R(2,1))/s;
    }
    const double n=std::sqrt(w*w+x*x+y*y+z*z),sign=w<0 ? -1.0:1.0;
    return {sign*w/n,sign*x/n,sign*y/n,sign*z/n};
}
// Scalar with exact first and second time derivatives. No finite differencing.
struct Jet {
    double v=0,d=0,dd=0;
    Jet()=default;
    Jet(double v_,double d_=0,double dd_=0):v(v_),d(d_),dd(dd_) {}
};
inline Jet operator+(Jet a,Jet b) { return {a.v+b.v,a.d+b.d,a.dd+b.dd}; }
inline Jet operator-(Jet a,Jet b) { return {a.v-b.v,a.d-b.d,a.dd-b.dd}; }
inline Jet operator-(Jet a) { return {-a.v,-a.d,-a.dd}; }
inline Jet operator*(Jet a,Jet b) { return {a.v*b.v,a.d*b.v+a.v*b.d,a.dd*b.v+2*a.d*b.d+a.v*b.dd}; }
inline Jet sin(Jet a) { return {std::sin(a.v),std::cos(a.v)*a.d,std::cos(a.v)*a.dd-std::sin(a.v)*a.d*a.d}; }
inline Jet cos(Jet a) { return {std::cos(a.v),-std::sin(a.v)*a.d,-std::sin(a.v)*a.dd-std::cos(a.v)*a.d*a.d}; }
// Per-stream PRNG and explicitly specified Box-Muller transform. No shared RNG.
inline uint64_t mix_seed(uint64_t z) {
    z=(z^(z>>30))*0xbf58476d1ce4e5b9ULL; z=(z^(z>>27))*0x94d049bb133111ebULL; return z^(z>>31);
}
class Noise {
    uint64_t state_; bool cached_=false; double spare_=0;
public:
    explicit Noise(uint64_t seed):state_(seed) {}
    double uniform() { state_+=0x9e3779b97f4a7c15ULL; return (double(mix_seed(state_)>>11)+.5)*0x1.0p-53; }
    double normal() {
        if(cached_) { cached_=false; return spare_; }
        const double a=std::sqrt(-2*std::log(uniform())),b=2*pi*uniform();
        spare_=a*std::sin(b); cached_=true; return a*std::cos(b);
    }
    Vec3 normal3() { return {normal(),normal(),normal()}; }
};
} // namespace vio
