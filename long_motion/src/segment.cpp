#include "long_motion/long_motion.hpp"
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <locale>
#include <cctype>

namespace long_motion {
std::string mode_name(Mode m) {
    switch(m) { case Mode::Rectilinear:return "rectilinear"; case Mode::Circular:return "circular"; case Mode::Scanner:return "scanner"; case Mode::XRHeadset:return "xr_headset"; case Mode::Controller:return "controller"; case Mode::Mower:return "mower"; }
    throw std::invalid_argument("Invalid motion mode");
}
Mode parse_mode(const std::string& s) {
    for(Mode m:{Mode::Rectilinear,Mode::Circular,Mode::Scanner,Mode::XRHeadset,Mode::Controller,Mode::Mower}) if(s==mode_name(m)) return m;
    throw std::invalid_argument("Unknown motion mode: "+s);
}
Config profile(Mode m) {
    Config c; c.mode=m;
    if(m==Mode::Rectilinear) { c.step_min=.5; c.step_max=3; c.look_pitch=0; c.look_roll=0; }
    if(m==Mode::Circular) { c.limits={1.2,3,18,2,6,40}; c.duration_min=2; c.duration_max=7; }
    if(m==Mode::Scanner) { c.limits={.65,2,12,1.8,6,40}; c.duration_min=1.4; c.duration_max=4; c.step_max=2.5; c.height_min=.9; c.height_max=1.6; }
    if(m==Mode::XRHeadset) { c.limits={1.6,4,30,4,18,150}; c.duration_min=.5; c.duration_max=2.8; c.height_min=1.25; c.height_max=1.9; c.start.z=1.6; c.look_yaw=rad(95); c.look_pitch=rad(40); c.look_roll=rad(12); c.step_max=2; }
    if(m==Mode::Controller) { c.limits={5,55,600,18,160,1800}; c.duration_min=.22; c.duration_max=.7; c.look_yaw=rad(150); c.look_pitch=rad(75); c.look_roll=rad(130); c.pause_probability=.1; c.pause_min=.08; c.pause_max=.4; c.height_min=.45; c.height_max=2.15; }
    if(m==Mode::Mower) { c.limits={.6,1.2,6,1.2,3,16}; c.start.z=c.mower_imu_height; c.duration_min=1; c.duration_max=3; c.pause_probability=0; c.look_pitch=0; c.look_roll=0; c.height_min=0; c.height_max=10; }
    return c;
}
static bool good_box(Aabb b) { return finite(b.lo)&&finite(b.hi)&&b.lo.x<=b.hi.x&&b.lo.y<=b.hi.y&&b.lo.z<=b.hi.z; }
void validate(const World& w,const Config& c) {
    (void)mode_name(c.mode);
    if(!good_box(w.room)||!std::isfinite(w.floor_z)||w.floor_z<w.room.lo.z||w.floor_z>=w.room.hi.z) throw std::invalid_argument("Invalid world room/floor");
    for(auto b:w.obstacles) if(!good_box(b)) throw std::invalid_argument("Invalid obstacle bounds");
    for(int i=0;i<3;++i) if(std::abs(w.room.lo[i])>1e9||std::abs(w.room.hi[i])>1e9) throw std::invalid_argument("Use a local world coordinate frame (room bounds must be within 1e9 m)");
    if(!finite(c.start)||!rotation_valid(c.initial_rotation)) throw std::invalid_argument("Invalid initial pose");
    if(!finite(c.circle_reference_offset_body)||!finite(c.optical_axis_body)||!finite(c.up_body)||!std::isfinite(norm(c.optical_axis_body))||!std::isfinite(norm(c.up_body))||norm(cross(c.optical_axis_body,c.up_body))<1e-7) throw std::invalid_argument("Invalid optical/up axes");
    auto pos=[](double x){return std::isfinite(x)&&x>0;};
    auto nonneg=[](double x){return std::isfinite(x)&&x>=0;};
    for(double x:{c.limits.speed,c.limits.acceleration,c.limits.jerk,c.limits.angular_speed,c.limits.angular_acceleration,c.limits.angular_jerk,c.duration_min,c.duration_max,c.step_min,c.step_max,c.circle_radius_min,c.circle_radius_max,c.circle_angle_min,c.circle_angle_max,c.circle_radius,c.gesture_radius,c.nav_resolution,c.numerical_guard,c.pause_min,c.pause_max,c.controller_relocate_speed,c.mower_radius,c.mower_imu_height,c.mower_top_above_imu,c.mower_lane_spacing}) if(!pos(x)) throw std::invalid_argument("All timing/limit/radius/resolution values must be finite and positive");
    for(double x:{c.warmup,c.body_radius,c.camera_radius,c.clearance,c.look_yaw,c.look_pitch,c.look_roll,c.pause_probability,c.rectilinear_distance,c.rectilinear_final_hold}) if(!nonneg(x)) throw std::invalid_argument("Invalid nonnegative motion option");
    if(!std::isfinite(c.height_min)||!std::isfinite(c.height_max)||c.height_min>c.height_max||c.duration_min>c.duration_max||c.step_min>c.step_max||c.pause_min>c.pause_max||c.pause_probability>1||c.circle_radius_min>c.circle_radius_max||c.circle_angle_min>c.circle_angle_max||c.circle_angle_max>4*pi||c.warmup>1e7) throw std::invalid_argument("Invalid option interval");
    if(c.pause_max>1e8||c.duration_max>1e8) throw std::invalid_argument("Primitive/hold durations must be below 1e8 seconds");
    if(c.nav_max_nodes<16||c.nav_max_nodes>2000000||c.max_attempts<1||c.max_attempts>100000||c.collision_depth<1||c.collision_depth>24||c.controller_relocate_every<0||c.history_capacity<0||c.history_capacity>4096) throw std::invalid_argument("Invalid planner budget");
    for(auto p:c.camera_offsets) if(!finite(p)) throw std::invalid_argument("Invalid camera lever arm");
    if(!finite(c.line_axis)||norm(c.line_axis)<1e-7||!finite(c.circle_center)) throw std::invalid_argument("Invalid line/circle geometry");
    if(c.mode==Mode::Mower) {
        if(norm(c.initial_rotation*unit(c.up_body)-Vec3{0,0,1})>1e-6) throw std::invalid_argument("Mower initial attitude must be upright; sloped terrain is not implemented");
        if(std::abs(c.start.z-(w.floor_z+c.mower_imu_height))>1e-6) throw std::invalid_argument("Mower start.z must equal floor_z + mower_imu_height (no silent snap)");
    }
    if(c.mode==Mode::Circular&&c.circle_fixed_center) {
        Vec3 d=c.start+c.initial_rotation*c.circle_reference_offset_body-c.circle_center;
        if(std::abs(d.z)>1e-10||std::abs(norm(d)-c.circle_radius)>1e-10) throw std::invalid_argument("Fixed-circle reference CAMERA center must start on the configured circle");
        if(norm(c.initial_rotation*unit(c.optical_axis_body)-unit(d))>1e-9) throw std::invalid_argument("Fixed-circle reference camera must initially look radially outward (no pose snap)");
    }
}
// Locale-independent, fully resolved parameters. Deliberately no rendering rate,
// total recording duration, thread count, noise level, texture or output path.
void write_config(std::ostream& o,const Config& c) {
    o.imbue(std::locale::classic()); o<<std::setprecision(17)<<std::boolalpha;
    auto v=[&](const char* k,Vec3 p){o<<k<<": ["<<p.x<<", "<<p.y<<", "<<p.z<<"]\n";};
    o<<"algorithm_version: "<<algorithm_version<<"\nmode: "<<mode_name(c.mode)<<"\nseed: "<<c.seed<<'\n';
    v("start",c.start); v("circle_reference_offset_body",c.circle_reference_offset_body); v("optical_axis_body",c.optical_axis_body); v("up_body",c.up_body);
    o<<"initial_rotation_row_major: ["; for(int i=0;i<9;++i) { if(i)o<<", "; o<<c.initial_rotation.a[i]; } o<<"]\n";
    o<<"camera_offsets: ["; for(size_t i=0;i<c.camera_offsets.size();++i) { if(i)o<<", "; Vec3 p=c.camera_offsets[i]; o<<'['<<p.x<<", "<<p.y<<", "<<p.z<<']'; } o<<"]\n";
#define VALUE(k) o<<#k<<": "<<c.k<<'\n'
    VALUE(body_radius); VALUE(camera_radius); VALUE(clearance); VALUE(warmup);
    VALUE(limits.speed); VALUE(limits.acceleration); VALUE(limits.jerk); VALUE(limits.angular_speed); VALUE(limits.angular_acceleration); VALUE(limits.angular_jerk);
    VALUE(duration_min); VALUE(duration_max); VALUE(step_min); VALUE(step_max); VALUE(height_min); VALUE(height_max);
    VALUE(pause_probability); VALUE(pause_min); VALUE(pause_max); VALUE(look_yaw); VALUE(look_pitch); VALUE(look_roll);
    VALUE(nav_resolution); VALUE(nav_max_nodes); VALUE(max_attempts); VALUE(collision_depth); VALUE(numerical_guard); VALUE(hold_when_blocked); VALUE(enable_safe_retreat); VALUE(history_capacity);
    VALUE(rectilinear_fixed_axis); v("line_axis",c.line_axis); VALUE(align_with_travel); VALUE(rectilinear_distance); VALUE(rectilinear_final_hold);
    VALUE(circle_radius_min); VALUE(circle_radius_max); VALUE(circle_angle_min); VALUE(circle_angle_max); VALUE(circle_fixed_center); v("circle_center",c.circle_center); VALUE(circle_radius);
    VALUE(gesture_radius); VALUE(controller_relocate_every); VALUE(controller_relocate_speed);
    VALUE(mower_radius); VALUE(mower_imu_height); VALUE(mower_top_above_imu); VALUE(mower_lane_spacing); VALUE(mower_alternate_axis);
#undef VALUE
}
std::string signature(const World& w,const Config& c) {
    // FNV-1a of a canonical textual snapshot. Fingerprint, not cryptographic hash.
    std::ostringstream o; write_config(o,c);
    auto vec=[&](Vec3 p){o<<p.x<<' '<<p.y<<' '<<p.z<<' ';};
    vec(w.room.lo);vec(w.room.hi);o<<w.floor_z<<'\n';
    auto obs=w.obstacles; std::sort(obs.begin(),obs.end(),[](const Aabb& a,const Aabb& b){ for(int k=0;k<6;++k) { double x=k<3?a.lo[k]:a.hi[k-3],y=k<3?b.lo[k]:b.hi[k-3]; if(x!=y)return x<y; } return false; });
    for(auto b:obs) {vec(b.lo);vec(b.hi);o<<'\n';}
    uint64_t h=14695981039346656037ULL; for(unsigned char x:o.str()){h^=x;h*=1099511628211ULL;}
    std::ostringstream r;r<<std::hex<<std::setfill('0')<<std::setw(16)<<h;return r.str();
}
State Segment::evaluate(double t) const {
    if(!std::isfinite(t)||duration_ns<=0) throw std::invalid_argument("Invalid segment time/duration");
    double T=double(duration_ns)*1e-9,u=std::clamp((t-start_time())/T,0.,1.);
    double u2=u*u,u3=u2*u,u4=u3*u;
    double s=std::clamp(u4*(35+u*(-84+u*(70-20*u))),0.,1.);
    // Exact endpoint values suppress polynomial cancellation in joins.
    if(u<=0) s=0;
    if(u>=1) s=1;
    double om=1-u,s1=140*u3*om*om*om/T;
    double s2=420*u2*om*om*(1-2*u)/(T*T);
    double s3=840*u*om*(1-5*u+5*u2)/(T*T*T);
    State out; Vec3 p1,p2,p3;
    if(kind==SegmentKind::Arc) {
        double theta=angle0+angle_delta*s,ct=std::cos(theta),st=std::sin(theta),d=angle_delta;
        out.p=center+Vec3{radius*ct,radius*st,0};
        p1={-radius*st*d,radius*ct*d,0}; p2={-radius*ct*d*d,-radius*st*d*d,0}; p3={radius*st*d*d*d,-radius*ct*d*d*d,0};
        out.R=rz(d*s)*R0;
        Vec3 axis=R0.transpose()*Vec3{0,0,1}; out.omega=axis*(d*s1);out.alpha=axis*(d*s2);
    } else {
        // Difference form preserves constant coordinates exactly (important for
        // an upright mower touching the supporting floor) and avoids cancellation.
        Vec3 d0=control[1]-control[0],d1=control[2]-control[1],d2=control[3]-control[2];
        Vec3 q=d1-d0,r=(d2-d1)-q;
        out.p=control[0]+(d0*3+(q*3+r*s)*s)*s;
        if(s==0) out.p=control[0];
        if(s==1) out.p=control[3];
        p1=d0*3+q*(6*s)+r*(3*s*s);p2=q*6+r*(6*s);p3=r*6;
        out.R=R0*exp_so3(rotation_vector*s);out.omega=rotation_vector*s1;out.alpha=rotation_vector*s2;
    }
    out.v=p1*s1;out.a=p2*(s1*s1)+p1*s2;out.jerk=p3*(s1*s1*s1)+p2*(3*s1*s2)+p1*s3;
    return out;
}
void write_segment_header(std::ostream& o) {
    o<<"kind,start_ns,duration_ns,event,label";
    for(int i=0;i<4;++i)for(char a:std::string("xyz"))o<<",b"<<i<<a;
    o<<",cx,cy,cz,radius,angle0,angle_delta";
    for(int i=0;i<9;++i)o<<",r"<<i;
    o<<",rotation_x,rotation_y,rotation_z,reference_cx,reference_cy,reference_cz,reference_radius\n";
}
void write_segment(std::ostream& o,const Segment& s) {
    o.imbue(std::locale::classic());o<<std::setprecision(17)<<int(s.kind)<<','<<s.start_ns<<','<<s.duration_ns<<','<<s.event<<','<<s.label;
    auto v=[&](Vec3 x){o<<','<<x.x<<','<<x.y<<','<<x.z;};
    for(auto p:s.control) v(p);
    v(s.center);o<<','<<s.radius<<','<<s.angle0<<','<<s.angle_delta;
    for(auto x:s.R0.a) o<<','<<x;
    v(s.rotation_vector);v(s.reference_center);o<<','<<s.reference_radius<<'\n';
}
Segment read_segment(const std::string& line) {
    std::vector<std::string> f;std::istringstream in(line);std::string x;while(std::getline(in,x,',')){while(!x.empty()&&std::isspace(static_cast<unsigned char>(x.back())))x.pop_back();f.push_back(x);}
    if(f.size()!=39) throw std::invalid_argument("Invalid primitive CSV column count");
    size_t k=0;
    auto d=[&](){size_t n=0;double v=std::stod(f.at(k++),&n);if(n!=f[k-1].size()||!std::isfinite(v))throw std::invalid_argument("Invalid primitive number");return v;};
    Segment s;int kind=std::stoi(f[k++]);if(kind<0||kind>2)throw std::invalid_argument("Invalid primitive kind");s.kind=SegmentKind(kind);
    s.start_ns=std::stoll(f[k++]);s.duration_ns=std::stoll(f[k++]);s.event=std::stoull(f[k++]);s.label=f[k++];
    auto v=[&](){double a=d(),b=d(),c=d();return Vec3{a,b,c};};
    for(auto& p:s.control) p=v();
    s.center=v();s.radius=d();s.angle0=d();s.angle_delta=d();
    for(auto& r:s.R0.a) r=d();
    s.rotation_vector=v();s.reference_center=v();s.reference_radius=d();
    if(s.start_ns<0||s.duration_ns<=0||s.start_ns>std::numeric_limits<int64_t>::max()-s.duration_ns||!rotation_valid(s.R0)||(s.kind==SegmentKind::Arc&&s.radius<=0))throw std::invalid_argument("Invalid primitive data");
    return s;
}
} // namespace long_motion
