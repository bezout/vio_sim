#include "long_motion/long_motion.hpp"
#include "long_motion/options.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace long_motion;
namespace {
int checks=0;
void check(bool v,const std::string& s) {++checks;if(!v)throw std::runtime_error(s);}
void near(double a,double b,double eps,const std::string& s){check(std::abs(a-b)<=eps,s+": "+std::to_string(a)+" vs "+std::to_string(b));}
void near(Vec3 a,Vec3 b,double eps,const std::string& s){check(norm(a-b)<=eps,s+": vector error "+std::to_string(norm(a-b)));}
void near(const Mat3& a,const Mat3& b,double eps,const std::string& s){for(int k=0;k<9;++k)near(a.a[k],b.a[k],eps,s);}
void same(const State&a,const State&b,const std::string&s){near(a.p,b.p,0,s);near(a.v,b.v,0,s);near(a.a,b.a,0,s);near(a.omega,b.omega,0,s);near(a.R,b.R,0,s);}
World room(){return {{{-4,0,0},{4,10,3}},{{{-2,5,0},{-1,6,2}},{{1,3,0},{2,4,2}}},0};}
Config cfg(Mode m){Config c=profile(m);c.start={0,2,c.start.z};return c;}
std::vector<Mode> modes(){return {Mode::Rectilinear,Mode::Circular,Mode::Scanner,Mode::XRHeadset,Mode::Controller,Mode::Mower};}
template<class F> void must_throw(F f,const std::string&s){bool ok=false;try{f();}catch(const std::exception&){ok=true;}check(ok,s);}
void test_inputs(){
    auto w=room();auto c=cfg(Mode::Scanner);
    c.start={1.5,3.5,1};must_throw([&]{Generator g(w,c);},"inside box rejected");
    c=cfg(Mode::Scanner);c.limits.acceleration=0;must_throw([&]{Generator g(w,c);},"zero bound rejected");
    c=cfg(Mode::Mower);c.start.z+=.1;must_throw([&]{Generator g(w,c);},"mower no silent height snap");
    c=cfg(Mode::Scanner);c.seed=0;Generator g(w,c);must_throw([&]{g.at(-1);},"negative time");
    c.optical_axis_body=c.up_body;must_throw([&]{Generator h(w,c);},"collinear camera axes");
    std::cout<<"PASS input validation and no silent relocation\n";
}
void test_options(){
    for(auto m:modes()) {
        Config c=cfg(m);c.seed=UINT64_C(18446744073709551615);
        std::ostringstream out;write_config(out,c);std::istringstream in(out.str());
        auto options=read_flat_options(in);Config copy=profile(m);
        for(auto option:options)set_option(copy,option.first,option.second);
        check(signature(room(),copy)==signature(room(),c),"resolved configuration round trip");
        check(copy.seed==c.seed,"uint64 seed precision");
    }
    Config c=cfg(Mode::Scanner);
    must_throw([&]{set_option(c,"seed","-1");},"negative seed rejected");
    must_throw([&]{set_option(c,"speeed","1");},"unknown option rejected");
    must_throw([&]{set_option(c,"limits.speed","1x");},"trailing numeric data rejected");
    std::istringstream duplicates("mode: scanner\nmode: mower\n");
    must_throw([&]{read_flat_options(duplicates);},"duplicate keys rejected");
    std::cout<<"PASS resolved options, full-width uint64 seed, invalid values\n";
}
void test_collision(){
    World w{{{-2,-2,-2},{2,2,2}},{{{-.001,-.5,-.5},{.001,.5,.5}}},-2};Config c=cfg(Mode::Scanner);c.start={-1,0,0};c.body_radius=.02;c.camera_offsets={{0,0,0}};c.camera_radius=.01;c.clearance=.01;
    CollisionWorld cw(w,c);
    check(cw.point_free({-1,0,0})&&cw.point_free({1,0,0}),"thin wall endpoint free");
    check(!cw.line_free({-1,0,0},{1,0,0}),"2 mm wall cannot be skipped between samples");
    check(!cw.bezier_free({Vec3{-1,0,0},Vec3{-.5,0,0},Vec3{.5,0,0},Vec3{1,0,0}}),"Bezier crossing thin wall");
    check(cw.bezier_free({Vec3{-1,1,0},Vec3{-.5,1,0},Vec3{.5,1,0},Vec3{1,1,0}}),"Bezier above obstacle");
    check(!cw.arc_free({0,0,0},.3,0,2*pi),"circle crossing slab");
    check(cw.arc_free({-1,0,0},.3,-pi,2*pi),"free circle");
    World enclosure{{{-2,-2,-2},{2,2,2}},{{{.8,-.1,-.1},{.9,.1,.1}}},-2};
    c.start={0,0,0};c.camera_offsets={{.85,0,0}};must_throw([&]{CollisionWorld x(enclosure,c);},"camera lever arm enclosed even when IMU is clear");
    std::cout<<"PASS continuous collision certificates, thin wall, camera lever arms\n";
}
void test_frequency_prefix_seed(){
    for(auto m:modes()){
        Config c=cfg(m);c.seed=UINT64_C(18446744073709551557);
        Generator a(room(),c),b(room(),c),dense(room(),c),rewind(room(),c);
        std::vector<State> reference;
        for(int i=0;i<=1500;++i)reference.push_back(a.at(double(i)/50));
        for(int i=0;i<=18000;++i){auto s=dense.at(double(i)/500);if(i%10==0&&i/10<=1500)same(s,reference[i/10],"camera/IMU shared-time bit identity");}
        for(int i=0;i<=3000;++i){auto s=b.at(double(i)/50);if(i<=1500)same(s,reference[i],"30->60 second prefix");}
        auto token=b.trajectory_signature();check(token==a.trajectory_signature(),"identity stable across duration/sample calls");
        auto at30=rewind.at(30);(void)at30;for(int i:{2,1000,4,1400,0,1499})same(rewind.at(double(i)/50),reference[i],"out-of-order replay");
        c.seed++;Generator other(room(),c);bool difference=false;for(int i=250;i<1000;i+=37)if(norm(other.at(double(i)/50).p-reference[i].p)>1e-8)difference=true;check(difference,"different seed changes motion");
        World shuffled=room();std::reverse(shuffled.obstacles.begin(),shuffled.obstacles.end());c.seed--;Generator order(shuffled,c);
        check(order.trajectory_signature()==token,"geometry order-independent fingerprint");for(int i=0;i<1500;i+=79)same(order.at(double(i)/50),reference[i],"geometry insertion order");
        std::cout<<"PASS prefix/rates/rewind/order/seed "<<mode_name(m)<<"\n";
    }
}
void check_derivatives(const Segment&s,double t){
    double h=1e-5;auto a=s.evaluate(t-h),b=s.evaluate(t),c=s.evaluate(t+h);
    near((c.p-a.p)/(2*h),b.v,2e-5,"position analytic derivative");
    near((c.v-a.v)/(2*h),b.a,5e-5,"velocity analytic derivative");
    near((c.a-a.a)/(2*h),b.jerk,3e-3,"acceleration analytic derivative");
    Mat3 d;for(int i=0;i<9;++i)d.a[i]=(c.R.a[i]-a.R.a[i])/(2*h);
    Mat3 k=b.R.transpose()*d;Vec3 w{(k(2,1)-k(1,2))/2,(k(0,2)-k(2,0))/2,(k(1,0)-k(0,1))/2};
    near(w,b.omega,2e-5,"gyro from R^T dR");
    near((c.omega-a.omega)/(2*h),b.alpha,3e-4,"angular acceleration");
    check(rotation_valid(b.R),"rotation orthonormal");
}
void test_continuity_limits_and_profiles(){
    for(auto m:modes()){
        auto c=cfg(m);c.pause_probability=0;Generator g(room(),c);double t=0;int n=0;double maxv=0,maxw=0;
        while(t<180){g.at(t);Segment s=g.current_segment();check(g.collision_world().segment_free(s),"complete segment certificate");
            auto q0=s.initial(),q1=s.final();near(q0.v,{},1e-10,"start v zero");near(q1.v,{},1e-10,"end v zero");near(q0.a,{},1e-9,"start a zero");near(q1.a,{},1e-9,"end a zero");near(q0.jerk,{},1e-8,"start jerk zero");near(q1.jerk,{},1e-8,"end jerk zero");
            for(int k=1;k<20;++k){double tt=s.start_time()+(s.end_time()-s.start_time())*double(k)/20;auto state=s.evaluate(tt);check_derivatives(s,tt);
                double v=norm(state.v),w=norm(state.omega);maxv=std::max(maxv,v);maxw=std::max(maxw,w);
                check(v<=c.limits.speed*(1+1e-9),"speed bound");check(norm(state.a)<=c.limits.acceleration*(1+1e-9),"acceleration bound");check(norm(state.jerk)<=c.limits.jerk*(1+1e-9),"jerk bound");check(w<=c.limits.angular_speed*(1+1e-9),"gyro bound");check(norm(state.alpha)<=c.limits.angular_acceleration*(1+1e-9),"angular acceleration bound");
                check(g.collision_world().point_free(state.p),"sampled geometry independent point test");
                if(m==Mode::Circular&&s.kind==SegmentKind::Arc){Vec3 reference=state.p+state.R*c.circle_reference_offset_body;Vec3 radial=unit(reference-s.reference_center);near(state.R*unit(c.optical_axis_body),radial,1e-9,"outward optical axis at actual reference CAMERA center");near(norm(reference-s.reference_center),s.reference_radius,1e-10,"exact reference camera circle radius");}
                if(m==Mode::Rectilinear&&s.label=="straight_leg"){Vec3 d=s.control[3]-s.control[0];near(norm(cross(state.p-s.control[0],unit(d))),0,1e-10,"exact straight path");}
                if(m==Mode::Mower){near(state.p.z,c.mower_imu_height,1e-10,"constant ground height");near(state.R*unit(c.up_body),{0,0,1},1e-9,"upright mower");if(v>1e-6){Vec3 fw=state.R*unit(c.optical_axis_body);near(dot(unit(state.v),unit(fw)),1,1e-9,"mower moves along heading; turns at rest");}}
            }
            t=s.end_time();State next=g.at(t);near(next.p,q1.p,3e-10,"C0 position join");near(next.R,q1.R,3e-10,"C0 attitude join");near(next.v,q1.v,1e-8,"C1 join");near(next.a,q1.a,1e-7,"C2 join");near(next.jerk,q1.jerk,1e-6,"C3 join");++n;
        }
        check(n>2,"enough random primitives");std::cout<<"PASS C3/derivatives/limits/profiles "<<mode_name(m)<<" primitives="<<n<<" observed_v="<<maxv<<" observed_w="<<maxw<<"\n";
    }
}
void test_circle_and_line_special_modes(){
    World w{{{-4,-4,0},{4,4,3}},{},0};Config c=profile(Mode::Circular);c.circle_fixed_center=true;c.circle_center={0,0,1.3};c.circle_radius=1;c.start={1,0,1.3};c.initial_rotation=look_rotation({1,0,0},c.optical_axis_body,c.up_body);c.start=Vec3{1,0,1.3}-c.initial_rotation*c.circle_reference_offset_body;
    Generator g(w,c);for(int i=0;i<2000;++i){auto s=g.at(i*.1);Vec3 reference=s.p+s.R*c.circle_reference_offset_body;near(norm(reference-c.circle_center),1,1e-9,"fixed-camera-circle radius");near(s.R*unit(c.optical_axis_body),unit(reference-c.circle_center),1e-9,"fixed-camera-circle outward axis");}
    c=profile(Mode::Rectilinear);c.rectilinear_fixed_axis=true;c.start={0,0,1.3};Generator l(w,c);for(int i=0;i<2000;++i){auto s=l.at(i*.1);near(s.p.x,0,1e-12,"fixed line x");near(s.p.z,1.3,1e-12,"fixed line z");}
    // Tilted/nondefault reference camera direction: outward means the CAMERA,
    // not an assumed body Y axis. Different stereo optical axes cannot both align.
    c=profile(Mode::Circular);c.start={0,0,1.3};c.optical_axis_body=unit(Vec3{.1,1,.25});Generator tilted(w,c);
    for(int i=0;i<1000;++i){auto s=tilted.at(i*.1);auto arc=tilted.current_segment();if(arc.kind==SegmentKind::Arc)near(s.R*unit(c.optical_axis_body),unit(s.p+s.R*c.circle_reference_offset_body-arc.reference_center),1e-9,"tilted reference camera aligned");}
    std::cout<<"PASS fixed circle, fixed line, nondefault optical axis\n";
}
void test_blocking_fork_serialization(){
    World w{{{-1,-1,0},{1,1,3}},{},0};auto c=profile(Mode::Rectilinear);c.start={0,0,1.3};c.pause_probability=0;c.step_min=4;c.step_max=5;
    Generator blocked(w,c);must_throw([&]{blocked.at(10);},"impossible size raises error");c.hold_when_blocked=true;Generator held(w,c);auto state=held.at(30);near(state.p,c.start,0,"blocked mode holds safely");check(held.diagnostics().blocked_holds>0,"blocking is counted");
    c=cfg(Mode::Controller);Generator g(room(),c);g.at(23.4);auto fork=g.fork();for(int i=0;i<600;++i)same(g.at(23.4+i*.013),fork.at(23.4+i*.013),"cursor fork checkpoint");
    g.at(50);Segment original=g.current_segment();std::ostringstream line;write_segment(line,original);Segment copy=read_segment(line.str());for(int k=0;k<30;++k){double t=original.start_time()+(original.end_time()-original.start_time())*k/30;same(original.evaluate(t),copy.evaluate(t),"primitive tape exact round trip");}
    // Separate cursors accessed concurrently, independent of thread scheduling.
    std::vector<State> left(100),right(100);auto f0=g.fork(),f1=g.fork();std::thread a([&]{for(int i=0;i<100;++i)left[i]=f0.at(60+i*.01);});std::thread b([&]{for(int i=99;i>=0;--i)right[i]=f1.at(60+i*.01);});a.join();b.join();for(int i=0;i<100;++i)same(left[i],right[i],"separate threaded readers");
    std::cout<<"PASS blocking policy, forks, tape replay, independent thread readers\n";
}

void test_safe_retreat() {
    auto c=cfg(Mode::Rectilinear);c.seed=0;c.max_attempts=2;
    c.step_min=.7;c.step_max=2.2;c.pause_probability=0;
    c.hold_when_blocked=true;c.history_capacity=4;
    Generator g(room(),c),dense(room(),c);
    bool saw_retreat=false;
    double t=0;
    while(t<600) {
        auto a=g.at(t);const auto segment=g.current_segment();
        check(g.collision_world().segment_free(segment),"certified retreat/retry geometry");
        if(segment.label.find("safe_retreat")!=std::string::npos) {
            saw_retreat=true;
            for(int i=1;i<8;++i) {
                double tt=segment.start_time()+(segment.end_time()-segment.start_time())*i/8.;
                auto s=segment.evaluate(tt);check_derivatives(segment,tt);
                check(norm(s.v)<=c.limits.speed*(1+1e-9),"retreat speed limit");
                check(norm(s.a)<=c.limits.acceleration*(1+1e-9),"retreat acceleration limit");
                same(g.at(tt),dense.at(tt),"retreat prefix and query-order independence");
            }
        }
        auto end=segment.final();t=segment.end_time();auto next=g.at(t);
        near(next.p,end.p,1e-9,"retreat boundary position continuity");
        near(next.R,end.R,1e-9,"retreat boundary attitude continuity");
        (void)a;
    }
    check(saw_retreat&&g.diagnostics().safe_retreats>0,"bounded recovery exercised");
    c.enable_safe_retreat=false;Generator disabled(room(),c);disabled.at(600);
    check(disabled.diagnostics().safe_retreats==0,"retreat opt-out respected");
    check(disabled.diagnostics().blocked_holds>0,"without recovery, impossible proposals produce explicit holds");
    std::cout<<"PASS bounded safe retreats, continuity, limits, replay, explicit opt-out count="<<g.diagnostics().safe_retreats<<"\n";
}

void test_imu_integration(){
    for(Mode m:{Mode::Scanner,Mode::Controller,Mode::Circular}){
        auto c=cfg(m);Generator g(room(),c);State s0=g.at(0);Vec3 p=s0.p,v=s0.v;Mat3 R=s0.R;double dt=.0005; // 2 kHz for high-motion numerical integration test.
        for(int k=0;k<16000;++k){State a=g.at(k*dt),b=g.at((k+1)*dt);Vec3 omega=(a.omega+b.omega)*.5;Mat3 rn=R*exp_so3(omega*dt);Vec3 acceleration=((R*a.specific_force())+(rn*b.specific_force()))*.5+Vec3{0,0,-9.81};p+=v*dt+acceleration*(.5*dt*dt);v+=acceleration*dt;R=rn;}
        State gt=g.at(8);near(p,gt.p,.006,"IMU reintegration position");near(v,gt.v,.004,"IMU reintegration velocity");near(R,gt.R,3e-4,"IMU reintegration attitude");
        std::cout<<"PASS midpoint ideal-IMU reintegration "<<mode_name(m)<<" position_error_m="<<norm(p-gt.p)<<"\n";
    }
}
}
int main(){try{auto t=std::chrono::steady_clock::now();test_inputs();test_options();test_collision();test_frequency_prefix_seed();test_continuity_limits_and_profiles();test_circle_and_line_special_modes();test_blocking_fork_serialization();test_safe_retreat();test_imu_integration();std::cout<<"PASS ALL checks="<<checks<<" elapsed_s="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<<"\n";return 0;}catch(std::exception&e){std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';return 1;}}
