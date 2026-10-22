#include "viosim/trajectory.hpp"
#include "viosim/camera.hpp"
#include "viosim/scene.hpp"
#include "long_motion/long_motion.hpp"
#include "long_motion/options.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace vio {
namespace {
long_motion::Vec3 to_long(Vec3 v) { return {v.x,v.y,v.z}; }
long_motion::Mat3 to_long(const Mat3& m) { long_motion::Mat3 result; result.a=m.a; return result; }
Vec3 from_long(long_motion::Vec3 v) { return {v.x,v.y,v.z}; }
Mat3 from_long(const long_motion::Mat3& m) { Mat3 result; result.a=m.a; return result; }

long_motion::World motion_world(const Scene& scene) {
    long_motion::World world;
    world.room={to_long(scene.room.lo),to_long(scene.room.hi)};
    world.floor_z=scene.room.lo.z;
    world.obstacles.reserve(scene.boxes.size());
    for(const auto& box:scene.boxes)
        world.obstacles.push_back({to_long(box.bounds.lo),to_long(box.bounds.hi)});
    return world;
}

long_motion::Config motion_config(const Config& host,const Scene& scene,const Rig& rig) {
    const auto mode=long_motion::parse_mode(host.motion_mode);
    auto config=long_motion::profile(mode);
    const Mat3 basis=imu_basis_rotation(host);
    config.optical_axis_body=to_long(basis.transpose()*Vec3{0,1,0});
    config.up_body=to_long(basis.transpose()*Vec3{0,0,1});
    config.initial_rotation=to_long(basis);
    config.warmup=host.warmup;
    config.start.x=scene.anchor.x;
    config.start.y=scene.anchor.y;
    if(mode==long_motion::Mode::Mower)
        config.start.z=scene.room.lo.z+config.mower_imu_height;

    if(!host.motion_config.empty()) {
        std::ifstream input(host.motion_config);
        if(!input) throw std::runtime_error("Cannot read motion config: "+host.motion_config);
        try {
            const auto options=long_motion::read_flat_options(input);
            for(const auto& option:options)
                long_motion::set_option(config,option.first,option.second);
            if(mode==long_motion::Mode::Mower && !options.count("start"))
                config.start.z=scene.room.lo.z+config.mower_imu_height;
        } catch(const std::exception& error) {
            throw std::runtime_error("Invalid motion config "+host.motion_config+": "+error.what());
        }
    }

    if(host.motion_fixed_distance>0) {
        if(mode!=long_motion::Mode::Rectilinear)
            throw std::invalid_argument("motion_fixed_distance requires motion_mode rectilinear");
        config.rectilinear_fixed_axis=true;
        config.line_axis={0,1,0};
        config.rectilinear_distance=host.motion_fixed_distance;
        config.rectilinear_final_hold=host.motion_final_hold;
        config.warmup=host.warmup;
        config.duration_min=config.duration_max=host.motion_move_duration;
        config.limits.speed=std::max(config.limits.speed,host.motion_fixed_distance*2.2/host.motion_move_duration);
        config.pause_probability=0;
    }

    // The dataset configuration owns the selected family and its independent
    // seed. Physical rig geometry always comes from the current calibration.
    config.mode=mode;
    config.seed=host.motion_seed;
    config.camera_offsets.clear();
    for(const auto& camera:rig.BC) config.camera_offsets.push_back(to_long(camera.p));

    if(host.motion_reference_camera>=0) {
        const auto camera=static_cast<size_t>(host.motion_reference_camera);
        if(camera>=rig.size()) throw std::invalid_argument("motion_reference_camera is outside the configured rig");
        const Pose& reference=rig.BC[camera];
        config.optical_axis_body=to_long(reference.R*Vec3{0,0,1});
        // Keep the reference camera looking along world +Y during the static
        // prefix. The IMU pose is rotated accordingly, so IMU samples still
        // use the calibrated IMU axes rather than an assumed camera-aligned rig.
        const Mat3 camera_to_world{{1,0,0, 0,0,1, 0,-1,0}};
        config.initial_rotation=to_long(camera_to_world*reference.R.transpose());
        config.up_body=to_long(reference.R*Vec3{0,-1,0});
        config.circle_reference_offset_body=to_long(rig.BC[camera].p);
    } else {
        config.circle_reference_offset_body={};
    }
    return config;
}

State from_long(const long_motion::State& state) {
    return {from_long(state.p),from_long(state.v),from_long(state.a),from_long(state.omega),from_long(state.R)};
}
}

struct TrajectoryCursor::Impl {
    Config config;
    Vec3 anchor;
    std::unique_ptr<long_motion::Generator> generator;
    std::string algorithm_name="viosim_legacy";
    std::string signature_value;
    std::string resolved;

    Impl(const Config& config_,const Scene& scene,const Rig& rig):config(config_),anchor(scene.anchor) {
        if(config.trajectory!="random_long") return;
        auto world=motion_world(scene);
        auto motion=motion_config(config,scene,rig);
        generator=std::make_unique<long_motion::Generator>(std::move(world),motion);
        algorithm_name=long_motion::algorithm_version;
        signature_value=generator->trajectory_signature();
        std::ostringstream output;
        long_motion::write_config(output,generator->config());
        resolved=output.str();
    }
};

TrajectoryCursor::TrajectoryCursor(const Config& config,const Scene& scene,const Rig& rig)
    :impl_(std::make_unique<Impl>(config,scene,rig)) {}
TrajectoryCursor::~TrajectoryCursor()=default;
TrajectoryCursor::TrajectoryCursor(TrajectoryCursor&&) noexcept=default;
TrajectoryCursor& TrajectoryCursor::operator=(TrajectoryCursor&&) noexcept=default;
State TrajectoryCursor::at(double seconds) {
    if(!impl_->generator) return state_at(seconds,impl_->config,impl_->anchor);
    const double scale=impl_->config.motion_time_scale;
    State state=from_long(impl_->generator->at(impl_->config.motion_time_offset+seconds*scale));
    state.v=state.v*scale; state.a=state.a*(scale*scale); state.omega=state.omega*scale;
    return state;
}
bool TrajectoryCursor::continuously_certified() const { return bool(impl_->generator); }
double TrajectoryCursor::current_segment_end() const {
    if(!impl_->generator) throw std::logic_error("Legacy trajectory has no motion primitives");
    return (impl_->generator->current_segment().end_time()-impl_->config.motion_time_offset)/impl_->config.motion_time_scale;
}
const std::string& TrajectoryCursor::algorithm() const { return impl_->algorithm_name; }
const std::string& TrajectoryCursor::signature() const { return impl_->signature_value; }
const std::string& TrajectoryCursor::resolved_config() const { return impl_->resolved; }
size_t TrajectoryCursor::write_primitive_tape(std::ostream& output,double duration) {
    if(!impl_->generator) return 0;
    long_motion::write_segment_header(output);
    // Primitive tape stays in the generator clock; config_used.yaml records
    // offset and scale needed to map this clock to sensor time.
    duration=impl_->config.motion_time_offset+duration*impl_->config.motion_time_scale;
    impl_->generator->at(impl_->config.motion_time_offset);
    size_t count=0;
    for(;;) {
        const auto& segment=impl_->generator->current_segment();
        if(segment.start_time()>=duration) break;
        long_motion::write_segment(output,segment);
        ++count;
        if(segment.end_time()>=duration) break;
        impl_->generator->at(segment.end_time());
    }
    return count;
}
MotionDiagnostics TrajectoryCursor::diagnostics() const {
    if(!impl_->generator) return {};
    const auto& d=impl_->generator->diagnostics();
    return {d.events,d.candidates,d.rejected,d.blocked_holds,d.safe_retreats};
}

Jet motion_clock(double t,const Config& c) {
    const double u=t-c.warmup;
    if(u<=0) return {};
    if(u>=c.ramp) return {(u-.5*c.ramp)*c.speed_scale,c.speed_scale,0};
    // Integral of 10 z^3 - 15 z^4 + 6 z^5. Velocity and acceleration join continuously.
    const Jet z(u/c.ramp,1/c.ramp,0),z2=z*z,z3=z2*z,z4=z3*z;
    return (Jet(2.5)*z4-Jet(3)*z4*z+z3*z3)*Jet(c.ramp*c.speed_scale);
}
Jet straight_distance(double t,const Config& c) {
    const double u=t-c.warmup;
    if(u<=0) return {};
    if(u>=c.period) return {c.size,0,0};
    // Quintic minimum-jerk profile: position, velocity and acceleration are
    // continuous at both static phases.
    const Jet z(u/c.period,1/c.period,0),z2=z*z,z3=z2*z,z4=z3*z,z5=z4*z;
    return (Jet(10)*z3-Jet(15)*z4+Jet(6)*z5)*Jet(c.size);
}
State state_at(double t,const Config& c,Vec3 anchor) {
    const auto in_imu_frame=[&](State state) {
        const Mat3 basis=imu_basis_rotation(c);
        state.R=state.R*basis;
        state.omega=basis.transpose()*state.omega;
        return state;
    };
    if(c.trajectory=="multilevel_walk" || c.trajectory=="tables_walk") {
        struct Node { double time,x,y,z,yaw; };
        const bool flat=c.trajectory=="tables_walk";
        const Node nodes[]={{0,0,1.6,1.6,0},{3,-.6,2.5,1.6,-10},
            {6,.5,3,1.6,10},{9,-.4,2.5,1.6,-8},{12,0,4,1.6,0},
            {24,0,10,4.6,0},{28,.5,11.6,4.6,8},{32,-.5,13,4.6,-8},
            {36,.5,14,4.6,10},{40,0,10,4.6,180},{52,0,4,1.6,180},
            {56,-.5,3,1.6,175},{60,.5,2,1.6,0},{64,-.5,3,1.6,-10},
            {70,0,1.6,1.6,0}};
        const double rate=70/c.period,clock=std::max(0.0,(t-c.warmup)*rate);
        size_t index=0;
        while(index+1<std::size(nodes) && clock>nodes[index+1].time) ++index;
        const auto& a=nodes[index]; const auto& b=nodes[std::min(index+1,std::size(nodes)-1)];
        Jet blend;
        if(b.time>a.time && t>c.warmup && clock<b.time) {
            const Jet u((clock-a.time)/(b.time-a.time),rate/(b.time-a.time));
            const Jet u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
            blend=Jet(10)*u3-Jet(15)*u4+Jet(6)*u5;
        } else if(clock>=b.time) blend=Jet(1);
        const Jet x=Jet(a.x)+Jet(b.x-a.x)*blend,y=Jet(a.y)+Jet(b.y-a.y)*blend;
        Jet z=flat?Jet(1.6):Jet(a.z)+Jet(b.z-a.z)*blend;
        // A crouch-and-return on a flat floor is a negative level-change control.
        if(flat && clock>16 && clock<22) {
            const Jet phase((clock-16)*pi/6,rate*pi/6);
            const Jet pulse=sin(phase)*sin(phase);
            z=z-Jet(.55)*pulse*pulse;
        }
        const Jet yaw=Jet(radians(a.yaw))+Jet(radians(b.yaw-a.yaw))*blend;
        // Look down at the floor / treads; rotation derivatives feed the IMU.
        const Jet roll=Jet(radians(-18))+Jet(radians(3))*sin(blend*Jet(2*pi));
        const Jet pitch=Jet(radians(2))*sin(blend*Jet(2*pi));
        const Mat3 Rz=rz(-yaw.v),Rx=rx(roll.v),Ry=ry(pitch.v);
        State state; state.p={x.v,y.v,z.v}; state.v={x.d,y.d,z.d}; state.a={x.dd,y.dd,z.dd};
        state.R=Rz*Rx*Ry;
        state.omega=Ry.transpose()*(Rx.transpose()*Vec3{0,0,-yaw.d})
            +Ry.transpose()*Vec3{roll.d,0,0}+Vec3{0,pitch.d,0};
        return in_imu_frame(state);
    }
    const Jet phase=motion_clock(t,c)*Jet(2*pi/c.period);
    const double scale=c.size;
    Jet x,y,z;
    if(c.trajectory=="figure8_forward") {
        // Planar eight: orient the reference camera along the tangent, while
        // keeping its calibrated relationship to the IMU (body) axes.
        // The tangent remains defined during warmup.
        x=Jet(.55*scale)*sin(phase); y=Jet(.65*scale)*sin(Jet(2)*phase);
        const Jet tx=Jet(.55)*cos(phase),ty=Jet(1.3)*cos(Jet(2)*phase);
        const double yaw=std::atan2(tx.v,ty.v);
        const double yaw_rate=(ty.v*tx.d-tx.v*ty.d)/(tx.v*tx.v+ty.v*ty.v);
        const Mat3 camera_to_world{{1,0,0, 0,0,1, 0,-1,0}};
        const Mat3 reference_camera=Rig(c).BC.at(size_t(c.motion_reference_camera)).R;
        const Mat3 body_alignment=camera_to_world*reference_camera.transpose();
        State s; s.p=anchor+Vec3{x.v,y.v,0}; s.v={x.d,y.d,0}; s.a={x.dd,y.dd,0};
        s.R=rz(-yaw)*body_alignment;
        s.omega=body_alignment.transpose()*Vec3{0,0,-yaw_rate};
        return s;
    } else if(c.trajectory=="figure8") {
        x=Jet(.55*scale)*sin(phase); y=Jet(.65*scale)*sin(Jet(2)*phase); z=Jet(.06*scale)*sin(phase);
    } else if(c.trajectory=="ellipse") {
        x=Jet(.9*scale)*sin(phase); y=Jet(1.35*scale)*cos(phase); z=Jet(.07*scale)*sin(Jet(2)*phase);
    } else if(c.trajectory=="back_and_forth") {
        y=Jet(1.4*scale)*sin(phase);
    } else if(c.trajectory=="slow_forward") {
        y=Jet(.28*scale/c.period)*motion_clock(t,c);
    } else if(c.trajectory=="straight") {
        y=straight_distance(t,c);
    }
    // A smooth yaw sweep avoids undefined tangent headings at reversals.
    // static, slow_forward and straight keep a fixed attitude; pure_rotation has fixed IMU origin.
    Jet yaw(radians(c.yaw_offset_deg)),roll,pitch;
    if(c.trajectory!="static" && c.trajectory!="slow_forward" && c.trajectory!="straight") {
        yaw=yaw+Jet(radians(c.yaw_deg))*sin(phase);
        roll=Jet(radians(c.roll_deg))*sin(phase);
        pitch=Jet(radians(c.pitch_deg))*sin(Jet(2)*phase);
    }
    if(c.yaw_turn_deg!=0) {
        const double u=(t-c.yaw_turn_start)/c.yaw_turn_duration;
        if(u>=1) yaw=yaw+Jet(radians(c.yaw_turn_deg));
        else if(u>0) {
            const Jet z(u,1/c.yaw_turn_duration,0);
            const Jet z2=z*z,z3=z2*z,z4=z3*z,z5=z4*z;
            yaw=yaw+Jet(radians(c.yaw_turn_deg))*(Jet(10)*z3-Jet(15)*z4+Jet(6)*z5);
        }
    }
    const Mat3 Rz=rz(-yaw.v),Rx=rx(roll.v),Ry=ry(pitch.v);
    State s; s.p=anchor+Vec3{x.v,y.v,z.v}; s.v={x.d,y.d,z.d}; s.a={x.dd,y.dd,z.dd}; s.R=Rz*Rx*Ry;
    s.omega=Ry.transpose()*(Rx.transpose()*Vec3{0,0,-yaw.d})+Ry.transpose()*Vec3{roll.d,0,0}+Vec3{0,pitch.d,0};
    return in_imu_frame(s);
}
} // namespace vio
