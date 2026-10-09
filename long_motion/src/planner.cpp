#include "long_motion/long_motion.hpp"
#include <limits>
#include <sstream>

namespace long_motion {
namespace {
constexpr double S1=2.1875,S2=7.514,S3=52.5; // Certified upper bounds for the septic time law.
int64_t duration_ns(double proposal,double L1,double L2,double L3,double angle,const Limits& l) {
    double T=std::max(.01,proposal);
    T=std::max(T,L1*S1/l.speed);
    T=std::max(T,std::sqrt((L2*S1*S1+L1*S2)/l.acceleration));
    T=std::max(T,std::cbrt((L3*S1*S1*S1+3*L2*S1*S2+L1*S3)/l.jerk));
    T=std::max(T,angle*S1/l.angular_speed);
    T=std::max(T,std::sqrt(angle*S2/l.angular_acceleration));
    T=std::max(T,std::cbrt(angle*S3/l.angular_jerk));
    T*=1.000001; // Outward rounding margin for bounds and nanosecond conversion.
    if(!std::isfinite(T)||T>1e8)throw std::runtime_error("Primitive duration outside supported numerical range");
    return std::max<int64_t>(1,static_cast<int64_t>(std::ceil(T*1e9)));
}
std::array<Vec3,4> straight(Vec3 a,Vec3 b) { return {a,a+(b-a)/3.,a+(b-a)*(2./3.),b}; }
Vec3 sphere_clamp(Vec3 p,Vec3 center,double radius) {Vec3 d=p-center;double n=norm(d);return n>radius?center+d*(radius/n):p;}
}
Generator::Generator(World w,Config c):config_(std::move(c)) {
    validate(w,config_);
    // Geometry order never affects planning or the identity.
    std::sort(w.obstacles.begin(),w.obstacles.end(),[](Aabb a,Aabb b){for(int k=0;k<6;++k){double x=k<3?a.lo[k]:a.hi[k-3],y=k<3?b.lo[k]:b.hi[k-3];if(x!=y)return x<y;}return false;});
    signature_=signature(w,config_);
    world_=std::make_shared<World>(std::move(w));
    collision_=std::make_shared<CollisionWorld>(*world_,config_);
    if(config_.mode==Mode::Controller&&(config_.start.z<config_.height_min||config_.start.z>config_.height_max))throw std::invalid_argument("Controller workspace center must lie within the configured height band");
    reset();
}
void Generator::reset() {
    failed_=false;rectilinear_done_=false;diagnostics_={};pending_.clear();history_.clear();next_event_=0;mower_pass_=0;mower_lane_=0;mower_lanes_.clear();hand_anchor_=config_.start;
    State start;start.p=config_.start;start.R=config_.initial_rotation;
    current_=make_hold(start,std::max(1e-9,config_.warmup),0,"warmup");current_.start_ns=0;
}
double Generator::draw(uint64_t e,uint64_t stream,uint64_t index,double lo,double hi) const {return lo+(hi-lo)*random01(config_.seed,e,stream,index);}
Mat3 Generator::look(Vec3 forward) const {
    Vec3 axis=config_.optical_axis_body;
    if(config_.mode==Mode::Mower){Vec3 u=unit(config_.up_body);axis=axis-u*dot(axis,u);forward.z=0;}
    return look_rotation(forward,axis,config_.up_body);
}
Segment Generator::make_hold(State a,double seconds,uint64_t event,const std::string& label) const {
    if(!std::isfinite(seconds)||seconds<=0||seconds>1e8) throw std::invalid_argument("Invalid hold duration");
    Segment s;s.kind=SegmentKind::Hold;s.control.fill(a.p);s.R0=a.R;s.duration_ns=std::max<int64_t>(1,int64_t(std::ceil(seconds*1e9)));s.event=event;s.label=label;return s;
}
Segment Generator::make_bezier(State a,const std::array<Vec3,4>& p,const Mat3& R1,double T,uint64_t event,const std::string& label,double speed_cap) const {
    Segment s;s.kind=SegmentKind::Bezier;s.control=p;s.R0=a.R;s.rotation_vector=log_so3(a.R.transpose()*R1);s.event=event;s.label=label;
    double L1=0,L2=0;
    for(int i=0;i<3;++i)L1=std::max(L1,3*norm(p[i+1]-p[i]));
    for(int i=0;i<2;++i)L2=std::max(L2,6*norm(p[i+2]-p[i+1]*2+p[i]));
    double L3=6*norm(p[3]-p[2]*3+p[1]*3-p[0]);
    Limits limits=config_.limits;if(speed_cap>0)limits.speed=std::min(limits.speed,speed_cap);
    s.duration_ns=duration_ns(T,L1,L2,L3,norm(s.rotation_vector),limits);return s;
}
Segment Generator::make_arc(State a,Vec3 c,double r,double phi,double angle,double T,uint64_t event) const {
    Segment s;s.kind=SegmentKind::Arc;s.control.fill(a.p);s.center=c;s.radius=r;s.angle0=phi;s.angle_delta=angle;s.R0=a.R;s.event=event;s.label="outward_circular_arc";
    double d=std::abs(angle);s.duration_ns=duration_ns(T,r*d,r*d*d,r*d*d*d,d,config_.limits);return s;
}
bool Generator::append_route(State a,const std::vector<Vec3>& points,uint64_t event,const std::string& label,bool align,double speed_cap) {
    std::deque<Segment> route;
    for(size_t i=0;i<points.size();++i) {
        Vec3 b=points[i],d=b-a.p;if(norm(d)<1e-7)continue;
        if(!collision_->line_free(a.p,b))return false;
        Mat3 R=a.R;
        if(align) {
            R=look(d);
            if(norm(log_so3(a.R.transpose()*R))>1e-12) {
                auto turn=make_bezier(a,straight(a.p,a.p),R,.2,event,label+"_turn");
                route.push_back(turn);a=turn.final();
            }
        } else {
            Vec3 dir=d;dir.z=0;
            if(norm(dir)<1e-6)dir=a.R*config_.optical_axis_body;
            double yaw=draw(event,220,i*3,-.35*config_.look_yaw,.35*config_.look_yaw);
            dir=rz(yaw)*unit(dir);dir.z=draw(event,220,i*3+1,-.2,.1);
            R=look(dir)*exp_so3(unit(config_.optical_axis_body)*draw(event,220,i*3+2,-config_.look_roll,config_.look_roll));
        }
        auto move=make_bezier(a,straight(a.p,b),R,draw(event,221,i,config_.duration_min,config_.duration_max),event,label,speed_cap);
        if(!collision_->segment_free(move))return false;
        route.push_back(move);a=move.final();
    }
    if(route.empty())return false;
    pending_.insert(pending_.end(),route.begin(),route.end());return true;
}
bool Generator::plan_rectilinear(State a,uint64_t e) {
    if(config_.rectilinear_distance>0) {
        if(rectilinear_done_) {
            pending_.push_back(make_hold(a,1e8,e,"rectilinear_complete_hold"));
            return true;
        }
        ++diagnostics_.candidates;
        const Vec3 target=config_.start+unit(config_.line_axis)*config_.rectilinear_distance;
        if(!collision_->line_free(a.p,target) || !append_route(a,{target},e,"rectilinear_fixed_leg",true)) {
            ++diagnostics_.rejected;
            return false;
        }
        if(config_.rectilinear_final_hold>0)
            pending_.push_back(make_hold(pending_.back().final(),config_.rectilinear_final_hold,e,"rectilinear_final_hold"));
        rectilinear_done_=true;
        return true;
    }
    for(int k=0;k<config_.max_attempts;++k) {
        ++diagnostics_.candidates;Vec3 dir;
        if(config_.rectilinear_fixed_axis)dir=unit(config_.line_axis)*(draw(e,100,k*4)<.5?-1:1);
        else {double theta=draw(e,100,k*4,-pi,pi);dir={std::cos(theta),std::sin(theta),0};}
        Vec3 target=a.p+dir*draw(e,100,k*4+1,config_.step_min,config_.step_max);
        if(collision_->line_free(a.p,target)&&append_route(a,{target},e,"straight_leg",config_.align_with_travel))return true;
        ++diagnostics_.rejected;
    }
    return false;
}
bool Generator::plan_circular(State a,uint64_t e) {
    Vec3 forward=a.R*unit(config_.optical_axis_body);forward.z=0;
    if(norm(forward)<1e-8) forward={0,1,0};
    forward=unit(forward);
    for(int k=0;k<config_.max_attempts;++k) {
        ++diagnostics_.candidates;
        double r=draw(e,110,k*5,config_.circle_radius_min,config_.circle_radius_max);
        Vec3 radial= k<8?forward:rz(draw(e,110,k*5+1,-pi,pi))*forward;
        Mat3 R=look(radial);
        Vec3 reference_center=a.p+R*config_.circle_reference_offset_body-radial*r;
        if(config_.circle_fixed_center) {
            reference_center=config_.circle_center;r=config_.circle_radius;
            radial=unit(a.p+a.R*config_.circle_reference_offset_body-reference_center);
            // No pre-arc alignment here: it would move a nonzero camera lever arm
            // off the fixed circle. The initial pose has already been validated.
            R=a.R;
        }
        Vec3 offset=R*config_.circle_reference_offset_body;
        Vec3 center=reference_center-Vec3{0,0,offset.z};
        Vec3 body_radial=a.p-center;
        double body_radius=std::hypot(body_radial.x,body_radial.y);
        double phi=std::atan2(body_radial.y,body_radial.x);
        double angle=draw(e,110,k*5+2,config_.circle_angle_min,config_.circle_angle_max)*(draw(e,110,k*5+3)<.5?-1:1);
        if(!collision_->arc_free(center,body_radius,phi,angle)){++diagnostics_.rejected;continue;}
        State start=a;
        if(norm(log_so3(a.R.transpose()*R))>1e-12){auto turn=make_bezier(a,straight(a.p,a.p),R,.25,e,"circle_align_at_rest");pending_.push_back(turn);start=turn.final();}
        // Keep the incoming attitude if already aligned; never snap the pose.
        auto arc=make_arc(start,center,body_radius,phi,angle,draw(e,110,k*5+4,config_.duration_min,config_.duration_max),e);
        arc.reference_center=reference_center;arc.reference_radius=r;
        pending_.push_back(arc);return true;
    }
    return false;
}
bool Generator::plan_handheld(State a,uint64_t e,bool headset) {
    const bool walking=draw(e,120,0)<(headset?.55:.60);
    if(walking) {
        for(int k=0;k<config_.max_attempts;++k) {
            ++diagnostics_.candidates;double theta=draw(e,121,k*3,-pi,pi),length=draw(e,121,k*3+1,config_.step_min,config_.step_max);
            Vec3 target=a.p+Vec3{std::cos(theta)*length,std::sin(theta)*length,0};
            if(!collision_->point_free(target)){++diagnostics_.rejected;continue;}
            auto path=navigation(a.p,target);
            if(!path.empty()&&append_route(a,path,e,headset?"headset_walk":"scanner_walk",false))return true;
            ++diagnostics_.rejected;
        }
    }
    // Inspection, look-around and duck/reach events use small certified 3-D curves.
    for(int k=0;k<config_.max_attempts;++k) {
        ++diagnostics_.candidates;
        double theta=draw(e,122,k*10,-pi,pi),distance=draw(e,122,k*10+1,.02,headset?.25:.45);
        Vec3 d{std::cos(theta)*distance,std::sin(theta)*distance,0};
        if(headset&&draw(e,122,k*10+2)<.5)d={};
        Vec3 target=a.p+d;target.z=draw(e,122,k*10+3,config_.height_min,config_.height_max);
        Vec3 side{-d.y*.18,d.x*.18,0};
        auto p=straight(a.p,target);p[1]+=side;p[2]+=side;
        if(!collision_->bezier_free(p)){++diagnostics_.rejected;continue;}
        Vec3 fw=a.R*unit(config_.optical_axis_body);fw.z=0;if(norm(fw)<1e-6)fw={0,1,0};fw=unit(fw);
        Vec3 direction=rz(draw(e,122,k*10+4,-config_.look_yaw,config_.look_yaw))*fw;
        double pitch=draw(e,122,k*10+5,-config_.look_pitch,config_.look_pitch);direction=direction*std::cos(pitch);direction.z=std::sin(pitch);
        // A scanner sometimes faces a static fixture; the object list is canonical.
        if(!headset&&!world_->obstacles.empty()&&draw(e,122,k*10+6)<.5){size_t id=std::min(world_->obstacles.size()-1,size_t(draw(e,122,k*10+7)*world_->obstacles.size()));Vec3 object=(world_->obstacles[id].lo+world_->obstacles[id].hi)*.5;object.z=target.z; if(norm(object-target)>.1)direction=object-target;}
        Mat3 R=look(direction)*exp_so3(unit(config_.optical_axis_body)*draw(e,122,k*10+8,-config_.look_roll,config_.look_roll));
        auto s=make_bezier(a,p,R,draw(e,122,k*10+9,config_.duration_min,config_.duration_max),e,headset?"headset_gaze_duck":"scanner_inspection");pending_.push_back(s);return true;
    }
    return false;
}
bool Generator::plan_controller(State a,uint64_t e) {
    if(e>0&&config_.controller_relocate_every>0&&e%uint64_t(config_.controller_relocate_every)==0) {
        for(int k=0;k<config_.max_attempts;++k) {
            ++diagnostics_.candidates;
            double theta=draw(e,130,k*2,-pi,pi),dist=draw(e,130,k*2+1,.4,config_.step_max);
            Vec3 target=a.p+Vec3{dist*std::cos(theta),dist*std::sin(theta),0};
            target.z=config_.start.z;
            const double radius=config_.gesture_radius;
            Aabb workspace{target-Vec3{radius,radius,radius},target+Vec3{radius,radius,radius}};
            workspace.lo.z=std::max(workspace.lo.z,config_.height_min);
            workspace.hi.z=std::min(workspace.hi.z,config_.height_max);
            // A new player location must leave room for the requested hand volume.
            // Otherwise remain in the current play area instead of walking into a
            // narrow corner and subsequently having no feasible gestures.
            if(collision_->classify(workspace)!=1){++diagnostics_.rejected;continue;}
            Vec3 horizontal=target;horizontal.z=a.p.z;
            auto path=navigation(a.p,horizontal);
            if((path.empty()&&norm(a.p-horizontal)>1e-7)||!collision_->line_free(horizontal,target)){++diagnostics_.rejected;continue;}
            path.push_back(target);
            if(append_route(a,path,e,"controller_roomscale_relocation",false,config_.controller_relocate_speed)){hand_anchor_=target;return true;}
            ++diagnostics_.rejected;
        }
        // Still permit local play when room-scale relocation is unavailable.
    }
    for(int k=0;k<config_.max_attempts;++k) {
        ++diagnostics_.candidates;
        std::array<Vec3,4> p;p[0]=a.p;
        const double scale=std::pow(.6,k/std::max(1,config_.max_attempts/4));
        Vec3 local_center=a.p+(hand_anchor_-a.p)*scale;
        for(int j=1;j<4;++j){double az=draw(e,131,k*16+j*3,-pi,pi),z=draw(e,131,k*16+j*3+1,-.8,.8),r=draw(e,131,k*16+j*3+2,.45,.98)*config_.gesture_radius*scale;double h=std::sqrt(1-z*z);
            p[j]=local_center+Vec3{h*std::cos(az),h*std::sin(az),z}*r;
            p[j].z=std::clamp(p[j].z,config_.height_min,config_.height_max);
            p[j]=sphere_clamp(p[j],hand_anchor_,config_.gesture_radius);
        }
        if(!collision_->bezier_free(p)){++diagnostics_.rejected;continue;}
        Vec3 previous_forward=a.R*unit(config_.optical_axis_body);
        double az=std::atan2(previous_forward.y,previous_forward.x)+draw(e,132,k*4,-config_.look_yaw,config_.look_yaw),pitch=draw(e,132,k*4+1,-config_.look_pitch,config_.look_pitch);
        Vec3 direction{std::cos(az)*std::cos(pitch),std::sin(az)*std::cos(pitch),std::sin(pitch)};
        Mat3 R=look(direction)*exp_so3(unit(config_.optical_axis_body)*draw(e,132,k*4+2,-config_.look_roll,config_.look_roll));
        auto s=make_bezier(a,p,R,draw(e,132,k*4+3,config_.duration_min,config_.duration_max),e,"controller_saber_swing");pending_.push_back(s);return true;
    }
    // A certified device can still rotate safely in its enclosing sphere. This
    // constrained-space gesture is explicitly labeled, not a silent pose clamp.
    Vec3 axis{draw(e,133,0,-1,1),draw(e,133,1,-1,1),draw(e,133,2,-1,1)};
    if(norm(axis)<1e-8) axis=config_.up_body;
    axis=unit(axis);
    const double max_angle=std::min(1.5,std::max({config_.look_yaw,config_.look_pitch,config_.look_roll}));
    if(max_angle<1e-12) return false;
    Mat3 target=a.R*exp_so3(axis*draw(e,133,3,std::min(.3,max_angle),max_angle));
    pending_.push_back(make_bezier(a,straight(a.p,a.p),target,.5,e,"controller_constrained_rotation"));
    return true;
}
void Generator::make_mower_lanes(double z) {
    mower_lanes_.clear();mower_lane_=0;
    bool swap_axes=config_.mower_alternate_axis&&(mower_pass_%2==1);
    int lateral=swap_axes?1:0,longitudinal=swap_axes?0:1;
    auto bounds=collision_->allowed;
    double spacing=config_.mower_lane_spacing*draw(mower_pass_,140,0,.9,1.1);
    double expected_lanes=(bounds.hi[lateral]-bounds.lo[lateral])/spacing;
    if(!std::isfinite(expected_lanes)||expected_lanes>config_.nav_max_nodes) throw std::runtime_error("Mower lane budget exceeded; increase lane spacing");
    double offset=draw(mower_pass_,140,1,.12,.88)*spacing;
    double low=bounds.lo[longitudinal]+.002,high=bounds.hi[longitudinal]-.002;
    size_t lane_number=0;
    for(double cross_pos=bounds.lo[lateral]+offset;cross_pos<bounds.hi[lateral]-.002;cross_pos+=spacing,++lane_number) {
        std::vector<std::pair<double,double>> blocked;
        for(auto box:collision_->forbidden)if(cross_pos>=box.lo[lateral]&&cross_pos<=box.hi[lateral]&&z>=box.lo.z&&z<=box.hi.z)blocked.emplace_back(std::max(low,box.lo[longitudinal]-.002),std::min(high,box.hi[longitudinal]+.002));
        std::sort(blocked.begin(),blocked.end());std::vector<std::pair<double,double>> intervals;
        double cursor=low;for(auto b:blocked){if(b.second<cursor||b.first>high)continue;if(b.first-cursor>.2)intervals.emplace_back(cursor,b.first);cursor=std::max(cursor,b.second);}if(high-cursor>.2)intervals.emplace_back(cursor,high);
        bool reverse=((lane_number+mower_pass_)%2)==1;
        if(reverse)std::reverse(intervals.begin(),intervals.end());
        for(auto interval:intervals){Vec3 a{},b{};a.z=b.z=z;a[lateral]=b[lateral]=cross_pos;a[longitudinal]=reverse?interval.second:interval.first;b[longitudinal]=reverse?interval.first:interval.second;
            if(collision_->line_free(a,b))mower_lanes_.emplace_back(a,b);
            if(mower_lanes_.size()>size_t(config_.nav_max_nodes)) throw std::runtime_error("Mower interval budget exceeded");}
    }
    ++mower_pass_;
}
bool Generator::plan_mower(State a,uint64_t e) {
    for(int k=0;k<config_.max_attempts;++k) {
        if(mower_lane_>=mower_lanes_.size())make_mower_lanes(a.p.z);
        if(mower_lanes_.empty())return false;
        auto lane=mower_lanes_[mower_lane_++];++diagnostics_.candidates;
        auto path=navigation(a.p,lane.first);
        if(path.empty()&&norm(a.p-lane.first)>1e-6){++diagnostics_.rejected;continue;}
        path.push_back(lane.second);
        if(append_route(a,path,e,"mower_sweep",true))return true;
        ++diagnostics_.rejected;
    }
    return false;
}
bool Generator::plan_retreat(State a,uint64_t e) {
    if(!config_.enable_safe_retreat||history_.empty()) return false;
    const Segment previous=history_.back();
    // Failed proposals have not changed the pose. Retrace a known safe geometric
    // path instead of clipping or teleporting out of a tight free-space pocket.
    if(norm(previous.final().p-a.p)>1e-7) throw std::logic_error("Certified breadcrumb does not end at current position");
    bool ok=false;
    if(config_.mode==Mode::Mower) {
        ok=append_route(a,{previous.initial().p},e,"mower_safe_retreat",true);
    } else if(previous.kind==SegmentKind::Arc) {
        auto reverse=make_arc(a,previous.center,previous.radius,
            std::remainder(previous.angle0+previous.angle_delta,2*pi),
            -previous.angle_delta,previous.duration_ns*1e-9,e);
        reverse.reference_center=previous.reference_center;
        reverse.reference_radius=previous.reference_radius;
        reverse.label="circle_safe_retreat";
        if(collision_->segment_free(reverse)){pending_.push_back(reverse);ok=true;}
    } else {
        auto controls=previous.control;std::reverse(controls.begin(),controls.end());
        auto reverse=make_bezier(a,controls,previous.R0,previous.duration_ns*1e-9,e,"safe_retreat");
        if(collision_->segment_free(reverse)){pending_.push_back(reverse);ok=true;}
    }
    if(ok){history_.pop_back();++diagnostics_.safe_retreats;}
    return ok;
}
bool Generator::plan_event(State a) {
    uint64_t e=next_event_++;
    if(next_event_==0)throw std::overflow_error("Motion event index overflow");
    ++diagnostics_.events;
    if(draw(e,10,0)<config_.pause_probability){pending_.push_back(make_hold(a,draw(e,10,1,config_.pause_min,config_.pause_max),e,"intentional_pause"));return true;}
    bool ok=false;
    switch(config_.mode){
    case Mode::Rectilinear:ok=plan_rectilinear(a,e);break;
    case Mode::Circular:ok=plan_circular(a,e);break;
    case Mode::Scanner:ok=plan_handheld(a,e,false);break;
    case Mode::XRHeadset:ok=plan_handheld(a,e,true);break;
    case Mode::Controller:ok=plan_controller(a,e);break;
    case Mode::Mower:ok=plan_mower(a,e);break;
    }
    if(ok)return true;
    if(plan_retreat(a,e)) return true;
    if(config_.hold_when_blocked){++diagnostics_.blocked_holds;pending_.push_back(make_hold(a,config_.pause_max,e,"BLOCKED_safe_hold"));return true;}
    throw std::runtime_error("No certified feasible motion in mode "+mode_name(config_.mode)+" at event "+std::to_string(e)+". Check clearance, size, height, navigation resolution, or explicitly enable hold_when_blocked.");
}
void Generator::next_segment() {
    State end=current_.final();int64_t start=current_.start_ns+current_.duration_ns;
    bool moved=current_.kind==SegmentKind::Arc&&std::abs(current_.angle_delta)>1e-9;
    for(int i=1;i<4;++i) moved=moved||norm(current_.control[i]-current_.control[0])>1e-8;
    if(moved&&current_.label.find("safe_retreat")==std::string::npos&&config_.history_capacity>0) {
        history_.push_back(current_);
        while(history_.size()>size_t(config_.history_capacity))history_.pop_front();
    }
    if(pending_.empty())plan_event(end);
    if(pending_.empty())throw std::logic_error("Planner produced no primitive");
    current_=pending_.front();pending_.pop_front();current_.start_ns=start;
    if(start>std::numeric_limits<int64_t>::max()-current_.duration_ns)throw std::overflow_error("Trajectory timestamp exceeds int64 nanoseconds");
    if(!collision_->segment_free(current_))throw std::logic_error("Internal error: an uncertified primitive reached the cursor");
    ++diagnostics_.segments;
}
State Generator::at(double t) {
    if(failed_) throw std::runtime_error("Trajectory cursor is in a failed state; reset before retrying the same deterministic plan");
    if(!std::isfinite(t)||t<0||t>9e9)throw std::invalid_argument("Trajectory time must be finite, nonnegative, and within int64 nanoseconds");
    if(t<current_.start_time()) {uint64_t rewinds=diagnostics_.rewinds+1;reset();diagnostics_.rewinds=rewinds;}
    try { while(t>=current_.end_time()) next_segment(); }
    catch(...) { failed_=true; throw; }
    return current_.evaluate(t);
}
} // namespace long_motion
