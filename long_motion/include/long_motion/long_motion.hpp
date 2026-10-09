#pragma once
#include "math.hpp"
#include <cstdint>
#include <deque>
#include <iosfwd>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace long_motion {
inline constexpr const char* algorithm_version="long_motion/1.0.0";
enum class Mode { Rectilinear, Circular, Scanner, XRHeadset, Controller, Mower };
std::string mode_name(Mode);
Mode parse_mode(const std::string&);
struct Aabb { Vec3 lo,hi; };
struct World {
    Aabb room;
    std::vector<Aabb> obstacles;
    // The floor of the CURRENT renderer. Mower supports a flat horizontal floor.
    double floor_z=0;
};
struct Limits {
    double speed=1.0,acceleration=3.0,jerk=20.0;
    double angular_speed=3.0,angular_acceleration=10.0,angular_jerk=80.0;
};
struct Config {
    Mode mode=Mode::Scanner;
    uint64_t seed=1234;
    Vec3 start{0,1.6,1.3};
    Mat3 initial_rotation;
    Vec3 optical_axis_body{0,1,0},up_body{0,0,1};
    // Reference optical CENTER as well as axis. Default = cam0 of an 8 cm rig.
    // Set to zero for a virtual optical center at the IMU/stereo midpoint.
    Vec3 circle_reference_offset_body{-.04,0,0};
    // Radius containing the device around the IMU; camera extrinsics are also
    // included. This is NOT a human-body or imaginary light-saber collision model.
    double body_radius=.08,camera_radius=.025,clearance=.06;
    std::vector<Vec3> camera_offsets{{-.04,0,0},{.04,0,0}};
    double warmup=1.0;
    Limits limits;
    double duration_min=1.0,duration_max=4.0; // Primitive timing proposals, not recording length.
    double step_min=.3,step_max=2.0;
    double height_min=.9,height_max=1.7;
    double pause_probability=.08,pause_min=.15,pause_max=.8;
    double look_yaw=rad(60),look_pitch=rad(25),look_roll=rad(10);
    double nav_resolution=.28;
    int nav_max_nodes=60000,max_attempts=64,collision_depth=18;
    double numerical_guard=1e-6;
    // "error" never silently turns a failed motion into a stationary dataset.
    // "hold" waits safely and retries on the next event; diagnostics count holds.
    bool hold_when_blocked=false;
    bool enable_safe_retreat=true;
    int history_capacity=64; // Bounded certified breadcrumbs, independent of recording length.
    bool rectilinear_fixed_axis=false;
    Vec3 line_axis{0,1,0};
    bool align_with_travel=true;
    // A positive distance selects one finite forward leg followed by the final
    // hold. Zero keeps the regular indefinite rectilinear generator.
    double rectilinear_distance=0,rectilinear_final_hold=0;
    double circle_radius_min=.45,circle_radius_max=1.4;
    double circle_angle_min=rad(80),circle_angle_max=rad(290);
    bool circle_fixed_center=false;
    Vec3 circle_center{0,3,1.3};
    double circle_radius=1.0;
    // Local hand workspace, centered on the initial hand pose; optional slow
    // room-scale relocation of this workspace after N gesture events (0 disables).
    double gesture_radius=.7;
    int controller_relocate_every=16;
    double controller_relocate_speed=.6;
    // Upright chassis proxy enclosing ALL yaw orientations, not a point robot.
    double mower_radius=.32,mower_imu_height=.45,mower_top_above_imu=.12;
    double mower_lane_spacing=.48;
    bool mower_alternate_axis=true;
};
Config profile(Mode);
void validate(const World&,const Config&);
std::string signature(const World&,const Config&);

struct State {
    Vec3 p,v,a,jerk,omega,alpha;
    Mat3 R; // R_WB: body to world. omega and alpha expressed in body coordinates.
    Vec3 specific_force(Vec3 gravity={0,0,-9.81}) const { return R.transpose()*(a-gravity); }
};
enum class SegmentKind { Bezier, Arc, Hold };
struct Segment {
    SegmentKind kind=SegmentKind::Hold;
    int64_t start_ns=0,duration_ns=1;
    uint64_t event=0;
    std::string label;
    // A cubic geometric Bezier composed with a seventh-order time law.
    std::array<Vec3,4> control{};
    // Exact horizontal circular arc: p = center + radius*(cos(theta),sin(theta),0).
    Vec3 center;
    double radius=0,angle0=0,angle_delta=0;
    // Center/radius of the reference CAMERA orbit, not necessarily the IMU orbit.
    Vec3 reference_center;
    double reference_radius=0;
    Mat3 R0;
    // Bezier/Hold orientation R=R0*Exp(rotation_vector*s). For an Arc,
    // R=Rz(angle_delta*s)*R0, so the reference optical axis stays radial.
    Vec3 rotation_vector;
    State evaluate(double seconds) const;
    State initial() const { return evaluate(double(start_ns)*1e-9); }
    State final() const { return evaluate(end_time()); }
    double start_time() const { return double(start_ns)*1e-9; }
    double end_time() const {
        if(start_ns<0 || duration_ns<=0 || start_ns>std::numeric_limits<int64_t>::max()-duration_ns)
            throw std::invalid_argument("Invalid primitive timestamp range");
        return double(start_ns+duration_ns)*1e-9;
    }
};

// Inflation of the room and obstacles into IMU-center configuration space.
// All collision decisions are conservative; no timed collision sampling.
class CollisionWorld {
public:
    CollisionWorld(const World&,const Config&);
    Aabb allowed;
    std::vector<Aabb> forbidden;
    bool point_free(Vec3) const;
    bool line_free(Vec3,Vec3) const;
    bool bezier_free(const std::array<Vec3,4>&) const;
    bool arc_free(Vec3 center,double radius,double angle0,double angle_delta) const;
    bool segment_free(const Segment&) const;
    // 1 => complete box certified free, -1 => outside or inside an obstacle,
    // 0 => inconclusive. Unknown leaves at max depth are rejected.
    int classify(Aabb) const;
    int depth=18;
private:
    bool bezier_recursive(const std::array<Vec3,4>&,int) const;
    bool arc_recursive(Vec3,double,double,double,int) const;
};

struct Diagnostics {
    uint64_t events=0,segments=0,candidates=0,rejected=0,blocked_holds=0,safe_retreats=0,rewinds=0;
};
// Streaming cursor: memory depends on scene/navigation size, NOT recording length.
// at(t) supports earlier queries by deterministic reset/replay. For performance,
// use separate cursors for preflight, sorted camera times, and sorted IMU times.
// A cursor is NOT shared-mutable-thread-safe; copy/fork it for each reader.
class Generator {
public:
    Generator(World,Config);
    State at(double seconds);
    State at_ns(int64_t relative_ns) { return at(double(relative_ns)*1e-9); }
    const Segment& current_segment() const { return current_; }
    const Diagnostics& diagnostics() const { return diagnostics_; }
    const Config& config() const { return config_; }
    const CollisionWorld& collision_world() const { return *collision_; }
    const std::string& trajectory_signature() const { return signature_; }
    Generator fork() const { return *this; } // lightweight cursor checkpoint
    void reset();
    size_t queued_segments() const { return pending_.size(); }
private:
    std::shared_ptr<const World> world_;
    std::shared_ptr<const CollisionWorld> collision_;
    Config config_;
    std::string signature_;
    Segment current_;
    std::deque<Segment> pending_;
    std::deque<Segment> history_;
    Diagnostics diagnostics_;
    uint64_t next_event_=0,mower_pass_=0;
    size_t mower_lane_=0;
    std::vector<std::pair<Vec3,Vec3>> mower_lanes_;
    Vec3 hand_anchor_;
    bool failed_=false,rectilinear_done_=false;
    void next_segment();
    bool plan_event(State);
    bool plan_retreat(State,uint64_t);
    bool plan_rectilinear(State,uint64_t);
    bool plan_circular(State,uint64_t);
    bool plan_handheld(State,uint64_t,bool headset);
    bool plan_controller(State,uint64_t);
    bool plan_mower(State,uint64_t);
    bool append_route(State,const std::vector<Vec3>&,uint64_t,const std::string&,bool align,double speed_cap=0);
    std::vector<Vec3> navigation(Vec3 start,Vec3 goal) const;
    void make_mower_lanes(double z);
    Segment make_bezier(State,const std::array<Vec3,4>&,const Mat3&,double,uint64_t,const std::string&,double speed_cap=0) const;
    Segment make_arc(State,Vec3,double,double,double,double,uint64_t) const;
    Segment make_hold(State,double,uint64_t,const std::string&) const;
    double draw(uint64_t event,uint64_t stream,uint64_t index,double lo=0,double hi=1) const;
    Mat3 look(Vec3) const;
};

// Exact geometric/kinematic primitive serialization. Saving a tape freezes the
// accepted plan independently of future planner versions (still floating math).
void write_segment_header(std::ostream&);
void write_segment(std::ostream&,const Segment&);
Segment read_segment(const std::string& csv_line);
// Human-readable resolved configuration and canonical geometry fingerprint.
void write_config(std::ostream&,const Config&);
} // namespace long_motion
