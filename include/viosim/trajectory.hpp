#pragma once
#include "math.hpp"
#include "config.hpp"
#include <iosfwd>
#include <memory>
#include <string>
namespace vio {
struct State { Vec3 p,v,a,omega; Mat3 R; };
Jet motion_clock(double t,const Config&);
State state_at(double t,const Config&,Vec3 anchor);
inline Vec3 specific_force(const State& s,double gravity=9.81) { return s.R.transpose()*(s.a-Vec3{0,0,-gravity}); }

struct Scene;
struct Rig;
struct MotionDiagnostics {
    uint64_t events=0,candidates=0,rejected=0,blocked_holds=0,safe_retreats=0;
};

class TrajectoryCursor {
public:
    TrajectoryCursor(const Config&,const Scene&,const Rig&);
    ~TrajectoryCursor();
    TrajectoryCursor(TrajectoryCursor&&) noexcept;
    TrajectoryCursor& operator=(TrajectoryCursor&&) noexcept;
    State at(double seconds);
    bool continuously_certified() const;
    double current_segment_end() const;
    const std::string& algorithm() const;
    const std::string& signature() const;
    const std::string& resolved_config() const;
    size_t write_primitive_tape(std::ostream&,double duration);
    MotionDiagnostics diagnostics() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace vio
