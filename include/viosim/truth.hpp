#pragma once
#include "simulation.hpp"
#include <fstream>

namespace vio {
// Optional live export, on the device's output clock. No rendered images and
// no second simulation: the samples include the actual injected random walk.
class LiveTruthWriter {
public:
    explicit LiveTruthWriter(const Simulation&);
    void imu(const ImuSample&,int64_t output_timestamp_us);
    void finish();
private:
    std::ofstream poses_;
    std::ofstream floors_;
    const Scene* scene_=nullptr;
};
}
