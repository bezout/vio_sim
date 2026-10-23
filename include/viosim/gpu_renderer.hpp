#pragma once

#include "renderer.hpp"

#include <memory>
#include <string>
#include <vector>

namespace vio {

class GpuRenderer {
public:
    GpuRenderer(const Camera&,const Scene&,const TextureBank&,const Config&);
    GpuRenderer(const std::vector<Camera>&,const Scene&,const TextureBank&,const Config&);
    ~GpuRenderer();
    GpuRenderer(const GpuRenderer&)=delete;
    GpuRenderer& operator=(const GpuRenderer&)=delete;

    std::vector<Image8> render(const std::vector<Pose>&,const std::vector<uint64_t>&,
                             const std::vector<size_t>& cameras={}) const;
    const std::string& device_name() const;
    std::string profile() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace vio
