#include "viosim/dataset.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
static void help() {
    std::cout<<R"(viosim_cpp 1.0.0 - multi-camera inertial sequence generator

Commands:
  viosim generate [--config FILE] [options]
  viosim list-scenes
  viosim list-trajectories
  viosim list-presets
  viosim list-benchmarks
  viosim config --preset NAME [options]
  viosim --version

Scene / motion:
  --preset NAME                Start from a named VioSim configuration
  --scene office|office_tag|office_tag_near|corridor|warehouse|lab_room
          multilevel|multilevel_tables|flat_tables
  --trajectory figure8|ellipse|back_and_forth|slow_forward|straight|pure_rotation|static|random_long
               multilevel_walk|tables_walk
  --motion-mode rectilinear|circular|scanner|xr_headset|controller|mower
  --motion-seed UINT64          Seed independent from sensor/image noise
  --motion-config FILE          Dedicated flat YAML motion configuration
  --motion-reference-camera N  Camera used by outward circles (-1: virtual center)
  --difficulty ideal|easy|medium|hard
  --duration SECONDS            Recording length (default 8)
  --period SECONDS              Cycle period once the speed ramp ends (default 8)
  --size SCALE                  Spatial amplitude multiplier (default 1)
  --warmup SECONDS              Truly static initial section (default 1)
  --ramp SECONDS                Smooth speed ramp after warmup (default 1)
  --cooldown SECONDS            Minimum final static section for straight motion
  --speed-scale SCALE           Motion clock multiplier (default 1)
  --roll-deg D --pitch-deg D --yaw-deg D --yaw-offset-deg D
  --clearance METERS            Sampled trajectory/scene clearance (default 0.10)

Sensors:
  --width 640 --height 400 --camera-hz 50 --imu-hz 500
  --camera-model seucm|ucm      Projection model (default seucm)
  --alpha VALUE --beta VALUE    SEUCM parameters (defaults 4/9 and 1)
  --baseline 0.08 --fov-deg 120 --xi 0.8
  --right-yaw-deg 1 --right-roll-deg 0.6
  --camera-count 2|4           Enable explicit rig extrinsics (normally from YAML)
  --camN-rotation-bc [9 values] --camN-translation-bc [3 values]
  --image-noise SIGMA --extra-blur SIGMA --sensor-blur SIGMA
  --brightness FACTOR --contrast FACTOR
  --gyro-noise-density V --accel-noise-density V
  --gyro-bias-rw V --accel-bias-rw V
  --initial-gyro-bias [x,y,z] --initial-accel-bias [x,y,z]
  --gravity VALUE              Magnitude in m/s² (default 9.81)
  --imu-frame camera|legacy_z_up  IMU axes: camera-aligned by default; legacy Z-up opt-in
  --motion-time-offset SECONDS --motion-time-scale FACTOR (random_long)
  --blackout-start SECONDS --blackout-duration SECONDS
  --truth-output DIRECTORY     Live device ground truth export (no images)
  --reported-focal-scale FACTOR --reported-baseline-scale FACTOR (live only)
  --lab-reflections true|false  Approximate floor highlights (lab_room only)
  --lab-vignette 0..0.95        Smooth radial attenuation (lab_room only)
  --seed 1234 --start-ns 1700000000000000000

Rendering / export:
  --renderer cpu|gpu|auto       Default: auto (hardware GPU, otherwise CPU); OpenGL 4.3
  --gpu-precompute-rays true|false  Cache camera rays (default true)
  --gpu-postprocess true|false  GPU blur/noise and 8-bit readback (default true)
  --profile-render true|false   Measure GPU and host rendering stages
  --threads N                  Parallel camera-group tasks (default 4)
  --samples N                  N x N rays per final pixel (default 2; max 4)
  --texture-filter nearest|bilinear|mipmap  (default mipmap)
  --png-compression 0..9        Default 1
  --format png|pgm              PGM avoids PNG compression, but is larger
  --output DIRECTORY           Must not contain existing files
  --assets TEXTURE_DIRECTORY   Optional explicit texture location
  --preview-count N            Stereo strips in debug/preview.png (default 5)
  --depth-every N               Save center-ray range/IDs every N frames (0=off)
  --distance-shading true|false Legacy view-dependent attenuation (default false)
  --dry-run                    Check configuration and trajectory without writing

CLI values override the flat scalar YAML configuration. Existing output is never
recursively deleted. All timestamps are integer nanoseconds. See README.md.
)";
}
int main(int argc,char** argv) {
    try {
        if(argc<2) { help(); return 0; }
        const std::string command=argv[1];
        if(command=="--help" || command=="-h" || command=="help") { help(); return 0; }
        if(command=="--version") { std::cout<<"viosim_cpp 1.0.0\n"; return 0; }
        if(command=="list-scenes" || command=="list-trajectories" || command=="list-presets") {
            const auto& names=command=="list-scenes" ? vio::scene_names()
                             : command=="list-trajectories" ? vio::trajectory_names()
                                                              : vio::preset_names();
            for(const auto& name:names) std::cout<<name<<'\n';
            return 0;
        }
        if(command=="list-benchmarks") {
            for(const auto& p:vio::benchmark_presets())
                std::cout<<p.name<<'\t'<<p.category<<'\t'<<p.description<<'\n';
            return 0;
        }
        if(command!="generate" && command!="config") throw std::invalid_argument("Unknown command: "+command);
        std::string preset;
        std::vector<std::string> config_paths;
        std::vector<std::pair<std::string,std::string>> overrides;
        for(int i=2;i<argc;++i) {
            std::string key=argv[i],value;
            if(key=="--help" || key=="-h") { help(); return 0; }
            if(key.rfind("--",0)!=0) throw std::invalid_argument("Expected an option, got "+key);
            key=key.substr(2); const auto equal=key.find('=');
            if(equal!=std::string::npos) { value=key.substr(equal+1); key=key.substr(0,equal); }
            else if(key=="dry-run") value="true";
            else { if(i+1>=argc || std::string(argv[i+1]).rfind("--",0)==0) throw std::invalid_argument("Missing value for --"+key); value=argv[++i]; }
            if(key=="config") config_paths.push_back(value);
            else if(key=="preset") preset=value;
            else overrides.emplace_back(key,value);
        }
        vio::Config cfg=preset.empty() ? vio::Config{} : vio::preset_config(preset);
        for(const auto& path:config_paths) vio::load_config(cfg,path);
        for(const auto& item:overrides) vio::set_option(cfg,item.first,item.second);
        if(command=="config") { vio::validate_config(cfg); std::cout<<vio::dump_config(cfg); }
        else vio::generate(cfg,argv[0]);
        return 0;
    } catch(const std::exception& e) { std::cerr<<"Error: "<<e.what()<<'\n'; return 1; }
}
