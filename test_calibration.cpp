#include "viosim/simulation.hpp"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition,const char* message) {
    if(!condition) throw std::runtime_error(message);
}

void test_configuration_is_validated_after_loading() {
    namespace fs=std::filesystem;
    struct TemporaryDirectory {
        fs::path path=fs::temp_directory_path()/(
            "viosim_calibration_order_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        TemporaryDirectory() { require(fs::create_directory(path),"Cannot create calibration test directory"); }
        ~TemporaryDirectory() { std::error_code error;fs::remove_all(path,error); }
    } directory;
    {
        std::ofstream calibration(directory.path/"rig.txt");
        calibration<<"2\n";
        for(int camera=0;camera<2;++camera)
            calibration<<"R= 1 0 0 0 1 0 0 0 1\nT= "<<.08*camera<<" 0 0\n"
                         "SEUCM 640 480 300 300 319.5 239.5 319.5 239.5 .6 1.1\n";
    }
    const auto write_config=[&](const char* contents) {
        std::ofstream file(directory.path/"test.yaml");file<<contents;
        require(bool(file),"Cannot write calibration configuration");
    };
    const auto read_config=[&]() {
        vio::Config config;vio::load_config(config,directory.path/"test.yaml");
        require(config.image_noise==-1,"Loading resolved unrelated noise defaults");
        return config;
    };
    write_config("trajectory: straight\ncalibration: rig.txt\nduration: 20\n");
    auto first=read_config();vio::validate_config(first);
    write_config("duration: 20\ncalibration: rig.txt\ntrajectory: straight\n");
    auto second=read_config();vio::validate_config(second);
    require(vio::dump_config(first)==vio::dump_config(second),"Calibration depends on YAML key order");

    // The CLI applies overrides after all YAML files have been loaded.
    write_config("trajectory: straight\nduration: 1\ncalibration: rig.txt\n");
    auto pending=read_config();auto invalid=pending;
    bool rejected=false;
    try { vio::validate_config(invalid); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"Final validation accepted an invalid straight-motion duration");
    vio::set_option(pending,"duration","20");vio::validate_config(pending);
    require(vio::dump_config(pending)==vio::dump_config(first),"CLI override did not repair the pending configuration");

    pending.camera_extrinsics[0].intrinsics[0]=-1.;
    rejected=false;
    try { vio::validate_config(pending); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"Final validation accepted invalid calibration intrinsics");
}
}

int main(int argc,char** argv) {
    try {
        if(argc<3) throw std::runtime_error("Expected B50 calibration and texture directory [gpu]");
        test_configuration_is_validated_after_loading();
        vio::Config config;
        vio::load_calibration(config,argv[1]);
        require(config.camera_count==4 && config.width==640 && config.height==480,"B50 import lost cameras or resolution");
        require(config.image_noise==-1,"Calibration import resolved noise defaults prematurely");
        config.assets=argv[2];config.renderer=argc>3?"gpu":"cpu";
        config.scene="corridor";config.trajectory="static";config.samples=1;
        config.difficulty="ideal";config.sensor_blur=0;config.duration=.1;
        vio::Simulation simulation(config,argv[0]);

        // Independent reference: n::SEUCM::ray({0,0}) in fast_slam/libn.
        const vio::Vec3 references[]={
            {-.77612318990205187,-.61977176291704550,.11625728358671245},
            {-.78055726340151976,-.61396086363769575,.11739853692608498},
            {-.78281420701183002,-.60495618651855676,.14570494052490066},
            {-.77672044025218501,-.61690100706542905,.12703741644142222}};
        size_t checked=0;
        for(size_t index=0;index<4;++index) {
            const auto& camera=simulation.camera(index);
            require(vio::norm(camera.unproject(0,0)-references[index])<1e-14,"SEUCM disagrees with libn reference");
            require(camera.fx==config.camera_extrinsics[index].intrinsics[0],"Camera intrinsics were shared");
            require(simulation.rig().BC[index].R.a==config.camera_extrinsics[index].rotation,"Import changed rotation_BC");
            for(int v=0;v<480;v+=13) for(int u=0;u<640;u+=17) {
                const auto pixel=camera.project(camera.unproject(u,v));
                require(std::hypot(pixel[0]-u,pixel[1]-v)<1e-9,"SEUCM projection round trip failed");
                ++checked;
            }
        }
        auto full=simulation.render_camera_group(simulation.prepare_camera_group(0));
        auto selected=simulation.render_camera_group(simulation.prepare_camera_group(0,{3,0}));
        require(selected.images[0].data==full.images[3].data && selected.images[1].data==full.images[0].data,
                "Selected cameras lost physical calibration or image seeds");
        require(full.images[0].data!=full.images[1].data,"Distinct camera images unexpectedly identical");
        std::cout<<"PASS B50: "<<checked<<" round trips, libn rays, extrinsics, selected cameras; renderer="
                 <<simulation.renderer_backend()<<'\n';
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
