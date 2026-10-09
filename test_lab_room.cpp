#include "viosim/lab_room.hpp"
#include "viosim/renderer.hpp"
#include "viosim/simulation.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

namespace {
int checks=0;
void check(bool condition,const char* message) {
    ++checks;
    if(!condition) throw std::runtime_error(message);
}
}

int main(int argc,char** argv) {
    using namespace vio;
    try {
        check(argc==2,"Expected texture directory argument");
        const std::vector<std::string> expected_presets={"office-controller","office-bias-validation","corridor-10m","warehouse-scanner","lab-room-scanner"};
        for(const auto& name:expected_presets)
            check(std::find(preset_names().begin(),preset_names().end(),name)!=preset_names().end(),"Missing VioSim preset");
        for(const auto& preset:benchmark_presets()) {
            Config config=preset.config;
            validate_config(config);
            const Scene scene=make_scene(config.scene);
            const Rig rig(config);
            preflight(config,scene,rig);
            TrajectoryCursor trajectory(config,scene,rig);
            const auto initial=trajectory.at(0);
            if(preset.category=="initialization")
                check(norm(initial.v)>.01 || norm(initial.omega)>.01,"Moving start must actually move");
            if(preset.name=="test-reference") {
                const auto still=trajectory.at(1.999);
                check(norm(still.p-initial.p)<1e-12 && norm(still.omega)<1e-12,"Reference starts with two static seconds");
            }
            if(preset.name=="test-quad-1000hz") {
                check(config.camera_count==4,"Quad benchmark must have four cameras");
                check(config.width==640 && config.height==480 && config.alpha>.60 && config.beta>1.08,
                      "Quad benchmark must use the representative SEUCM configuration");
                check(config.camera_extrinsics[0].translation==std::array<double,3>{-.02,.01,.01} &&
                      config.camera_extrinsics[1].translation==std::array<double,3>{ .11,.01,.01} &&
                      config.camera_extrinsics[2].translation==std::array<double,3>{-.04,.01,-.01} &&
                      config.camera_extrinsics[3].translation==std::array<double,3>{ .14,.01,-.01},
                      "Quad benchmark must use the representative four-camera layout");
                const std::array<double,4> expected_yaw_degrees{{-10.,10.,-70.,70.}};
                for(std::size_t camera=0;camera<config.camera_extrinsics.size();++camera)
                    check(config.camera_extrinsics[camera].rotation==ry(radians(expected_yaw_degrees[camera])).a,
                          "Quad benchmark rotations must use whole yaw degrees");
                const Pose camera0=rig.camera_pose(initial.p,initial.R,0);
                check(norm(camera0.R*Vec3{0,0,1}-Vec3{0,1,0})<1e-8 &&
                      norm(camera0.R*Vec3{0,-1,0}-Vec3{0,0,1})<1e-8,
                      "Quad benchmark must align camera zero with the reference scene axes during warmup");
            }
        }
        for(const auto& preset:expected_presets) {
            Config config=preset_config(preset);
            validate_config(config);
            check(config.trajectory=="random_long","VioSim preset must use the trajectory generator");
        }
        Config injected=preset_config("test-bias-known");
        injected.renderer="cpu"; injected.width=32; injected.height=24; injected.duration=.02;
        Simulation simulation(injected,argv[0]);
        const auto sample=simulation.next_imu();
        check(norm(sample.gyro-Vec3{.005,-.003,.002})<1e-12,"Explicit gyro bias in ideal mode");
        check(norm(sample.accel-specific_force(sample.ideal_state,injected.gravity)-Vec3{.08,-.06,.04})<1e-12,
              "Explicit accelerometer bias and gravity in ideal mode");
        const Config corridor_config=preset_config("corridor-10m");
        check(corridor_config.warmup==2 && corridor_config.motion_fixed_distance==10 &&
              corridor_config.motion_final_hold==2 && corridor_config.motion_move_duration==5,
              "Corridor preset timing and distance");
        check(corridor_config.gyro_noise_density==0 && corridor_config.accel_noise_density==0 &&
              corridor_config.gyro_bias_rw==0 && corridor_config.accel_bias_rw==0,
              "Corridor preset must use an ideal IMU");
        const Config bias_config=preset_config("office-bias-validation");
        check(bias_config.duration==30 && bias_config.warmup==2 && bias_config.difficulty=="ideal",
              "Bias-validation preset timing and rendering");
        check(bias_config.gyro_noise_density==0 && bias_config.accel_noise_density==0 &&
              bias_config.gyro_bias_rw==0 && bias_config.accel_bias_rw==0,
              "Bias-validation preset must use an ideal IMU");
        const Scene bias_scene=make_scene(bias_config.scene);
        const TrajectoryStats bias_stats=preflight(bias_config,bias_scene,Rig(bias_config));
        check(bias_stats.distance>5 && bias_stats.max_angular_speed>.1,
              "Bias-validation trajectory must excite translation and rotation");
        const Scene corridor=make_scene("corridor");
        TrajectoryCursor corridor_trajectory(corridor_config,corridor,Rig(corridor_config));
        const State start=corridor_trajectory.at(0);
        const State initial=corridor_trajectory.at(1.999);
        check(norm(initial.p-start.p)<1e-9,"Corridor trajectory must start with a static phase");
        corridor_trajectory.at(2);
        const double move_end=corridor_trajectory.current_segment_end();
        const State final_position=corridor_trajectory.at(move_end);
        check(norm(final_position.p-(initial.p+Vec3{0,10,0}))<1e-9,
              "Corridor trajectory must move 10m forward");
        const double final_hold_end=corridor_trajectory.current_segment_end();
        check(std::abs(final_hold_end-move_end-2)<1e-9,
              "Corridor trajectory must end with a two-second static phase");
        check(norm(corridor_trajectory.at(final_hold_end).p-final_position.p)<1e-9,
              "Corridor trajectory must remain static after the move");
        bool unknown_preset_rejected=false;
        try { static_cast<void>(preset_config("unknown")); } catch(const std::invalid_argument&) { unknown_preset_rejected=true; }
        check(unknown_preset_rejected,"Unknown VioSim preset must be rejected");
        Config projection;
        const Camera seucm(projection,false);
        projection.camera_model="ucm";
        const Camera ucm(projection,false);
        for(int y=0;y<400;y+=13) for(int x=0;x<640;x+=17) {
            check(norm(seucm.unproject(x,y)-ucm.unproject(x,y))<1e-12,"Default SEUCM/UCM ray equivalence");
        }
        for(const auto& model:{"ucm","seucm"}) for(double alpha:{.2,.44,.6}) {
            projection.camera_model=model;projection.alpha=alpha;projection.beta=.8;
            const Camera camera(projection,false);
            for(int y=0;y<400;y+=13) for(int x=0;x<640;x+=17) {
                const auto pixel=camera.project(camera.unproject(x,y));
                check(std::hypot(pixel[0]-x,pixel[1]-y)<1e-9,"Camera projection round trip");
            }
        }
        const Scene scene=make_scene("lab_room");
        check(scene.boxes.size()==38,"Reference scene must have 38 boxes");
        check(6*(scene.boxes.size()+1)<=255,"Surface IDs must fit in uint8");
        check(scene.safe(scene.anchor,.10),"IMU anchor must be collision-free");
        for(double y=.50;y<=7.10;y+=.11) for(double x=-1.4;x<=1.4;x+=.13) {
            const Hit hit=scene.trace({x,y,.30},{0,0,-1});
            check(hit.object==0 && hit.face==4,"Central aisle must contain no object");
            check(scene.material(hit).texture==LAB_FLOOR,"Central aisle must use the plain floor");
        }

        Config config; config.scene="lab_room";config.difficulty="ideal";config.duration=20;
        validate_config(config);
        for(const auto& trajectory:trajectory_names()) {
            config.trajectory=trajectory;
            preflight(config,scene,Rig(config));
        }
        check(std::find(scene_names().begin(),scene_names().end(),"lab_room")!=scene_names().end(),"Scene registry");
        set_option(config,"lab-vignette","0.4");set_option(config,"lab-reflections","false");
        check(config.lab_vignette==.4 && !config.lab_reflections,"Configuration parser");
        const std::string text=dump_config(config);
        check(text.find("lab_reflections: false")!=std::string::npos,"Reflection export");
        check(text.find("lab_vignette:")!=std::string::npos,"Vignette export");
        bool rejected=false;
        try { Config bad=config;set_option(bad,"lab-vignette","1.1");validate_config(bad); }
        catch(const std::invalid_argument&) { rejected=true; }
        check(rejected,"Invalid vignette must be rejected");

        config.width=96;config.height=64;config.samples=1;config.sensor_blur=0;
        const Camera camera(config);const TextureBank textures(argv[1],true);
        check(textures.textures.size()==size_t(LAB_OPENING)+1,"Texture registry order");
        Hit box_hit;box_hit.object=1;box_hit.face=1;box_hit.point={-2.87,1.3,1};
        check(scene.material(box_hit).texture==LAB_LOCKERS_A,"Locker front texture");
        box_hit.face=0;
        check(scene.material(box_hit).texture==LAB_METAL,"Locker side fallback texture");
        for(double strength:{0.,.4,.95}) for(int y=0;y<camera.height;++y) for(int x=0;x<camera.width;++x) {
            const double gain=lab_room_vignette(camera,x,y,strength);
            check(gain>=.05 && gain<=1,"Vignette bounds");
            if(strength==0) check(gain==1,"Zero vignette identity");
        }

        const Pose pose=Rig(config).camera_pose({0,1.1,1.65},rx(radians(-17)),0);
        config.lab_vignette=0;config.lab_reflections=false;
        const Image8 diffuse=render(camera,scene,textures,pose,config,1234);
        config.lab_reflections=true;
        const Image8 reflected=render(camera,scene,textures,pose,config,1234);
        check(diffuse.data!=reflected.data,"Reflection switch must alter lab image");
        const GeometryImage geometry=render_geometry(camera,scene,pose);
        int changed_floor=0;
        for(size_t i=0;i<diffuse.data.size();++i) {
            const int id=geometry.surface_id.data[i];
            if(id==5 && diffuse.data[i]!=reflected.data[i]) ++changed_floor;
            if(id!=5) check(diffuse.data[i]==reflected.data[i],"Reflections must only alter the floor");
        }
        check(changed_floor>0,"Floor highlights must be visible");

        const Scene office=make_scene("office");
        const Pose office_pose=Rig(config).camera_pose(office.anchor,Mat3{},0);
        config.lab_reflections=false;config.lab_vignette=0;
        const Image8 office_before=render(camera,office,textures,office_pose,config,1234);
        config.lab_reflections=true;config.lab_vignette=.95;
        const Image8 office_after=render(camera,office,textures,office_pose,config,1234);
        check(office_before.data==office_after.data,"Lab controls must not alter existing scenes");
        std::cout<<"PASS: "<<checks<<" lab-room checks\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL after "<<checks<<" checks: "<<error.what()<<'\n';
        return 1;
    }
}
