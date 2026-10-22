#include "viosim/lab_room.hpp"
#include "viosim/renderer.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    using namespace vio;
    try {
        if(argc<2 || argc>3) {
            std::cout<<"Usage: viosim_lab_preview OUTPUT_DIR [TEXTURE_DIR]\n";
            return argc<2?0:1;
        }
        const std::filesystem::path output=argv[1];
        if(std::filesystem::exists(output) && (!std::filesystem::is_directory(output) || !std::filesystem::is_empty(output)))
            throw std::runtime_error("Output must be a new or empty directory");
        Config config;config.scene="lab_room";config.difficulty="medium";config.samples=2;
        if(argc==3) config.assets=argv[2];
        validate_config(config);
        const auto asset_dir=locate_assets(config,argv[0]);
        const Scene scene=make_scene(config.scene);const Camera camera(config);const Rig rig(config);
        const TextureBank textures(asset_dir,true);
        const std::array<Vec3,3> positions={{{0,1.10,1.65},{.42,2.1,1.65},{-.38,2.9,1.65}}};
        const std::array<double,3> yaws={{0,7,-7}};
        std::filesystem::create_directories(output);
        std::ofstream poses(output/"inspection_poses.csv");poses.exceptions(std::ios::badbit|std::ios::failbit);
        poses<<std::setprecision(17)<<"view,camera,px,py,pz,qw,qx,qy,qz\n";
        Image8 montage{camera.width*2,camera.height*3,std::vector<uint8_t>(size_t(camera.width)*2*camera.height*3)};
        for(size_t view=0;view<positions.size();++view) {
            const Mat3 body_rotation=rz(radians(-yaws[view]))*rx(radians(-17))*imu_basis_rotation(config);
            for(int camera_index=0;camera_index<2;++camera_index) {
                const Pose pose=rig.camera_pose(positions[view],body_rotation,size_t(camera_index));
                if(!scene.safe(pose.p,.10)) throw std::runtime_error("Unsafe inspection viewpoint");
                const Image8 image=render(camera,scene,textures,pose,config,1234+view*2+size_t(camera_index));
                const std::string stem="view_"+std::to_string(view)+"_cam"+std::to_string(camera_index);
                write_png(output/(stem+".png"),image,3);
                const auto q=quaternion(pose.R);
                poses<<view<<','<<camera_index<<','<<pose.p.x<<','<<pose.p.y<<','<<pose.p.z;
                for(double value:q) poses<<','<<value;
                poses<<'\n';
                for(int y=0;y<camera.height;++y)
                    std::copy_n(image.data.data()+size_t(y)*camera.width,camera.width,
                        montage.data.data()+(view*camera.height+y)*montage.width+camera_index*camera.width);
            }
        }
        write_png(output/"stereo_inspection.png",montage,3);
        std::ofstream saved_config(output/"inspection_config.yaml");
        saved_config.exceptions(std::ios::badbit|std::ios::failbit);
        saved_config<<dump_config(config)
            <<"# Inspection-only body views: z=1.65 m, downward angle=17 deg.\n"
            <<"# Full camera poses are in inspection_poses.csv. This is not a timed dataset.\n";
        std::cout<<"Rendered three stereo inspection views in "<<output<<'\n';
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';
        return 1;
    }
}
