#include <viosim/gpu_renderer.hpp>
#include <viosim/scene.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    using namespace vio;
    try {
        if(argc!=2 && argc!=3) throw std::runtime_error("Expected texture directory [calib_fisheyes.txt or rig.yaml]");
        const auto scene=make_scene("warehouse");
        const TextureBank textures(argv[1]);
        int checks=0;
        for(const auto model:{"ucm","seucm"}) for(int samples:{1,2})
            for(const auto filter:{"nearest","bilinear","mipmap"}) {
                Config config;config.camera_model=model;config.alpha=.6;config.beta=.8;
                config.width=96;config.height=64;config.samples=samples;config.texture_filter=filter;
                config.difficulty="ideal";validate_config(config);
                const Camera camera(config,false);const Rig rig(config);
                config.camera_count=6;
                config.gpu_precompute_rays=false;
                const GpuRenderer reference(camera,scene,textures,config);
                config.gpu_precompute_rays=true;
                const GpuRenderer cached(camera,scene,textures,config);
                // Each renderer switches its own context: this also checks that
                // the table remains correctly bound when multiple renderers exist.
                for(int step=0;step<3;++step) {
                    std::vector<Pose> poses;
                    for(int k=0;k<6;++k)
                        poses.push_back(rig.camera_pose(scene.anchor+Vec3{.03*step,.06*step,0},rz(.1*k)*imu_basis_rotation(config),k%2));
                    const std::vector<uint64_t> seeds{1,2,3,4,5,6};
                    const auto a=reference.render(poses,seeds),b=cached.render(poses,seeds);
                    for(size_t k=0;k<a.size();++k) {
                        if(a[k].data!=b[k].data) throw std::runtime_error(std::string("Ray cache changed pixels: ")+model+" "+filter+" samples="+std::to_string(samples));
                        ++checks;
                    }
                }
            }
        std::cout<<"PASS "<<checks<<" GPU images identical with/without ray cache\n";
        if(argc==3) for(int samples:{1,2}) {
            Config config;
            if(std::filesystem::path(argv[2]).extension()==".yaml") load_config(config,argv[2]);
            else load_calibration(config,argv[2]);
            config.samples=samples;config.difficulty="ideal";config.sensor_blur=0;
            config.profile_render=true;validate_config(config);
            std::vector<Camera> cameras;
            for(const auto& calibration:config.camera_extrinsics) {
                cameras.emplace_back(config,calibration,false);
                // Exercise independent tables even for rigs with shared optics.
                if(!calibration.intrinsics_set) cameras.back().fx*=1+.01*cameras.size();
            }
            const Rig rig(config);
            std::vector<Pose> poses;
            for(size_t k=0;k<rig.size();++k) poses.push_back(rig.camera_pose(scene.anchor,imu_basis_rotation(config),k));
            std::vector<uint64_t> seeds;
            for(size_t k=0;k<rig.size();++k) seeds.push_back(k+1);
            config.gpu_precompute_rays=false;const GpuRenderer reference(cameras,scene,textures,config);
            config.gpu_precompute_rays=true;const GpuRenderer cached(cameras,scene,textures,config);
            const auto a=reference.render(poses,seeds),b=cached.render(poses,seeds);
            bool identical=true;
            for(size_t k=0;k<a.size();++k) {
                size_t changed=0;int maximum=0;
                for(size_t p=0;p<a[k].data.size();++p) {
                    const int delta=std::abs(int(a[k].data[p])-int(b[k].data[p]));
                    changed+=delta!=0;maximum=std::max(maximum,delta);
                }
                std::cout<<"Calibrated cache samples="<<samples<<" camera="<<k<<" changed="<<changed<<" max_delta="<<maximum<<'\n';
                identical=identical && changed==0;
            }
            if(!identical) throw std::runtime_error("Per-camera ray cache changed pixels");
            const std::vector<size_t> selection{rig.size()-1,0};
            const auto selected=cached.render({poses.back(),poses.front()},{seeds.back(),seeds.front()},selection);
            if(selected[0].data!=b.back().data || selected[1].data!=b.front().data)
                throw std::runtime_error("Per-camera ray cache lost selected calibration indices");
            std::cout<<"PASS calibrated rig samples="<<samples<<cached.profile()<<'\n';
        }
        for(double blur:{0.,.55,8.}) for(const char* difficulty:{"ideal","medium","hard"}) {
            Config config;config.width=97;config.height=65;config.samples=1;
            config.difficulty=difficulty;config.extra_blur=blur;config.sensor_blur=0;
            validate_config(config);
            const Camera camera(config,false);const Rig rig(config);config.camera_count=6;
            config.gpu_postprocess=false;const GpuRenderer cpu(camera,scene,textures,config);
            config.gpu_postprocess=true;const GpuRenderer gpu(camera,scene,textures,config);
            std::vector<Pose> poses;
            for(int k=0;k<6;++k) poses.push_back(rig.camera_pose(scene.anchor,rz(.1*k)*imu_basis_rotation(config),k%2));
            const std::vector<uint64_t> seeds{0,1234,0xffffffffffffffffULL,0x12345678abcdef01ULL,5,6};
            const auto a=cpu.render(poses,seeds),b=gpu.render(poses,seeds),repeat=gpu.render(poses,seeds);
            size_t different=0,pixels=0;int maximum=0;
            for(size_t k=0;k<a.size();++k) {
                if(b[k].data!=repeat[k].data) throw std::runtime_error("GPU noise is not reproducible");
                for(size_t p=0;p<a[k].data.size();++p) {
                    const int delta=std::abs(int(a[k].data[p])-int(b[k].data[p]));
                    maximum=std::max(maximum,delta);different+=delta!=0;++pixels;
                }
            }
            if(maximum>1 || different>pixels/100)
                throw std::runtime_error("GPU postprocessing differs from CPU: max="+std::to_string(maximum)+" pixels="+std::to_string(different));
            std::cout<<"PASS postprocess "<<difficulty<<" blur="<<blur<<" max_delta="<<maximum<<" changed="<<different<<'/'<<pixels<<'\n';
        }
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
