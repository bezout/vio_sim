#include "viosim/config.hpp"
#include "viosim/math.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace vio {
const std::vector<std::string>& scene_names() { static const std::vector<std::string> v={"office","office_tag","office_tag_near","corridor","warehouse","lab_room","multilevel","multilevel_tables","flat_tables"}; return v; }
const std::vector<std::string>& trajectory_names() { static const std::vector<std::string> v={"figure8","figure8_forward","ellipse","back_and_forth","slow_forward","straight","pure_rotation","static","random_long","multilevel_walk","tables_walk"}; return v; }
const std::vector<std::string>& preset_names() {
    static const auto names=[] {
        std::vector<std::string> result={"office-controller","office-bias-validation","corridor-10m","warehouse-scanner","lab-room-scanner"};
        for(const auto& preset:benchmark_presets()) result.push_back(preset.name);
        return result;
    }();
    return names;
}
static Config generator_preset(const std::string& scene,const std::string& motion_mode,const std::string& difficulty) {
    Config config;
    config.scene=scene;
    config.trajectory="random_long";
    config.motion_mode=motion_mode;
    config.difficulty=difficulty;
    config.duration=120;
    config.samples=1;
    config.threads=4;
    config.preview_count=0;
    config.depth_every=0;
    return config;
}
Config preset_config(const std::string& name) {
    if(name=="office-controller") return generator_preset("office","controller","easy");
    if(name=="office-bias-validation") {
        Config config=generator_preset("office","controller","ideal");
        config.duration=30;
        config.warmup=2;
        config.gyro_noise_density=0;
        config.accel_noise_density=0;
        config.gyro_bias_rw=0;
        config.accel_bias_rw=0;
        return config;
    }
    if(name=="corridor-10m") {
        Config config=generator_preset("corridor","rectilinear","easy");
        config.duration=9.00001;
        config.warmup=2;
        config.motion_fixed_distance=10;
        config.motion_final_hold=2;
        config.motion_move_duration=5;
        config.gyro_noise_density=0;
        config.accel_noise_density=0;
        config.gyro_bias_rw=0;
        config.accel_bias_rw=0;
        return config;
    }
    if(name=="warehouse-scanner") return generator_preset("warehouse","scanner","medium");
    if(name=="lab-room-scanner") return generator_preset("lab_room","scanner","easy");
    for(const auto& preset:benchmark_presets()) if(preset.name==name) return preset.config;
    throw std::invalid_argument("Unknown VioSim preset: "+name);
}
static std::string trim(std::string s) {
    auto a=s.find_first_not_of(" \t\r\n"),b=s.find_last_not_of(" \t\r\n");
    return a==std::string::npos ? "" : s.substr(a,b-a+1);
}
static double number(const std::string& v) {
    size_t n=0; double x=std::stod(v,&n);
    if(n!=v.size() || !std::isfinite(x)) throw std::invalid_argument("Not a finite number: "+v);
    return x;
}
static int integer(const std::string& v) {
    size_t n=0; long long x=std::stoll(v,&n);
    if(n!=v.size() || x<std::numeric_limits<int>::min() || x>std::numeric_limits<int>::max()) throw std::invalid_argument("Not an integer: "+v);
    return int(x);
}
static bool boolean(const std::string& value,const std::string& key) {
    if(value=="true" || value=="1") return true;
    if(value=="false" || value=="0") return false;
    throw std::invalid_argument("Expected true or false for "+key);
}
template<size_t N>
static std::array<double,N> array_value(const std::string& value,const std::string& key) {
    std::array<double,N> result{}; size_t count=0; const char* cursor=value.c_str();
    while(*cursor) {
        char* end=nullptr; const double x=std::strtod(cursor,&end);
        if(end!=cursor) {
            if(count>=N || !std::isfinite(x)) throw std::invalid_argument("Invalid "+key+" array");
            result[count++]=x; cursor=end;
        } else ++cursor;
    }
    if(count!=N) throw std::invalid_argument(key+" expects "+std::to_string(N)+" numbers");
    return result;
}
void set_option(Config& c,const std::string& key,const std::string& v) {
    std::string k=key; std::replace(k.begin(),k.end(),'-','_');
    if(k=="calibration") { load_calibration(c,v); return; }
#define S(x) if(k==#x) { c.x=v; return; }
#define D(x) if(k==#x) { c.x=number(v); return; }
#define I(x) if(k==#x) { c.x=integer(v); return; }
    S(truth_output) S(camera_model) S(imu_frame) S(scene) S(trajectory) S(difficulty) S(output) S(assets) S(texture_filter) S(format) S(motion_mode) S(motion_config) S(figure8_shape)
    D(blackout_start) D(blackout_duration) D(gravity) D(reported_focal_scale) D(reported_baseline_scale) D(reported_u0_offset) D(reported_v0_offset) D(motion_time_offset) D(motion_time_scale) D(duration) D(period) D(size) D(speed_scale) D(warmup) D(ramp) D(cooldown) D(roll_deg) D(pitch_deg) D(yaw_deg) D(yaw_offset_deg) D(yaw_turn_start) D(yaw_turn_duration) D(yaw_turn_deg) D(motion_fixed_distance) D(motion_final_hold) D(motion_move_duration)
    D(alpha) D(beta) D(camera_hz) D(imu_hz) D(baseline) D(fov_deg) D(xi) D(right_yaw_deg) D(right_roll_deg) D(clearance)
    D(figure8_height)
    D(image_noise) D(extra_blur) D(brightness) D(contrast) D(gyro_noise_density) D(accel_noise_density) D(gyro_bias_rw) D(accel_bias_rw) D(sensor_blur) D(lab_vignette)
    I(width) I(height) I(threads) I(samples) I(png_compression) I(preview_count) I(depth_every) I(motion_reference_camera)
#undef S
#undef D
#undef I
    if(k=="initial_gyro_bias" || k=="initial_accel_bias") {
        const auto bias=array_value<3>(v,k);
        if(k=="initial_gyro_bias") { c.initial_gyro_bias=bias; c.initial_gyro_bias_set=true; }
        else { c.initial_accel_bias=bias; c.initial_accel_bias_set=true; }
        return;
    }
    if(k=="renderer") { c.renderer=v; return; }
    if(k=="seed" || k=="motion_seed" || k=="start_ns") {
        size_t n=0;
        if(v.empty() || v[0]=='-') throw std::invalid_argument("Expected nonnegative integer: "+v);
        const auto x=std::stoull(v,&n);
        if(n!=v.size() || (k=="start_ns" && x>uint64_t(std::numeric_limits<int64_t>::max()))) throw std::invalid_argument("Integer out of range: "+v);
        if(k=="seed") c.seed=x;
        else if(k=="motion_seed") c.motion_seed=x;
        else c.start_ns=int64_t(x);
        return;
    }
    if(k=="camera_count") {
        c.camera_count=integer(v);
        if(c.camera_count<0 || c.camera_count>16) throw std::invalid_argument("camera_count is out of range");
        if(c.camera_extrinsics.size()>size_t(c.camera_count)) throw std::invalid_argument("camera_count is smaller than a configured camera index");
        c.camera_extrinsics.resize(size_t(c.camera_count));
        return;
    }
    if(k.rfind("cam",0)==0) {
        const auto separator=k.find('_',3);
        if(separator==std::string::npos) throw std::invalid_argument("Invalid camera option: "+key);
        const int index=integer(k.substr(3,separator-3));
        if(index<0 || index>=16) throw std::invalid_argument("Camera index is out of range: "+key);
        if(c.camera_count>0 && index>=c.camera_count) throw std::invalid_argument("Camera index exceeds camera_count: "+key);
        if(c.camera_extrinsics.size()<=size_t(index)) c.camera_extrinsics.resize(size_t(index)+1);
        auto& camera=c.camera_extrinsics[size_t(index)]; const auto field=k.substr(separator+1);
        if(field=="rotation_bc") { camera.rotation=array_value<9>(v,key); camera.rotation_set=true; return; }
        if(field=="translation_bc") { camera.translation=array_value<3>(v,key); camera.translation_set=true; return; }
        if(field=="intrinsics") { camera.intrinsics=array_value<8>(v,key); camera.intrinsics_set=true; return; }
        throw std::invalid_argument("Unknown camera field: "+key);
    }
    bool Config::*bool_option=nullptr;
    if(k=="dry_run") bool_option=&Config::dry_run;
    else if(k=="distance_shading") bool_option=&Config::distance_shading;
    else if(k=="lab_reflections") bool_option=&Config::lab_reflections;
    else if(k=="profile_render") bool_option=&Config::profile_render;
    else if(k=="gpu_precompute_rays") bool_option=&Config::gpu_precompute_rays;
    else if(k=="gpu_postprocess") bool_option=&Config::gpu_postprocess;
    if(bool_option) {
        c.*bool_option=boolean(v,key);
        return;
    }
    throw std::invalid_argument("Unknown option: "+key);
}
void load_calibration(Config& c,const std::filesystem::path& path) {
    std::ifstream input(path);
    if(!input) throw std::invalid_argument("Cannot open camera calibration: "+path.string());
    Config imported=c;
    int count=0;
    if(!(input>>count) || (count!=2 && count!=4 && count!=6))
        throw std::invalid_argument("Calibration must contain two, four or six SEUCM cameras");
    imported.camera_count=count;
    imported.camera_model="seucm";
    imported.camera_extrinsics.assign(size_t(count),CameraExtrinsic{});
    const auto expect=[&](const char* label) {
        std::string actual;
        if(!(input>>actual) || actual!=label)
            throw std::invalid_argument("Expected "+std::string(label)+" in "+path.string());
    };
    for(int index=0;index<count;++index) {
        auto& camera=imported.camera_extrinsics[size_t(index)];
        expect("R=");
        for(double& x:camera.rotation) if(!(input>>x)) throw std::invalid_argument("Invalid calibration rotation");
        expect("T=");
        for(double& x:camera.translation) if(!(input>>x)) throw std::invalid_argument("Invalid calibration translation");
        expect("SEUCM");
        int width=0,height=0;
        if(!(input>>width>>height)) throw std::invalid_argument("Invalid calibration resolution");
        if(index==0) { imported.width=width;imported.height=height; }
        else if(width!=imported.width || height!=imported.height)
            throw std::invalid_argument("All cameras must have the same resolution");
        for(double& x:camera.intrinsics) if(!(input>>x)) throw std::invalid_argument("Invalid SEUCM intrinsics");
        camera.rotation_set=camera.translation_set=camera.intrinsics_set=true;
    }
    std::string suffix;
    if(input>>suffix) {
        double shift=0;
        if(suffix!="IMU-Fisheye-shift-us" || !(input>>shift) || shift!=0 || (input>>suffix))
            throw std::invalid_argument("Calibration requires a zero IMU-Fisheye-shift-us and no extra fields");
    }
    // Validate the complete configuration after all files and CLI overrides
    // have been applied; unrelated options may still be incomplete here.
    c=std::move(imported);
}
void load_config(Config& c,const std::filesystem::path& path) {
    std::ifstream f(path); if(!f) throw std::runtime_error("Cannot read config: "+path.string());
    std::string line; size_t n=0; bool has_motion_config=false;
    while(std::getline(f,line)) {
        ++n; line=trim(line); if(line.empty() || line[0]=='#') continue;
        const auto colon=line.find(':');
        if(colon==std::string::npos) throw std::runtime_error("Expected key: value at config line "+std::to_string(n));
        auto key=trim(line.substr(0,colon)),value=trim(line.substr(colon+1));
        // Intentionally a flat scalar subset, not a general YAML parser.
        if(!value.empty() && (value[0]=='\'' || value[0]=='"')) {
            char quote=value[0]; auto end=value.find(quote,1);
            if(end==std::string::npos) throw std::runtime_error("Unclosed quoted config value");
            const auto rest=trim(value.substr(end+1));
            if(!rest.empty() && rest[0]!='#') throw std::runtime_error("Unexpected text after quoted config value");
            value=value.substr(1,end-1);
        } else { const auto hash=value.find('#'); if(hash!=std::string::npos) value=trim(value.substr(0,hash)); }
        try {
            std::replace(key.begin(),key.end(),'-','_');
            if(key=="calibration" && std::filesystem::path(value).is_relative())
                value=(path.parent_path()/value).lexically_normal().string();
            set_option(c,key,value);
            has_motion_config=has_motion_config || key=="motion_config";
        } catch(const std::exception& e) { throw std::runtime_error(path.string()+":"+std::to_string(n)+": "+e.what()); }
    }
    if(has_motion_config && !c.motion_config.empty()) {
        std::filesystem::path motion_path=c.motion_config;
        if(motion_path.is_relative()) c.motion_config=(path.parent_path()/motion_path).lexically_normal().string();
    }
}
int64_t sample_count(double duration,double rate) { return int64_t(std::ceil(duration*rate-1e-9)); }
int64_t timestamp(int64_t start,int64_t i,double rate) { return start+std::llround(double(i)*1e9/rate); }
void validate_config(Config& c) {
    auto require=[](bool b,const std::string& msg) { if(!b) throw std::invalid_argument(msg); };
    require(std::find(scene_names().begin(),scene_names().end(),c.scene)!=scene_names().end(),"Unknown scene");
    require(std::find(trajectory_names().begin(),trajectory_names().end(),c.trajectory)!=trajectory_names().end(),"Unknown trajectory");
    require(c.imu_frame=="camera" || c.imu_frame=="legacy_z_up","imu_frame must be camera or legacy_z_up");
    if(c.trajectory=="multilevel_walk")
        require(c.scene=="multilevel" || c.scene=="multilevel_tables","multilevel_walk requires a multilevel scene");
    if(c.trajectory=="tables_walk") require(c.scene=="flat_tables","tables_walk requires flat_tables");
    if(c.trajectory=="figure8_forward")
        require(c.motion_reference_camera>=0 && c.motion_reference_camera<(c.camera_count>0?c.camera_count:2),
                "figure8_forward requires a valid motion_reference_camera index");
    if(c.trajectory=="random_long") {
        const std::vector<std::string> motion_modes={"rectilinear","circular","scanner","xr_headset","controller","mower"};
        require(std::find(motion_modes.begin(),motion_modes.end(),c.motion_mode)!=motion_modes.end(),"Unknown motion mode");
        require(c.motion_reference_camera>=-1 && c.motion_reference_camera<16,"motion_reference_camera must be -1 (virtual center) or a camera index");
        require(c.motion_fixed_distance>=0 && c.motion_final_hold>=0 && c.motion_move_duration>=0,
                "Fixed rectilinear motion values must be nonnegative");
        if(c.motion_fixed_distance>0) {
            require(c.motion_mode=="rectilinear","motion_fixed_distance requires motion_mode rectilinear");
            require(c.motion_move_duration>0,"motion_fixed_distance requires motion_move_duration");
        }
    }
    const std::vector<std::string> names={"ideal","easy","medium","hard"};
    auto d=std::find(names.begin(),names.end(),c.difficulty); require(d!=names.end(),"Unknown difficulty");
    const size_t p=size_t(d-names.begin());
    const double defaults[4][8]={{0,0,1,1,0,0,0,0},{.7,.25,1,1,.002,.02,8e-6,8e-5},{1.4,.55,.97,.96,.003,.03,1e-5,1e-4},{2.8,.95,.90,.90,.0042,.042,1.4e-5,1.4e-4}};
    double* values[]={&c.image_noise,&c.extra_blur,&c.brightness,&c.contrast,&c.gyro_noise_density,&c.accel_noise_density,&c.gyro_bias_rw,&c.accel_bias_rw};
    for(size_t i=0;i<8;++i) {
        require(std::isfinite(*values[i]) && (*values[i]>=0 || *values[i]==-1),"Noise / optics values must be nonnegative (or -1 for profile default)");
        if(*values[i]==-1) *values[i]=defaults[p][i];
    }
    require(c.blackout_start>=0 && c.blackout_duration>=0 && c.blackout_start+c.blackout_duration<=86400,"Invalid camera blackout interval");
    require(std::isfinite(c.gravity) && c.gravity>0 && c.gravity<20,"Invalid gravity");
    require(std::isfinite(c.motion_time_offset) && c.motion_time_offset>=0 && c.motion_time_offset<=3600 &&
            std::isfinite(c.motion_time_scale) && c.motion_time_scale>0 && c.motion_time_scale<=10,"Invalid motion time transform");
    require(c.trajectory=="random_long" || (c.motion_time_offset==0 && c.motion_time_scale==1),
            "motion_time_offset/scale require random_long");
    require(c.reported_focal_scale>0.5 && c.reported_focal_scale<2 &&
            c.reported_baseline_scale>0.5 && c.reported_baseline_scale<2,"Invalid reported calibration scale");
    require(std::isfinite(c.reported_u0_offset) && std::isfinite(c.reported_v0_offset),"Reported principal point offsets must be finite");
    for(double v:c.initial_gyro_bias) require(std::isfinite(v),"Invalid gyro bias");
    for(double v:c.initial_accel_bias) require(std::isfinite(v),"Invalid accel bias");
    require(c.duration>0 && c.duration<=86400,"duration must be in (0, 86400] seconds");
    require(c.period>0 && c.period<=86400 && c.size>=0 && c.size<=1000 && c.speed_scale>0 && c.speed_scale<=100,"Invalid period, size, or speed scale");
    require(std::isfinite(c.figure8_height) && c.figure8_height>=0 && c.figure8_height<=1,"figure8_height must be in [0,1] metres at size=1");
    require(c.figure8_shape=="gerono" || c.figure8_shape=="bernoulli","figure8_shape must be gerono or bernoulli");
    require(c.figure8_shape=="gerono" || c.trajectory=="figure8_forward","Bernoulli shape requires figure8_forward");
    require(c.warmup>=0 && c.ramp>0 && c.cooldown>=0 && c.warmup<=86400 && c.ramp<=86400 && c.cooldown<=86400,"warmup and cooldown must be nonnegative; ramp must be positive");
    require(std::isfinite(c.yaw_turn_start) && c.yaw_turn_start>=0 &&
            std::isfinite(c.yaw_turn_duration) && c.yaw_turn_duration>=0 &&
            std::isfinite(c.yaw_turn_deg) && std::abs(c.yaw_turn_deg)<=360 &&
            (c.yaw_turn_deg==0 || (c.trajectory!="random_long" &&
                                   c.yaw_turn_duration>0 &&
                                   c.yaw_turn_start+c.yaw_turn_duration<=c.duration)),
            "Invalid yaw turn interval or angle");
    if(c.trajectory=="straight")
        require(c.warmup+c.period+c.cooldown<=c.duration,"straight trajectory requires duration >= warmup + period + cooldown");
    require(c.camera_hz>0 && c.camera_hz<=10000 && c.imu_hz>0 && c.imu_hz<=100000,"Invalid sensor rates");
    require(sample_count(c.duration,c.imu_hz)<=100000000 && sample_count(c.duration,c.camera_hz)<=10000000,"Sequence exceeds protective sample limit");
    require(c.width>=16 && c.height>=16 && c.width<=8192 && c.height<=8192,"Image dimensions must be in [16,8192]");
    require(c.samples>=1 && c.samples<=4,"samples is the supersampling factor per axis: 1, 2, 3, or 4");
    require(int64_t(c.width)*c.height*c.samples*c.samples<=16000000,"Supersampled image exceeds 16 million rays");
    require(c.threads>=1 && c.threads<=256,"threads must be in [1,256]");
    require(c.camera_model=="ucm" || c.camera_model=="seucm","camera_model must be ucm or seucm");
    require(std::isfinite(c.alpha) && c.alpha>=0 && c.alpha<1 && std::isfinite(c.beta) && c.beta>0,"Invalid SEUCM alpha/beta");
    require(c.baseline>=0 && c.baseline<2 && c.xi>=0 && c.xi<=1 && c.fov_deg>=10 && c.fov_deg<180,"Invalid camera model (xi must be in [0,1], FOV in [10,180))");
    require(c.clearance>=0 && c.clearance<=10,"Invalid collision clearance");
    require(c.extra_blur<=20 && c.sensor_blur>=0 && c.sensor_blur<=20,"Blur sigma must be in [0,20]");
    require(std::isfinite(c.lab_vignette) && c.lab_vignette>=0 && c.lab_vignette<=.95,"lab_vignette must be in [0,0.95]");
    require(c.brightness<=10 && c.contrast<=10,"Brightness/contrast must be in [0,10]");
    require(c.png_compression>=0 && c.png_compression<=9 && c.preview_count>=0 && c.preview_count<=50 && c.depth_every>=0,"Invalid output options");
    require(c.format=="png" || c.format=="pgm","format must be png or pgm");
    require(c.texture_filter=="nearest" || c.texture_filter=="bilinear" || c.texture_filter=="mipmap","Invalid texture filter");
    require(c.renderer=="cpu" || c.renderer=="gpu" || c.renderer=="auto","renderer must be cpu, gpu, or auto");
    if(c.camera_count==0) {
        require(c.camera_extrinsics.empty(),"camera_count is required when explicit camera extrinsics are configured");
    } else {
        require(c.camera_count==2 || c.camera_count==4 || c.camera_count==6,"camera_count must be 2, 4 or 6");
        require(c.camera_extrinsics.size()==size_t(c.camera_count),"Incomplete camera rig");
        for(size_t index=0;index<c.camera_extrinsics.size();++index) {
            const auto& camera=c.camera_extrinsics[index];
            require(camera.rotation_set && camera.translation_set,"Every explicit camera needs rotation_BC and translation_BC");
            if(camera.intrinsics_set) {
                const auto& k=camera.intrinsics;
                require(c.camera_model=="seucm","Explicit camera intrinsics require camera_model: seucm");
                require(std::all_of(k.begin(),k.end(),[](double x){return std::isfinite(x);}) &&
                        k[0]>0 && k[1]>0 && k[6]>=0 && k[6]<1 && k[7]>0,
                        "Invalid per-camera SEUCM intrinsics");
            }
            Mat3 R; R.a=camera.rotation; const Mat3 I=R.transpose()*R;
            double error=0; for(int row=0;row<3;++row) for(int col=0;col<3;++col) error=std::max(error,std::abs(I(row,col)-(row==col?1.0:0.0)));
            const double det=R(0,0)*(R(1,1)*R(2,2)-R(1,2)*R(2,1))-R(0,1)*(R(1,0)*R(2,2)-R(1,2)*R(2,0))+R(0,2)*(R(1,0)*R(2,1)-R(1,1)*R(2,0));
            require(error<1e-5 && std::abs(det-1.0)<1e-5,"Camera "+std::to_string(index)+" rotation_BC is not a rotation matrix");
        }
    }
    require(!c.output.empty(),"Empty output path");
    const long double end=static_cast<long double>(c.start_ns)+static_cast<long double>(c.duration+1)*1e9L;
    require(end<std::numeric_limits<int64_t>::max(),"Timestamp overflow");
}
std::string dump_config(const Config& c) {
    std::ostringstream s; s<<std::setprecision(17)<<"# viosim_cpp 1.0.0; flat scalar configuration\n";
#define X(x) s<<#x<<": "<<c.x<<'\n';
#define S(x) s<<#x<<": '"<<c.x<<"'\n";
    S(truth_output) S(camera_model) S(imu_frame) S(scene) S(trajectory) S(difficulty) S(output) S(assets) S(texture_filter) S(format) S(renderer) S(motion_mode) S(motion_config) S(figure8_shape)
    X(blackout_start) X(blackout_duration) X(gravity) X(reported_focal_scale) X(reported_baseline_scale) X(reported_u0_offset) X(reported_v0_offset) X(motion_time_offset) X(motion_time_scale) X(duration) X(period) X(size) X(speed_scale) X(warmup) X(ramp) X(cooldown) X(roll_deg) X(yaw_deg) X(yaw_offset_deg) X(yaw_turn_start) X(yaw_turn_duration) X(yaw_turn_deg) X(motion_fixed_distance) X(motion_final_hold) X(motion_move_duration)
    X(profile_render) X(gpu_precompute_rays) X(gpu_postprocess)
    X(alpha) X(beta) X(width) X(height) X(camera_hz) X(imu_hz) X(baseline) X(xi) X(fov_deg) X(right_yaw_deg) X(right_roll_deg) X(clearance)
    X(figure8_height)
    X(image_noise) X(extra_blur) X(sensor_blur) X(brightness) X(contrast) X(gyro_noise_density) X(accel_noise_density) X(gyro_bias_rw) X(accel_bias_rw) X(lab_vignette)
    X(threads) X(samples) X(png_compression) X(preview_count) X(depth_every) X(seed) X(motion_seed) X(motion_reference_camera) X(start_ns)
#undef X
#undef S
    s<<"lab_reflections: "<<(c.lab_reflections?"true":"false")<<'\n';
    s<<"distance_shading: "<<(c.distance_shading?"true":"false")<<'\n';
    if(c.camera_count>0) {
        s<<"camera_count: "<<c.camera_count<<'\n';
        for(size_t i=0;i<c.camera_extrinsics.size();++i) {
            s<<"cam"<<i<<"_rotation_bc: [";
            for(size_t j=0;j<9;++j) s<<(j?", ":"")<<c.camera_extrinsics[i].rotation[j];
            s<<"]\ncam"<<i<<"_translation_bc: [";
            for(size_t j=0;j<3;++j) s<<(j?", ":"")<<c.camera_extrinsics[i].translation[j];
            s<<"]\n";
            if(c.camera_extrinsics[i].intrinsics_set) {
                s<<"cam"<<i<<"_intrinsics: [";
                for(size_t j=0;j<8;++j) s<<(j?", ":"")<<c.camera_extrinsics[i].intrinsics[j];
                s<<"]\n";
            }
        }
    }
    if(c.initial_gyro_bias_set) s<<"initial_gyro_bias: ["<<c.initial_gyro_bias[0]<<", "<<c.initial_gyro_bias[1]<<", "<<c.initial_gyro_bias[2]<<"]\n";
    if(c.initial_accel_bias_set) s<<"initial_accel_bias: ["<<c.initial_accel_bias[0]<<", "<<c.initial_accel_bias[1]<<", "<<c.initial_accel_bias[2]<<"]\n";
    return s.str();
}
std::filesystem::path locate_assets(const Config& c,const std::filesystem::path& exe) {
    namespace fs=std::filesystem;
    std::vector<fs::path> candidates;
    if(!c.assets.empty()) candidates.push_back(c.assets);
    else {
        fs::path resolved=fs::absolute(exe);
        // argv[0] may contain only "viosim" when launched through PATH.
        if(!fs::is_regular_file(resolved)) {
            if(const char* path_env=std::getenv("PATH")) {
                std::istringstream paths(path_env); std::string directory;
#ifdef _WIN32
                const char separator=';';
#else
                const char separator=':';
#endif
                while(std::getline(paths,directory,separator)) {
                    const auto p=fs::path(directory)/exe;
                    if(fs::is_regular_file(p)) { resolved=fs::absolute(p); break; }
                }
            }
        }
        std::error_code ec;
        const auto canonical=fs::canonical(resolved,ec);
        const auto parent=(ec?resolved:canonical).parent_path();
        candidates={parent/"assets/textures",parent/"../share/viosim/assets/textures",fs::current_path()/"assets/textures",VIOSIM_SOURCE_ASSETS};
    }
    for(auto& p:candidates) if(fs::is_regular_file(p/"floor_tiles.png")) return fs::weakly_canonical(p);
    throw std::runtime_error("Textures not found. Use --assets /path/to/assets/textures");
}
} // namespace vio
