#include "long_motion/options.hpp"
#include <sstream>
#include <locale>
#include <cctype>
#include <limits>
namespace long_motion {
namespace {
std::string trim(std::string s) {auto a=s.find_first_not_of(" \t\r\n"),b=s.find_last_not_of(" \t\r\n");return a==std::string::npos?"":s.substr(a,b-a+1);}
double real(const std::string&s){std::istringstream i(s);i.imbue(std::locale::classic());double v;i>>v;if(!i||!std::isfinite(v))throw std::invalid_argument("Invalid finite number: "+s);i>>std::ws;if(!i.eof())throw std::invalid_argument("Trailing data in number: "+s);return v;}
int integer(const std::string&s){double v=real(s);if(v!=std::floor(v)||v<std::numeric_limits<int>::min()||v>std::numeric_limits<int>::max())throw std::invalid_argument("Invalid integer: "+s);return int(v);}
bool boolean(const std::string&s){if(s=="true")return true;if(s=="false")return false;throw std::invalid_argument("Expected true or false");}
std::vector<double> array(std::string s){for(char& c:s)if(c=='['||c==']'||c==',')c=' ';std::istringstream i(s);i.imbue(std::locale::classic());std::vector<double> a;double x;while(i>>x){if(!std::isfinite(x))throw std::invalid_argument("Invalid array number");a.push_back(x);}if(!i.eof())throw std::invalid_argument("Invalid numeric array");return a;}
Vec3 vec(const std::string&s){auto a=array(s);if(a.size()!=3)throw std::invalid_argument("Expected three numbers");return {a[0],a[1],a[2]};}
}
std::map<std::string,std::string> read_flat_options(std::istream& i){
    std::map<std::string,std::string> values;std::string line;size_t n=0;
    while(std::getline(i,line)){++n;auto hash=line.find('#');if(hash!=std::string::npos)line.resize(hash);line=trim(line);if(line.empty())continue;auto colon=line.find(':');if(colon==std::string::npos)throw std::invalid_argument("Expected key: value at line "+std::to_string(n));auto key=trim(line.substr(0,colon)),value=trim(line.substr(colon+1));if(key.empty()||value.empty())throw std::invalid_argument("Nested/empty YAML values are not supported by the standalone tool");if(value.size()>=2&&((value.front()=='\''&&value.back()=='\'')||(value.front()=='\"'&&value.back()=='\"')))value=value.substr(1,value.size()-2);if(!values.emplace(key,value).second)throw std::invalid_argument("Duplicate option: "+key);}
    return values;
}
void set_option(Config& c,const std::string& k,const std::string& v){
    if(k=="algorithm_version"){if(v!=algorithm_version)throw std::invalid_argument("Motion algorithm version mismatch");return;}
    if(k=="mode"){c.mode=parse_mode(v);return;}
    if(k=="seed"){if(v.empty()||v.find_first_not_of("0123456789")!=std::string::npos)throw std::invalid_argument("seed must be an unsigned decimal uint64");size_t used=0;c.seed=std::stoull(v,&used);if(used!=v.size())throw std::invalid_argument("Invalid seed");return;}
#define D(key) if(k==#key){c.key=real(v);return;}
#define I(key) if(k==#key){c.key=integer(v);return;}
#define B(key) if(k==#key){c.key=boolean(v);return;}
#define V(key) if(k==#key){c.key=vec(v);return;}
    V(start) V(circle_reference_offset_body) V(optical_axis_body) V(up_body) V(line_axis) V(circle_center)
    D(body_radius) D(camera_radius) D(clearance) D(warmup)
    D(limits.speed) D(limits.acceleration) D(limits.jerk) D(limits.angular_speed) D(limits.angular_acceleration) D(limits.angular_jerk)
    D(duration_min) D(duration_max) D(step_min) D(step_max) D(height_min) D(height_max)
    D(pause_probability) D(pause_min) D(pause_max) D(look_yaw) D(look_pitch) D(look_roll)
    D(nav_resolution) I(nav_max_nodes) I(max_attempts) I(collision_depth) D(numerical_guard) B(hold_when_blocked) B(enable_safe_retreat) I(history_capacity)
    B(rectilinear_fixed_axis) B(align_with_travel)
    D(rectilinear_distance) D(rectilinear_final_hold)
    D(circle_radius_min) D(circle_radius_max) D(circle_angle_min) D(circle_angle_max) B(circle_fixed_center) D(circle_radius)
    D(gesture_radius) I(controller_relocate_every) D(controller_relocate_speed)
    D(mower_radius) D(mower_imu_height) D(mower_top_above_imu) D(mower_lane_spacing) B(mower_alternate_axis)
#undef D
#undef I
#undef B
#undef V
    if(k=="initial_rotation_row_major"){auto a=array(v);if(a.size()!=9)throw std::invalid_argument("Expected nine rotation entries");std::copy(a.begin(),a.end(),c.initial_rotation.a.begin());return;}
    if(k=="camera_offsets"){auto a=array(v);if(a.size()%3)throw std::invalid_argument("Camera offset array must have a multiple of three numbers");c.camera_offsets.clear();for(size_t i=0;i<a.size();i+=3)c.camera_offsets.push_back({a[i],a[i+1],a[i+2]});return;}
    throw std::invalid_argument("Unknown motion option: "+k);
}
} // namespace long_motion
