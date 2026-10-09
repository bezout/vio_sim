#include "viosim/gpu_renderer.hpp"
#include "postprocess_shader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <chrono>
#include <sstream>
#include <iomanip>

#ifdef VIOSIM_HAVE_EGL_OPENGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/glcorearb.h>
#endif

namespace vio {

#ifdef VIOSIM_HAVE_EGL_OPENGL
namespace {

const char* compute_shader=R"GLSL(#version 430
layout(local_size_x=8,local_size_y=8,local_size_z=1) in;
layout(r32f,binding=0) uniform writeonly image2DArray output_image;
layout(binding=0) uniform sampler2DArray textures_image;
layout(std430,binding=1) readonly buffer BoxBuffer { vec4 boxes[]; };
layout(std430,binding=2) readonly buffer MaterialBuffer { vec4 materials[]; };
layout(std430,binding=3) buffer RayBuffer { vec4 cached_rays[]; };
// 0: reference calculation, 1: initialize table, 2: render using table.
uniform int ray_mode;

uniform int camera_seucm;
uniform float camera_alpha,camera_beta;
uniform int image_width,image_height,sample_count,camera_count,box_count,decal_count;
uniform int texture_filter,distance_shading,lab_scene,lab_reflections;
uniform float camera_fx,camera_fy,camera_cx,camera_cy,camera_xi,lab_vignette;
uniform vec3 room_lo,room_hi;
uniform vec3 pose_p[4],pose_r0[4],pose_r1[4],pose_r2[4];

vec3 rotate_camera(int camera,vec3 v) {
    return vec3(dot(pose_r0[camera],v),dot(pose_r1[camera],v),dot(pose_r2[camera],v));
}

vec3 camera_ray(float u,float v) {
    float x=(u-camera_cx)/camera_fx,y=(v-camera_cy)/camera_fy,r2=x*x+y*y;
    if(camera_seucm!=0) {
        float z=(1.0-camera_beta*camera_alpha*camera_alpha*r2)/(camera_alpha*sqrt(1.0-(2.0*camera_alpha-1.0)*camera_beta*r2)+1.0-camera_alpha);
        return normalize(vec3(x,y,z));
    }
    float l=(camera_xi+sqrt(1.0+(1.0-camera_xi*camera_xi)*r2))/(1.0+r2);
    return normalize(vec3(l*x,l*y,l-camera_xi));
}

bool intersect_box(vec3 lo,vec3 hi,vec3 origin,vec3 ray,
                   out float near_t,out float far_t,out int near_face,out int far_face) {
    near_t=-1e30; far_t=1e30; near_face=-1; far_face=-1;
    for(int axis=0;axis<3;++axis) {
        if(abs(ray[axis])<1e-14) {
            if(origin[axis]<lo[axis] || origin[axis]>hi[axis]) return false;
            continue;
        }
        float t0=(lo[axis]-origin[axis])/ray[axis];
        float t1=(hi[axis]-origin[axis])/ray[axis];
        int f0=2*axis,f1=2*axis+1;
        if(t0>t1) { float t=t0;t0=t1;t1=t; int f=f0;f0=f1;f1=f; }
        if(t0>near_t) { near_t=t0;near_face=f0; }
        if(t1<far_t) { far_t=t1;far_face=f1; }
        if(near_t>far_t) return false;
    }
    return far_t>1e-7;
}

vec2 surface_uv(vec3 point,int face) {
    int axis=face/2;
    if(axis==0) return point.yz;
    if(axis==1) return point.xz;
    return point.xy;
}

float texture_value(int texture_index,vec2 uv,float lod,bool repeat_texture) {
    ivec2 dimensions=textureSize(textures_image,0).xy;
    if(texture_filter==0) {
        vec2 p=repeat_texture?fract(uv):clamp(uv,0.0,1.0);
        ivec2 pixel=ivec2(clamp(p,0.0,1.0)*vec2(dimensions-1));
        return texelFetch(textures_image,ivec3(pixel,texture_index),0).r;
    }
    vec2 p=repeat_texture?fract(uv):clamp(uv,0.5/vec2(dimensions),1.0-0.5/vec2(dimensions));
    return textureLod(textures_image,vec3(p,float(texture_index)),lod).r;
}

float lab_radiance(int object_index,int face,vec3 point,vec3 eye,float albedo) {
    if(object_index==0 && face==5) return albedo>240.0?albedo:albedo*.63;
    if(object_index==0 && face==3 && point.x>-1.73 && point.x<-.61 && point.z<2.52) return albedo;
    float gain=.82;
    if(object_index==0 && face==4) gain=.95;
    else if(face==5) gain=1.06;
    else if(face==2) gain=.84;
    else if(face==0 || face==1) gain=.75;
    gain*=.87+.13*clamp(point.y/8.8,0.0,1.0);
    float value=albedo*gain;
    if(lab_reflections!=0 && object_index==0 && face==4 && eye.z>.05) {
        const vec4 lights[5]=vec4[5](vec4(-1.3,2.0,.65,.105),vec4(1.3,2.5,.65,.105),
            vec4(-1.25,4.65,.65,.105),vec4(1.25,5.35,.65,.105),vec4(0.0,7.0,.65,.105));
        float qx=point.x+(point.x-eye.x)*3.15/eye.z;
        float qy=point.y+(point.y-eye.y)*3.15/eye.z;
        for(int i=0;i<5;++i) {
            float dx=max(0.0,abs(qx-lights[i].x)-lights[i].z);
            float dy=max(0.0,abs(qy-lights[i].y)-lights[i].w);
            float exponent=(dx*dx+dy*dy)/(.34*.34);
            if(exponent<18.0) value+=76.0*exp(-.5*exponent);
        }
    }
    return value;
}

void main() {
    ivec3 output_pixel=ivec3(gl_GlobalInvocationID);
    if(output_pixel.x>=image_width || output_pixel.y>=image_height || output_pixel.z>=camera_count) return;
    int camera=output_pixel.z;
    float total=0.0;
    for(int sy=0;sy<sample_count;++sy) for(int sx=0;sx<sample_count;++sx) {
        float step_size=1.0/float(sample_count);
        float u=(float(output_pixel.x*sample_count+sx)+0.5)*step_size-0.5;
        float v=(float(output_pixel.y*sample_count+sy)+0.5)*step_size-0.5;
        int stride=texture_filter==2?3:1;
        int ray_index=((output_pixel.y*sample_count+sy)*(image_width*sample_count)+output_pixel.x*sample_count+sx)*stride;
        vec3 local_ray,local_dx=vec3(0),local_dy=vec3(0);
        if(ray_mode==2) {
            local_ray=cached_rays[ray_index].xyz;
        } else {
            local_ray=camera_ray(u,v);
        }
        if(ray_mode==1) {
            cached_rays[ray_index]=vec4(local_ray,0);
            if(texture_filter==2) {
                cached_rays[ray_index+1]=vec4(camera_ray(u+0.5*step_size,v)-camera_ray(u-0.5*step_size,v),0);
                cached_rays[ray_index+2]=vec4(camera_ray(u,v+0.5*step_size)-camera_ray(u,v-0.5*step_size),0);
            }
            continue;
        }
        vec3 ray=rotate_camera(camera,local_ray),origin=pose_p[camera];
        float best=1e30,near_t,far_t; int object_index=-1,face=-1,near_face,far_face;
        if(intersect_box(room_lo,room_hi,origin,ray,near_t,far_t,near_face,far_face)) {
            best=far_t; object_index=0; face=far_face;
        }
        for(int object=0;object<box_count;++object) {
            vec3 lo=boxes[4*object].xyz,hi=boxes[4*object+1].xyz;
            if(intersect_box(lo,hi,origin,ray,near_t,far_t,near_face,far_face)) {
                bool outside=near_t>1e-7; float hit_t=outside?near_t:far_t;
                if(hit_t<best) { best=hit_t;object_index=object+1;face=outside?near_face:far_face; }
            }
        }
        if(object_index<0) continue;
        vec3 point=origin+ray*best; vec2 uv=surface_uv(point,face);
        float value=160.0,us=1.0,vs=1.0,uo=0.0,vo=0.0;
        int texture_index=-1; bool repeat_texture=true;
        if(object_index==0) {
            vec4 p0=materials[3*face],p1=materials[3*face+1];
            value=p0.x;us=p0.y;vs=p0.z;uo=p0.w;vo=p1.x;
            texture_index=int(p1.y);repeat_texture=p1.z>0.5;
            for(int decal_index=0;decal_index<decal_count;++decal_index) {
                int material_index=6+decal_index;
                vec4 d0=materials[3*material_index],d1=materials[3*material_index+1],bounds=materials[3*material_index+2];
                if(int(d1.w)==face && uv.x>=bounds.x && uv.x<=bounds.y && uv.y>=bounds.z && uv.y<=bounds.w) {
                    value=d0.x;us=d0.y;vs=d0.z;uo=d0.w;vo=d1.x;
                    texture_index=int(d1.y);repeat_texture=d1.z>0.5;
                }
            }
        } else {
            int box_index=4*(object_index-1);
            vec3 lo=boxes[box_index].xyz,hi=boxes[box_index+1].xyz;
            int axis=face/2,a=axis==0?1:0,b=axis==2?1:2;
            us=1.0/(hi[a]-lo[a]);vs=1.0/(hi[b]-lo[b]);uo=-lo[a]*us;vo=-lo[b]*vs;
            float override_texture=boxes[box_index+2+face/4][face%4];
            texture_index=int(override_texture>=0.0?override_texture:boxes[box_index+1].w);repeat_texture=false;
        }
        if(texture_index>=0) {
            float lod=0.0;
            if(texture_filter==2) {
                int normal_axis=face/2;
                if(abs(ray[normal_axis])>1e-10) {
                    if(ray_mode==2) {
                        local_dx=cached_rays[ray_index+1].xyz;
                        local_dy=cached_rays[ray_index+2].xyz;
                    } else {
                        local_dx=camera_ray(u+0.5*step_size,v)-camera_ray(u-0.5*step_size,v);
                        local_dy=camera_ray(u,v+0.5*step_size)-camera_ray(u,v-0.5*step_size);
                    }
                    vec3 ddx=rotate_camera(camera,local_dx);
                    vec3 ddy=rotate_camera(camera,local_dy);
                    vec3 px=(ddx-ray*(ddx[normal_axis]/ray[normal_axis]))*best;
                    vec3 py=(ddy-ray*(ddy[normal_axis]/ray[normal_axis]))*best;
                    int a=normal_axis==0?1:0,b=normal_axis==2?1:2;
                    vec2 texture_size=vec2(textureSize(textures_image,0).xy)*vec2(us,vs);
                    float rho=max(length(vec2(px[a],px[b])*texture_size),length(vec2(py[a],py[b])*texture_size));
                    lod=log2(max(rho,1.0));
                } else lod=float(textureQueryLevels(textures_image)-1);
            }
            value=texture_value(texture_index,uv*vec2(us,vs)+vec2(uo,vo),lod,repeat_texture);
        }
        if(lab_scene!=0) value=lab_radiance(object_index,face,point,origin,value);
        if(distance_shading!=0) value*=clamp(1.0-0.02*best,0.75,1.0);
        total+=value;
    }
    if(ray_mode==1) return;
    float vignette=1.0;
    if(lab_scene!=0 && lab_vignette>0.0) {
        float dx=float(output_pixel.x)-camera_cx,dy=float(output_pixel.y)-camera_cy;
        float r2=(dx*dx+dy*dy)/(camera_cx*camera_cx+camera_cy*camera_cy);
        vignette=clamp(1.0-lab_vignette*r2,.05,1.0);
    }
    imageStore(output_image,output_pixel,vec4(total/float(sample_count*sample_count)*vignette,0,0,1));
}
)GLSL";

template<class T> T gl_function(const char* name) {
    auto function=reinterpret_cast<T>(eglGetProcAddress(name));
    if(!function) throw std::runtime_error(std::string("Missing OpenGL function ")+name);
    return function;
}

bool software_renderer(std::string name) {
    std::transform(name.begin(),name.end(),name.begin(),[](unsigned char c){ return char(std::tolower(c)); });
    return name.find("llvmpipe")!=std::string::npos || name.find("softpipe")!=std::string::npos
        || name.find("software")!=std::string::npos;
}

void append_material(std::vector<float>& data,const Material& material,int face,
                     double u0=0,double u1=0,double v0=0,double v1=0) {
    const float values[]={float(material.value),float(material.us),float(material.vs),float(material.uo),
                          float(material.vo),float(material.texture),material.repeat?1.f:0.f,float(face),
                          float(u0),float(u1),float(v0),float(v1)};
    data.insert(data.end(),std::begin(values),std::end(values));
}

Image8 finish_image(std::vector<float> values,const Config& config,uint64_t seed) {
    gaussian_blur(values,config.width,config.height,std::hypot(config.sensor_blur,config.extra_blur));
    Noise noise(seed); Image8 image{config.width,config.height,std::vector<uint8_t>(values.size())};
    for(size_t i=0;i<values.size();++i) {
        double value=std::clamp((values[i]-127.5)*config.contrast+127.5,0.0,255.0)*config.brightness;
        if(config.image_noise>0) value+=config.image_noise*noise.normal();
        image.data[i]=uint8_t(std::clamp(std::round(value),0.0,255.0));
    }
    return image;
}

} // namespace

struct GpuRenderer::Impl {
    EGLDisplay display=EGL_NO_DISPLAY; EGLContext context=EGL_NO_CONTEXT; EGLSurface surface=EGL_NO_SURFACE;
    GLuint program=0,texture_array=0,output_texture=0,box_buffer=0,material_buffer=0;
    GLuint post_program=0,blur_texture=0,byte_texture=0,kernel_buffer=0;
    GLint post_pass=-1,post_seeds=-1;
    Config config; std::string name; int boxes_count=0,decals_count=0,output_layers=2;
    GLint camera_count_location=-1,pose_p_location=-1,pose_r0_location=-1,pose_r1_location=-1,pose_r2_location=-1;
    mutable std::mutex mutex;
    using Clock=std::chrono::steady_clock;
    mutable std::mutex profile_mutex;
    uint64_t profile_count=0;
    // Profiling explicitly waits before readback to distinguish GPU waiting
    // from transfer. All durations are host elapsed time, not hardware timers.
    std::array<double,6> profile_sum{},profile_max{};
    GLuint ray_buffer=0;
    size_t ray_bytes=0;
    double ray_init_ms=0;
    static double ms(Clock::time_point a,Clock::time_point b) {
        return std::chrono::duration<double,std::milli>(b-a).count();
    }
    std::string profile() const {
        if(!config.profile_render) return {};
        std::lock_guard<std::mutex> lock(profile_mutex);
        std::ostringstream out;out<<std::fixed<<std::setprecision(3);
        out<<" GPU profile groups="<<profile_count<<" ray_cache_MiB="<<double(ray_bytes)/(1024*1024)
           <<" postprocess="<<(config.gpu_postprocess?"gpu":"cpu")
           <<" ray_init_host_ms="<<ray_init_ms<<" ms(avg/max)";
        const char* names[]={"mutex_wait","submit","gpu_wait_host","readback","post_cpu","total_host"};
        for(size_t i=0;i<profile_sum.size();++i) {
            out<<' '<<names[i]<<'='<<(profile_count?profile_sum[i]/profile_count:0)<<'/'<<profile_max[i];
        }
        return out.str();
    }

    PFNGLCREATESHADERPROC CreateShader{}; PFNGLSHADERSOURCEPROC ShaderSource{}; PFNGLCOMPILESHADERPROC CompileShader{};
    PFNGLGETSHADERIVPROC GetShaderiv{}; PFNGLGETSHADERINFOLOGPROC GetShaderInfoLog{}; PFNGLDELETESHADERPROC DeleteShader{};
    PFNGLCREATEPROGRAMPROC CreateProgram{}; PFNGLATTACHSHADERPROC AttachShader{}; PFNGLLINKPROGRAMPROC LinkProgram{};
    PFNGLGETPROGRAMIVPROC GetProgramiv{}; PFNGLGETPROGRAMINFOLOGPROC GetProgramInfoLog{}; PFNGLDELETEPROGRAMPROC DeleteProgram{};
    PFNGLUSEPROGRAMPROC UseProgram{}; PFNGLGETUNIFORMLOCATIONPROC GetUniformLocation{}; PFNGLUNIFORM1IPROC Uniform1i{};
    PFNGLUNIFORM1FPROC Uniform1f{}; PFNGLUNIFORM3FVPROC Uniform3fv{}; PFNGLGENTEXTURESPROC GenTextures{};
    PFNGLBINDTEXTUREPROC BindTexture{}; PFNGLTEXSTORAGE3DPROC TexStorage3D{}; PFNGLTEXSUBIMAGE3DPROC TexSubImage3D{};
    PFNGLTEXPARAMETERIPROC TexParameteri{}; PFNGLDELETETEXTURESPROC DeleteTextures{}; PFNGLACTIVETEXTUREPROC ActiveTexture{};
    PFNGLGENBUFFERSPROC GenBuffers{}; PFNGLBINDBUFFERPROC BindBuffer{}; PFNGLBUFFERDATAPROC BufferData{};
    PFNGLBINDBUFFERBASEPROC BindBufferBase{}; PFNGLDELETEBUFFERSPROC DeleteBuffers{}; PFNGLBINDIMAGETEXTUREPROC BindImageTexture{};
    PFNGLDISPATCHCOMPUTEPROC DispatchCompute{}; PFNGLMEMORYBARRIERPROC MemoryBarrier{}; PFNGLGETTEXIMAGEPROC GetTexImage{};
    PFNGLGETSTRINGPROC GetString{}; PFNGLPIXELSTOREIPROC PixelStorei{};
    PFNGLGETINTEGER64VPROC GetInteger64v{};
    PFNGLFINISHPROC Finish{};
    PFNGLUNIFORM2UIVPROC Uniform2uiv{};

    Impl(const Camera& camera,const Scene& scene,const TextureBank& textures,const Config& config_):config(config_) {
        try {
            initialize_egl(); load_functions();
            const char* renderer=reinterpret_cast<const char*>(GetString(GL_RENDERER));
            name=renderer?renderer:"unknown OpenGL renderer";
            if(software_renderer(name) && std::getenv("VIOSIM_ALLOW_SOFTWARE_GPU")==nullptr)
                throw std::runtime_error("OpenGL renderer is software ("+name+")");
            program=build_program();
            cache_dynamic_uniforms();
            upload_textures(textures); upload_scene(scene); create_output(camera);
            set_static_uniforms(camera,scene);
            initialize_rays(camera);
            if(config.gpu_postprocess) initialize_postprocess();
            eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
        } catch(...) { cleanup(); throw; }
    }

    ~Impl() { std::lock_guard<std::mutex> lock(mutex); cleanup(); }

    void initialize_egl() {
        auto get_platform=reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        EGLint major=0,minor=0;
#ifdef EGL_PLATFORM_SURFACELESS_MESA
        if(get_platform) display=get_platform(EGL_PLATFORM_SURFACELESS_MESA,EGL_DEFAULT_DISPLAY,nullptr);
#endif
        bool initialized=display!=EGL_NO_DISPLAY && eglInitialize(display,&major,&minor);
        if(!initialized) {
            if(display!=EGL_NO_DISPLAY) eglTerminate(display);
            display=eglGetDisplay(EGL_DEFAULT_DISPLAY);
            initialized=display!=EGL_NO_DISPLAY && eglInitialize(display,&major,&minor);
        }
        if(!initialized) throw std::runtime_error("Cannot initialize EGL display");
        if(!eglBindAPI(EGL_OPENGL_API)) throw std::runtime_error("Cannot bind EGL OpenGL API");
        const EGLint attributes[]={EGL_SURFACE_TYPE,EGL_PBUFFER_BIT,EGL_RENDERABLE_TYPE,EGL_OPENGL_BIT,
                                   EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,EGL_NONE};
        EGLConfig egl_config{}; EGLint count=0;
        if(!eglChooseConfig(display,attributes,&egl_config,1,&count) || count==0) throw std::runtime_error("No EGL OpenGL pbuffer configuration");
        const EGLint pbuffer_attributes[]={EGL_WIDTH,1,EGL_HEIGHT,1,EGL_NONE};
        surface=eglCreatePbufferSurface(display,egl_config,pbuffer_attributes);
        if(surface==EGL_NO_SURFACE) throw std::runtime_error("Cannot create EGL pbuffer surface");
        const EGLint context_attributes[]={EGL_CONTEXT_MAJOR_VERSION_KHR,4,EGL_CONTEXT_MINOR_VERSION_KHR,3,
                                           EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR,EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT_KHR,EGL_NONE};
        context=eglCreateContext(display,egl_config,EGL_NO_CONTEXT,context_attributes);
        if(context==EGL_NO_CONTEXT) throw std::runtime_error("Cannot create an OpenGL 4.3 EGL context");
        if(!eglMakeCurrent(display,surface,surface,context)) throw std::runtime_error("Cannot activate EGL OpenGL context");
    }

#define LOAD(name_) name_=gl_function<decltype(name_)>("gl" #name_)
    void load_functions() {
        LOAD(CreateShader);LOAD(ShaderSource);LOAD(CompileShader);LOAD(GetShaderiv);LOAD(GetShaderInfoLog);LOAD(DeleteShader);
        LOAD(CreateProgram);LOAD(AttachShader);LOAD(LinkProgram);LOAD(GetProgramiv);LOAD(GetProgramInfoLog);LOAD(DeleteProgram);
        LOAD(UseProgram);LOAD(GetUniformLocation);LOAD(Uniform1i);LOAD(Uniform1f);LOAD(Uniform3fv);
        LOAD(GenTextures);LOAD(BindTexture);LOAD(TexStorage3D);LOAD(TexSubImage3D);LOAD(TexParameteri);LOAD(DeleteTextures);LOAD(ActiveTexture);
        LOAD(GenBuffers);LOAD(BindBuffer);LOAD(BufferData);LOAD(BindBufferBase);LOAD(DeleteBuffers);
        LOAD(BindImageTexture);LOAD(DispatchCompute);LOAD(MemoryBarrier);LOAD(GetTexImage);LOAD(GetString);LOAD(PixelStorei);
        LOAD(GetInteger64v);LOAD(Finish);LOAD(Uniform2uiv);
    }
#undef LOAD

    GLuint build_program(const GLchar* source=compute_shader) {
        GLuint shader=CreateShader(GL_COMPUTE_SHADER);
        ShaderSource(shader,1,&source,nullptr);CompileShader(shader);
        GLint ok=0;GetShaderiv(shader,GL_COMPILE_STATUS,&ok);
        if(!ok) { GLint length=0;GetShaderiv(shader,GL_INFO_LOG_LENGTH,&length);std::string log(size_t(std::max(1,length)),'\0');GetShaderInfoLog(shader,length,nullptr,log.data());DeleteShader(shader);throw std::runtime_error("VioSim compute shader compilation failed: "+log); }
        GLuint result=CreateProgram();AttachShader(result,shader);LinkProgram(result);DeleteShader(shader);
        GetProgramiv(result,GL_LINK_STATUS,&ok);
        if(!ok) { GLint length=0;GetProgramiv(result,GL_INFO_LOG_LENGTH,&length);std::string log(size_t(std::max(1,length)),'\0');GetProgramInfoLog(result,length,nullptr,log.data());DeleteProgram(result);throw std::runtime_error("VioSim compute shader link failed: "+log); }
        return result;
    }

    void upload_textures(const TextureBank& bank) {
        if(bank.textures.empty()) throw std::runtime_error("VioSim texture bank is empty");
        int width=0,height=0;
        for(const auto& texture:bank.textures) {
            width=std::max(width,texture.levels.front().width);
            height=std::max(height,texture.levels.front().height);
        }
        int levels=1;
        for(int size=std::max(width,height);size>1;size/=2) ++levels;
        GenTextures(1,&texture_array);BindTexture(GL_TEXTURE_2D_ARRAY,texture_array);
        TexStorage3D(GL_TEXTURE_2D_ARRAY,levels,GL_R32F,width,height,GLsizei(bank.textures.size()));
        PixelStorei(GL_UNPACK_ALIGNMENT,1);
        for(size_t layer=0;layer<bank.textures.size();++layer) {
            std::vector<float> pixels(size_t(width)*height);
            for(int y=0;y<height;++y) for(int x=0;x<width;++x)
                pixels[size_t(y)*width+x]=float(bank.textures[layer].sample((x+.5)/width,(y+.5)/height,0,1,false));
            int previous_width=width,previous_height=height;
            for(int level=0;level<levels;++level) {
                TexSubImage3D(GL_TEXTURE_2D_ARRAY,level,0,0,GLint(layer),previous_width,previous_height,1,GL_RED,GL_FLOAT,pixels.data());
                if(level+1==levels) break;
                const int next_width=std::max(1,previous_width/2),next_height=std::max(1,previous_height/2);
                std::vector<float> next(size_t(next_width)*next_height);
                for(int y=0;y<next_height;++y) for(int x=0;x<next_width;++x) {
                    const int x0=2*x,x1=std::min(x0+1,previous_width-1),y0=2*y,y1=std::min(y0+1,previous_height-1);
                    next[size_t(y)*next_width+x]=.25f*(pixels[size_t(y0)*previous_width+x0]+pixels[size_t(y0)*previous_width+x1]
                        +pixels[size_t(y1)*previous_width+x0]+pixels[size_t(y1)*previous_width+x1]);
                }
                pixels=std::move(next);previous_width=next_width;previous_height=next_height;
            }
        }
        const int nearest=config.texture_filter=="nearest";
        TexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MAG_FILTER,nearest?GL_NEAREST:GL_LINEAR);
        TexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MIN_FILTER,nearest?GL_NEAREST:(config.texture_filter=="mipmap"?GL_LINEAR_MIPMAP_LINEAR:GL_LINEAR));
        TexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_WRAP_S,GL_REPEAT);TexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_WRAP_T,GL_REPEAT);
    }

    void upload_scene(const Scene& scene) {
        std::vector<float> box_data; boxes_count=int(scene.boxes.size());box_data.reserve(scene.boxes.size()*16);
        for(const auto& box:scene.boxes) {
            const float values[]={float(box.bounds.lo.x),float(box.bounds.lo.y),float(box.bounds.lo.z),0,
                                  float(box.bounds.hi.x),float(box.bounds.hi.y),float(box.bounds.hi.z),float(box.texture),
                                  float(box.face_textures[0]),float(box.face_textures[1]),float(box.face_textures[2]),float(box.face_textures[3]),
                                  float(box.face_textures[4]),float(box.face_textures[5]),-1,-1};
            box_data.insert(box_data.end(),std::begin(values),std::end(values));
        }
        std::vector<float> material_data;
        for(int face=0;face<6;++face) append_material(material_data,scene.walls[size_t(face)].base,-1);
        for(int face=0;face<6;++face) for(const auto& decal:scene.walls[size_t(face)].decals) {
            append_material(material_data,decal.material,face,decal.u0,decal.u1,decal.v0,decal.v1);++decals_count;
        }
        GenBuffers(1,&box_buffer);BindBuffer(GL_SHADER_STORAGE_BUFFER,box_buffer);
        BufferData(GL_SHADER_STORAGE_BUFFER,GLsizeiptr(box_data.size()*sizeof(float)),box_data.data(),GL_STATIC_DRAW);
        BindBufferBase(GL_SHADER_STORAGE_BUFFER,1,box_buffer);
        GenBuffers(1,&material_buffer);BindBuffer(GL_SHADER_STORAGE_BUFFER,material_buffer);
        BufferData(GL_SHADER_STORAGE_BUFFER,GLsizeiptr(material_data.size()*sizeof(float)),material_data.data(),GL_STATIC_DRAW);
        BindBufferBase(GL_SHADER_STORAGE_BUFFER,2,material_buffer);
    }

    void initialize_rays(const Camera& camera) {
        const auto start=Clock::now();
        const size_t bytes=size_t(camera.width)*camera.height*camera.samples*camera.samples
            *4*sizeof(float)*(config.texture_filter=="mipmap"?3:1);
        GLint64 limit=0;GetInteger64v(GL_MAX_SHADER_STORAGE_BLOCK_SIZE,&limit);
        // Keep the reference path on devices where the requested table exceeds
        // the per-buffer limit. The profile reports a zero-sized cache.
        const bool cache=config.gpu_precompute_rays && bytes<=size_t(limit);
        GenBuffers(1,&ray_buffer);BindBuffer(GL_SHADER_STORAGE_BUFFER,ray_buffer);
        ray_bytes=cache?bytes:0;
        BufferData(GL_SHADER_STORAGE_BUFFER,GLsizeiptr(cache?bytes:16),nullptr,GL_STATIC_DRAW);
        BindBufferBase(GL_SHADER_STORAGE_BUFFER,3,ray_buffer);
        if(cache) {
            BindImageTexture(0,output_texture,0,GL_TRUE,0,GL_WRITE_ONLY,GL_R32F);
            Uniform1i(camera_count_location,1);Uniform1i(location("ray_mode"),1);
            DispatchCompute(GLuint((camera.width+7)/8),GLuint((camera.height+7)/8),1);
            MemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            if(config.profile_render) Finish();
        }
        Uniform1i(location("ray_mode"),cache?2:0);
        ray_init_ms=ms(start,Clock::now());
    }

    void initialize_postprocess() {
        post_program=build_program(postprocess_shader);UseProgram(post_program);
        const auto loc=[&](const char* name){return GetUniformLocation(post_program,name);};
        Uniform1i(loc("width"),config.width);Uniform1i(loc("height"),config.height);
        Uniform1f(loc("contrast"),float(config.contrast));Uniform1f(loc("brightness"),float(config.brightness));
        Uniform1f(loc("noise_sigma"),float(config.image_noise));
        post_pass=loc("pass");post_seeds=loc("seeds[0]");
        const double sigma=std::hypot(config.sensor_blur,config.extra_blur);
        const int radius=sigma<=1e-5?0:std::max(1,int(std::ceil(3*sigma)));
        Uniform1i(loc("radius"),radius);
        std::vector<float> kernel(size_t(2*radius+1),1);double total=0;
        for(int i=-radius;i<=radius;++i) {
            const double v=radius?std::exp(-.5*i*i/(sigma*sigma)):1;
            kernel[size_t(i+radius)]=float(v);total+=v;
        }
        for(auto& v:kernel) v=float(v/total);
        GenBuffers(1,&kernel_buffer);BindBuffer(GL_SHADER_STORAGE_BUFFER,kernel_buffer);
        BufferData(GL_SHADER_STORAGE_BUFFER,GLsizeiptr(kernel.size()*sizeof(float)),kernel.data(),GL_STATIC_DRAW);
        BindBufferBase(GL_SHADER_STORAGE_BUFFER,4,kernel_buffer);
        GenTextures(1,&blur_texture);BindTexture(GL_TEXTURE_2D_ARRAY,blur_texture);
        TexStorage3D(GL_TEXTURE_2D_ARRAY,1,GL_R32F,config.width,config.height,output_layers);
        GenTextures(1,&byte_texture);BindTexture(GL_TEXTURE_2D_ARRAY,byte_texture);
        TexStorage3D(GL_TEXTURE_2D_ARRAY,1,GL_R8UI,config.width,config.height,output_layers);
    }

    void postprocess(const std::vector<uint64_t>& seeds) {
        UseProgram(post_program);
        GLuint words[8]{};
        for(size_t i=0;i<seeds.size();++i) { words[2*i]=GLuint(seeds[i]);words[2*i+1]=GLuint(seeds[i]>>32); }
        Uniform2uiv(post_seeds,GLsizei(seeds.size()),words);
        BindImageTexture(0,output_texture,0,GL_TRUE,0,GL_READ_ONLY,GL_R32F);
        BindImageTexture(1,blur_texture,0,GL_TRUE,0,GL_WRITE_ONLY,GL_R32F);
        BindImageTexture(2,byte_texture,0,GL_TRUE,0,GL_WRITE_ONLY,GL_R8UI);
        Uniform1i(post_pass,0);
        DispatchCompute(GLuint((config.width+7)/8),GLuint((config.height+7)/8),GLuint(seeds.size()));
        MemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
        BindImageTexture(0,blur_texture,0,GL_TRUE,0,GL_READ_ONLY,GL_R32F);
        Uniform1i(post_pass,1);
        DispatchCompute(GLuint((config.width+7)/8),GLuint((config.height+7)/8),GLuint(seeds.size()));
        MemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT|GL_TEXTURE_UPDATE_BARRIER_BIT);
    }

    void create_output(const Camera& camera) {
        output_layers=config.camera_count==4?4:2;
        GenTextures(1,&output_texture);BindTexture(GL_TEXTURE_2D_ARRAY,output_texture);
        TexStorage3D(GL_TEXTURE_2D_ARRAY,1,GL_R32F,camera.width,camera.height,output_layers);
        TexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MIN_FILTER,GL_NEAREST);TexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    }

    GLint location(const char* uniform) { GLint result=GetUniformLocation(program,uniform);if(result<0) throw std::runtime_error(std::string("Missing VioSim GPU uniform ")+uniform);return result; }
    void cache_dynamic_uniforms() {
        camera_count_location=location("camera_count");pose_p_location=location("pose_p[0]");
        pose_r0_location=location("pose_r0[0]");pose_r1_location=location("pose_r1[0]");pose_r2_location=location("pose_r2[0]");
    }
    void set_static_uniforms(const Camera& camera,const Scene& scene) {
        UseProgram(program);
        Uniform1i(location("camera_seucm"),camera.seucm);
        Uniform1f(location("camera_alpha"),float(camera.alpha));Uniform1f(location("camera_beta"),float(camera.beta));
        Uniform1i(location("image_width"),camera.width);Uniform1i(location("image_height"),camera.height);
        Uniform1i(location("sample_count"),camera.samples);Uniform1i(location("box_count"),boxes_count);Uniform1i(location("decal_count"),decals_count);
        Uniform1i(location("texture_filter"),config.texture_filter=="nearest"?0:(config.texture_filter=="bilinear"?1:2));
        Uniform1i(location("distance_shading"),config.distance_shading?1:0);
        Uniform1i(location("lab_scene"),scene.name=="lab_room"?1:0);
        Uniform1i(location("lab_reflections"),config.lab_reflections?1:0);
        Uniform1f(location("lab_vignette"),float(config.lab_vignette));
        Uniform1f(location("camera_fx"),float(camera.fx));Uniform1f(location("camera_fy"),float(camera.fy));
        Uniform1f(location("camera_cx"),float(camera.cx));Uniform1f(location("camera_cy"),float(camera.cy));Uniform1f(location("camera_xi"),float(camera.xi));
        const GLfloat lo[]={float(scene.room.lo.x),float(scene.room.lo.y),float(scene.room.lo.z)};
        const GLfloat hi[]={float(scene.room.hi.x),float(scene.room.hi.y),float(scene.room.hi.z)};
        Uniform3fv(location("room_lo"),1,lo);Uniform3fv(location("room_hi"),1,hi);Uniform1i(location("textures_image"),0);
    }

    void set_pose_uniforms(const std::vector<Pose>& poses) {
        std::array<GLfloat,12> positions{},rotations0{},rotations1{},rotations2{};
        for(size_t camera=0;camera<poses.size();++camera) {
            const Pose& pose=poses[camera];
            const size_t offset=3*camera;
            positions[offset]=float(pose.p.x); positions[offset+1]=float(pose.p.y); positions[offset+2]=float(pose.p.z);
            for(int column=0;column<3;++column) {
                rotations0[offset+column]=float(pose.R(0,column));
                rotations1[offset+column]=float(pose.R(1,column));
                rotations2[offset+column]=float(pose.R(2,column));
            }
        }
        const GLsizei count=GLsizei(poses.size());
        Uniform3fv(pose_p_location,count,positions.data());
        Uniform3fv(pose_r0_location,count,rotations0.data());
        Uniform3fv(pose_r1_location,count,rotations1.data());
        Uniform3fv(pose_r2_location,count,rotations2.data());
    }

    std::vector<Image8> render(const std::vector<Pose>& poses,const std::vector<uint64_t>& seeds) {
        if(poses.empty() || poses.size()>4 || poses.size()!=seeds.size()) throw std::invalid_argument("VioSim GPU expects one to four poses and matching seeds");
        const bool profiling=config.profile_render;
        const auto start=profiling?Clock::now():Clock::time_point{};
        std::array<double,6> timings{};
        const size_t pixels=size_t(config.width)*config.height;
        std::vector<float> all_values(config.gpu_postprocess?0:pixels*output_layers);
        std::vector<uint8_t> all_bytes(config.gpu_postprocess?pixels*output_layers:0);
        {
            const auto wait_start=profiling?Clock::now():Clock::time_point{};
            std::lock_guard<std::mutex> lock(mutex);
            const auto acquired=profiling?Clock::now():Clock::time_point{};
            if(profiling) timings[0]=ms(wait_start,acquired);
            if(!eglMakeCurrent(display,surface,surface,context)) throw std::runtime_error("Cannot activate VioSim GPU context for rendering");
            try {
                UseProgram(program);Uniform1i(camera_count_location,GLint(poses.size()));
                set_pose_uniforms(poses);
                ActiveTexture(GL_TEXTURE0);BindTexture(GL_TEXTURE_2D_ARRAY,texture_array);BindImageTexture(0,output_texture,0,GL_TRUE,0,GL_WRITE_ONLY,GL_R32F);
                BindBufferBase(GL_SHADER_STORAGE_BUFFER,1,box_buffer);BindBufferBase(GL_SHADER_STORAGE_BUFFER,2,material_buffer);
                DispatchCompute(GLuint((config.width+7)/8),GLuint((config.height+7)/8),GLuint(poses.size()));
                MemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT|GL_TEXTURE_UPDATE_BARRIER_BIT);
                if(config.gpu_postprocess) postprocess(seeds);
                if(profiling) {
                    const auto wait_start=Clock::now();timings[1]=ms(acquired,wait_start);
                    Finish();timings[2]=ms(wait_start,Clock::now());
                }
                const auto read_start=profiling?Clock::now():Clock::time_point{};
                PixelStorei(GL_PACK_ALIGNMENT,1);
                if(config.gpu_postprocess) {
                    BindTexture(GL_TEXTURE_2D_ARRAY,byte_texture);
                    GetTexImage(GL_TEXTURE_2D_ARRAY,0,GL_RED_INTEGER,GL_UNSIGNED_BYTE,all_bytes.data());
                } else {
                    BindTexture(GL_TEXTURE_2D_ARRAY,output_texture);
                    GetTexImage(GL_TEXTURE_2D_ARRAY,0,GL_RED,GL_FLOAT,all_values.data());
                }
                if(profiling) {
                    timings[3]=ms(read_start,Clock::now());
                }
            } catch(...) {
                eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
                throw;
            }
            eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
        }
        const auto post_start=profiling?Clock::now():Clock::time_point{};
        std::vector<Image8> images;images.reserve(poses.size());
        for(size_t camera=0;camera<poses.size();++camera) {
            if(config.gpu_postprocess)
                images.push_back({config.width,config.height,std::vector<uint8_t>(all_bytes.begin()+camera*pixels,all_bytes.begin()+(camera+1)*pixels)});
            else images.push_back(finish_image(std::vector<float>(all_values.begin()+camera*pixels,all_values.begin()+(camera+1)*pixels),config,seeds[camera]));
        }
        if(profiling) {
            const auto end=Clock::now();timings[4]=ms(post_start,end);timings[5]=ms(start,end);
            std::lock_guard<std::mutex> lock(profile_mutex);++profile_count;
            for(size_t i=0;i<timings.size();++i) { profile_sum[i]+=timings[i];profile_max[i]=std::max(profile_max[i],timings[i]); }
        }
        return images;
    }

    void cleanup() noexcept {
        if(display==EGL_NO_DISPLAY) return;
        if(context!=EGL_NO_CONTEXT && surface!=EGL_NO_SURFACE) eglMakeCurrent(display,surface,surface,context);
        if(DeleteBuffers) { if(box_buffer)DeleteBuffers(1,&box_buffer);if(material_buffer)DeleteBuffers(1,&material_buffer);if(ray_buffer)DeleteBuffers(1,&ray_buffer); }
        if(DeleteTextures) { if(texture_array)DeleteTextures(1,&texture_array);if(output_texture)DeleteTextures(1,&output_texture); }
        if(DeleteProgram && program)DeleteProgram(program);
        if(DeleteProgram && post_program)DeleteProgram(post_program);
        if(DeleteBuffers && kernel_buffer)DeleteBuffers(1,&kernel_buffer);
        if(DeleteTextures && blur_texture)DeleteTextures(1,&blur_texture);
        if(DeleteTextures && byte_texture)DeleteTextures(1,&byte_texture);
        eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
        if(context!=EGL_NO_CONTEXT) eglDestroyContext(display,context);
        if(surface!=EGL_NO_SURFACE) eglDestroySurface(display,surface);
        eglTerminate(display);
        context=EGL_NO_CONTEXT;surface=EGL_NO_SURFACE;display=EGL_NO_DISPLAY;
    }
};

GpuRenderer::GpuRenderer(const Camera& camera,const Scene& scene,const TextureBank& textures,const Config& config)
    :impl_(std::make_unique<Impl>(camera,scene,textures,config)) {}
GpuRenderer::~GpuRenderer()=default;
std::vector<Image8> GpuRenderer::render(const std::vector<Pose>& poses,const std::vector<uint64_t>& seeds) const { return impl_->render(poses,seeds); }
const std::string& GpuRenderer::device_name() const { return impl_->name; }
std::string GpuRenderer::profile() const { return impl_->profile(); }

#else

struct GpuRenderer::Impl {};
std::string GpuRenderer::profile() const { return {}; }
GpuRenderer::GpuRenderer(const Camera&,const Scene&,const TextureBank&,const Config&) { throw std::runtime_error("VioSim GPU renderer is not available in this build"); }
GpuRenderer::~GpuRenderer()=default;
std::vector<Image8> GpuRenderer::render(const std::vector<Pose>&,const std::vector<uint64_t>&) const { throw std::runtime_error("VioSim GPU renderer is not available in this build"); }
const std::string& GpuRenderer::device_name() const { static const std::string empty;return empty; }

#endif

} // namespace vio
