#pragma once

namespace vio {
// Same SplitMix64 stream and Box-Muller pairing as Noise, indexed by pixel.
// Two 32-bit words avoid requiring an optional GLSL int64 extension.
inline constexpr const char* postprocess_shader=R"GLSL(#version 430
layout(local_size_x=8,local_size_y=8) in;
layout(r32f,binding=0) uniform readonly image2DArray source_image;
layout(r32f,binding=1) uniform writeonly image2DArray blurred_image;
layout(r8ui,binding=2) uniform writeonly uimage2DArray final_image;
layout(std430,binding=0) readonly buffer Kernel { float weights[]; };
uniform int width,height,radius,pass;
uniform float contrast,brightness,noise_sigma;
uniform uvec2 seeds[6];
uvec2 add64(uvec2 a,uvec2 b) { uint lo=a.x+b.x;return uvec2(lo,a.y+b.y+uint(lo<a.x)); }
uvec2 mul64(uvec2 a,uvec2 b) {
    uint hi,lo;umulExtended(a.x,b.x,hi,lo);
    return uvec2(lo,hi+a.x*b.y+a.y*b.x);
}
uvec2 shr64(uvec2 a,uint n) { return uvec2((a.x>>n)|(a.y<<(32u-n)),a.y>>n); }
uvec2 mix64(uvec2 a) {
    a=mul64(a^shr64(a,30u),uvec2(0x1ce4e5b9u,0xbf58476du));
    a=mul64(a^shr64(a,27u),uvec2(0x133111ebu,0x94d049bbu));
    return a^shr64(a,31u);
}
float uniform_value(uvec2 state) {
    uvec2 h=mix64(state);
    return max(float(h.y)*2.3283064365386963e-10+float(h.x>>11u)*1.1102230246251565e-16,5.551115123125783e-17);
}
float gaussian(uint pixel,uint camera) {
    uvec2 gamma=uvec2(0x7f4a7c15u,0x9e3779b9u);
    uvec2 state=add64(seeds[camera],mul64(gamma,uvec2((pixel/2u)*2u+1u,0)));
    float a=sqrt(-2.0*log(uniform_value(state)));
    float b=6.283185307179586*uniform_value(add64(state,gamma));
    return a*((pixel&1u)==0u?cos(b):sin(b));
}
int reflect101(int p,int size) {
    while(p<0 || p>=size) { if(p<0)p=-p;if(p>=size)p=2*size-2-p; }
    return p;
}
void main() {
    ivec3 p=ivec3(gl_GlobalInvocationID);
    if(p.x>=width || p.y>=height) return;
    float value=0;
    for(int i=-radius;i<=radius;++i) {
        ivec3 q=p;
        if(pass==0) q.x=reflect101(p.x+i,width);
        else q.y=reflect101(p.y+i,height);
        value+=weights[i+radius]*imageLoad(source_image,q).r;
    }
    if(pass==0) { imageStore(blurred_image,p,vec4(value));return; }
    value=clamp((value-127.5)*contrast+127.5,0.0,255.0)*brightness;
    if(noise_sigma>0) value+=noise_sigma*gaussian(uint(p.y*width+p.x),uint(p.z));
    imageStore(final_image,p,uvec4(uint(clamp(floor(value+0.5),0.0,255.0))));
}
)GLSL";
}
