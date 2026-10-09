#include "viosim/renderer.hpp"
#include "viosim/lab_room.hpp"
namespace vio {
Image8 render(const Camera& cam,const Scene& scene,const TextureBank& bank,const Pose& pose,const Config& cfg,uint64_t seed) {
    const int w=cam.width,h=cam.height,ss=cam.samples,sw=w*ss;
    const bool lab=scene.name=="lab_room";
    const int filter=cfg.texture_filter=="nearest"?0:(cfg.texture_filter=="bilinear"?1:2);
    std::vector<float> values(size_t(w)*h);
    // Each render invocation owns one image. There are no locks or allocations in this pixel loop.
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
        double sum=0;
        for(int sy=0;sy<ss;++sy) for(int sx=0;sx<ss;++sx) {
            const auto& ray=cam.rays[size_t(y*ss+sy)*sw+x*ss+sx];
            const Vec3 d=pose.R*Vec3(ray.d); const Hit hit=scene.trace(pose.p,d);
            if(hit.object<0) continue;
            const Material m=scene.material(hit); double intensity=m.value;
            if(m.texture>=0) {
                const auto axes=Scene::uv_axes(hit.face);
                const double u=hit.point[axes[0]]*m.us+m.uo,v=hit.point[axes[1]]*m.vs+m.vo;
                const auto& texture=bank.textures[size_t(m.texture)]; double lod=0;
                if(filter==2) {
                    const int normal_axis=hit.face/2;
                    if(std::abs(d[normal_axis])>1e-10) {
                        const Vec3 ddx=pose.R*Vec3(ray.dx),ddy=pose.R*Vec3(ray.dy);
                        // Ray-plane differentials give the UV footprint, including grazing angles.
                        const Vec3 px=(ddx-d*(ddx[normal_axis]/d[normal_axis]))*hit.distance;
                        const Vec3 py=(ddy-d*(ddy[normal_axis]/d[normal_axis]))*hit.distance;
                        const double tw=texture.levels.front().width*m.us,th=texture.levels.front().height*m.vs;
                        const double rho=std::max(std::hypot(px[axes[0]]*tw,px[axes[1]]*th),std::hypot(py[axes[0]]*tw,py[axes[1]]*th));
                        lod=std::log2(std::max(rho,1.0));
                    } else lod=double(texture.levels.size()-1);
                }
                intensity=texture.sample(u,v,lod,filter,m.repeat);
            }
            if(lab) intensity=lab_room_radiance(hit,pose.p,intensity,cfg.lab_reflections);
            if(cfg.distance_shading) intensity*=std::clamp(1-.02*hit.distance,.75,1.0);
            sum+=intensity;
        }
        const double vignette=lab?lab_room_vignette(cam,x,y,cfg.lab_vignette):1.0;
        values[size_t(y)*w+x]=float(sum/(ss*ss)*vignette);
    }
    // Optics before independent pixel read noise. This does not model temporal motion blur.
    gaussian_blur(values,w,h,std::hypot(cfg.sensor_blur,cfg.extra_blur));
    Noise rng(seed); Image8 image{w,h,std::vector<uint8_t>(size_t(w)*h)};
    for(size_t i=0;i<values.size();++i) {
        double v=std::clamp((values[i]-127.5)*cfg.contrast+127.5,0.0,255.0)*cfg.brightness;
        if(cfg.image_noise>0) v+=cfg.image_noise*rng.normal();
        image.data[i]=uint8_t(std::clamp(std::round(v),0.0,255.0));
    }
    return image;
}
GeometryImage render_geometry(const Camera& cam,const Scene& scene,const Pose& pose) {
    GeometryImage g{std::vector<float>(size_t(cam.width)*cam.height),{cam.width,cam.height,std::vector<uint8_t>(size_t(cam.width)*cam.height)}};
    for(int y=0;y<cam.height;++y) for(int x=0;x<cam.width;++x) {
        const auto h=scene.trace(pose.p,pose.R*cam.unproject(x,y)); const auto i=size_t(y)*cam.width+x;
        g.range[i]=h.object<0?0.0f:float(h.distance);
        g.surface_id.data[i]=h.object<0?0:uint8_t(1+6*h.object+h.face);
    }
    return g;
}
} // namespace vio
