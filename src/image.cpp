#include "viosim/image.hpp"
#include <png.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace vio {
Image8 read_png(const std::filesystem::path& file) {
    png_image image{}; image.version=PNG_IMAGE_VERSION;
    const auto name=file.string();
    if(!png_image_begin_read_from_file(&image,name.c_str())) throw std::runtime_error("Cannot open PNG: "+name+": "+image.message);
    image.format=PNG_FORMAT_GRAY;
    if(image.width>16384 || image.height>16384 || uint64_t(image.width)*image.height>100000000) {
        png_image_free(&image); throw std::runtime_error("Texture dimensions are too large");
    }
    Image8 result{int(image.width),int(image.height),std::vector<uint8_t>(PNG_IMAGE_SIZE(image))};
    if(!png_image_finish_read(&image,nullptr,result.data.data(),0,nullptr)) {
        const std::string error=image.message; png_image_free(&image); throw std::runtime_error("Cannot decode PNG: "+error);
    }
    png_image_free(&image); return result;
}
void write_png(const std::filesystem::path& file,const Image8& image,int compression) {
    const auto name=file.string();
    if(image.data.size()!=size_t(image.width)*image.height) throw std::runtime_error("Invalid image size for PNG writer");
    // Allocate C++ containers before setjmp; libpng cannot jump over their construction.
    std::vector<png_bytep> rows(size_t(image.height));
    for(int y=0;y<image.height;++y) rows[size_t(y)]=const_cast<png_bytep>(image.data.data()+size_t(y)*image.width);
    FILE* f=std::fopen(name.c_str(),"wb"); if(!f) throw std::runtime_error("Cannot create PNG: "+name);
    png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING,nullptr,nullptr,nullptr);
    if(!png) { std::fclose(f); throw std::runtime_error("png_create_write_struct failed"); }
    png_infop info=png_create_info_struct(png);
    if(!info) { png_destroy_write_struct(&png,nullptr); std::fclose(f); throw std::runtime_error("png_create_info_struct failed"); }
    if(setjmp(png_jmpbuf(png))) { png_destroy_write_struct(&png,&info); std::fclose(f); throw std::runtime_error("PNG write failed: "+name); }
    png_init_io(png,f);
    png_set_IHDR(png,info,png_uint_32(image.width),png_uint_32(image.height),8,PNG_COLOR_TYPE_GRAY,PNG_INTERLACE_NONE,PNG_COMPRESSION_TYPE_BASE,PNG_FILTER_TYPE_BASE);
    png_set_compression_level(png,compression);
    png_set_filter(png,PNG_FILTER_TYPE_BASE,PNG_FILTER_SUB);
    png_write_info(png,info); png_write_image(png,rows.data()); png_write_end(png,nullptr);
    png_destroy_write_struct(&png,&info);
    if(std::fclose(f)!=0) throw std::runtime_error("Failed to close PNG: "+name);
}
void write_pgm(const std::filesystem::path& file,const Image8& image) {
    std::ofstream f(file,std::ios::binary); f.exceptions(std::ios::badbit|std::ios::failbit);
    f<<"P5\n"<<image.width<<' '<<image.height<<"\n255\n";
    f.write(reinterpret_cast<const char*>(image.data.data()),std::streamsize(image.data.size()));
}
void write_pfm(const std::filesystem::path& file,int width,int height,const std::vector<float>& data) {
    std::ofstream f(file,std::ios::binary); f.exceptions(std::ios::badbit|std::ios::failbit);
    f<<"Pf\n"<<width<<' '<<height<<"\n-1.0\n";
    // PFM is bottom-to-top. Negative scale specifies little-endian 32-bit floats.
    for(int y=height-1;y>=0;--y) for(int x=0;x<width;++x) {
        uint32_t bits=0; const float value=data[size_t(y)*width+x]; std::memcpy(&bits,&value,4);
        const char bytes[]={char(bits&255),char((bits>>8)&255),char((bits>>16)&255),char((bits>>24)&255)};
        f.write(bytes,4);
    }
}
static int reflect101(int x,int n) {
    if(n<=1) return 0;
    while(x<0 || x>=n) { if(x<0) x=-x; if(x>=n) x=2*n-2-x; }
    return x;
}
void gaussian_blur(std::vector<float>& a,int w,int h,double sigma) {
    if(sigma<=1e-5) return;
    const int radius=std::max(1,int(std::ceil(3*sigma)));
    std::vector<float> k(size_t(2*radius+1)); double total=0;
    for(int i=-radius;i<=radius;++i) { const double v=std::exp(-.5*i*i/(sigma*sigma)); k[size_t(i+radius)]=float(v); total+=v; }
    for(auto& v:k) v=float(v/total);
    std::vector<float> b(a.size());
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
        double s=0; for(int i=-radius;i<=radius;++i) s+=k[size_t(i+radius)]*a[size_t(y)*w+reflect101(x+i,w)];
        b[size_t(y)*w+x]=float(s);
    }
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
        double s=0; for(int i=-radius;i<=radius;++i) s+=k[size_t(i+radius)]*b[size_t(reflect101(y+i,h))*w+x];
        a[size_t(y)*w+x]=float(s);
    }
}
Texture::Texture(const Image8& image) {
    TextureLevel first{image.width,image.height,{}}; first.data.assign(image.data.begin(),image.data.end()); levels.push_back(std::move(first));
    while(levels.back().width>1 || levels.back().height>1) {
        const auto& src=levels.back(); TextureLevel dst{std::max(1,src.width/2),std::max(1,src.height/2),{}};
        dst.data.resize(size_t(dst.width)*dst.height);
        for(int y=0;y<dst.height;++y) for(int x=0;x<dst.width;++x) {
            double s=0; for(int j=0;j<2;++j) for(int i=0;i<2;++i) s+=src.data[size_t(std::min(2*y+j,src.height-1))*src.width+std::min(2*x+i,src.width-1)];
            dst.data[size_t(y)*dst.width+x]=float(.25*s);
        }
        levels.push_back(std::move(dst));
    }
}
static double bilinear(const TextureLevel& t,double u,double v,bool repeat) {
    if(repeat) { u-=std::floor(u); v-=std::floor(v); }
    else { u=std::clamp(u,0.0,1.0); v=std::clamp(v,0.0,1.0); }
    const double x=u*t.width-.5,y=v*t.height-.5;
    const int ix=int(std::floor(x)),iy=int(std::floor(y));
    const double ax=x-ix,ay=y-iy;
    auto index=[repeat](int i,int n) { if(repeat) { i%=n; return i<0?i+n:i; } return std::clamp(i,0,n-1); };
    const int x0=index(ix,t.width),x1=index(ix+1,t.width),y0=index(iy,t.height),y1=index(iy+1,t.height);
    return (1-ay)*((1-ax)*t.data[size_t(y0)*t.width+x0]+ax*t.data[size_t(y0)*t.width+x1])+ay*((1-ax)*t.data[size_t(y1)*t.width+x0]+ax*t.data[size_t(y1)*t.width+x1]);
}
double Texture::sample(double u,double v,double lod,int filter,bool repeat) const {
    if(filter==0) {
        const auto& t=levels.front();
        if(repeat) { u-=std::floor(u); v-=std::floor(v); }
        const int x=std::clamp(int(std::clamp(u,0.0,1.0)*(t.width-1)),0,t.width-1),y=std::clamp(int(std::clamp(v,0.0,1.0)*(t.height-1)),0,t.height-1);
        return t.data[size_t(y)*t.width+x];
    }
    if(filter==1) return bilinear(levels.front(),u,v,repeat);
    lod=std::clamp(lod,0.0,double(levels.size()-1));
    const size_t a=size_t(lod),b=std::min(a+1,levels.size()-1); const double t=lod-double(a);
    return (1-t)*bilinear(levels[a],u,v,repeat)+t*bilinear(levels[b],u,v,repeat);
}
TextureBank::TextureBank(const std::filesystem::path& root,bool with_lab_room) {
    const std::vector<std::string> names={"wall_panel","floor_tiles","ceiling_tiles","poster_light","poster_dark","door","crate_a","crate_b","crate_c","corridor_wall","warehouse_wall"};
    for(const auto& name:names) textures.emplace_back(read_png(root/(name+".png")));
    // Canonical tag36h11 ID 0, including its one-cell white margin. Pattern
    // from the vendored AprilTag tag36h11.c / apriltag_to_image convention.
    // Eight cells span the physical tag size; ten span the complete texture.
    static const char* tag[]={
        "1111111111", "1000000001", "1011010101", "1001110101", "1001100001",
        "1010100001", "1001011001", "1000010001", "1000000001", "1111111111"};
    constexpr int cell=32, side=10*cell;
    Image8 marker{side,side,std::vector<uint8_t>(side*side)};
    for(int y=0;y<side;++y) for(int x=0;x<side;++x)
        marker.data[size_t(y)*side+x]=tag[y/cell][x/cell]=='1'?255:0;
    textures.emplace_back(marker);
    if(with_lab_room) {
        const std::vector<std::string> lab_names={"lab_floor","lab_plaster","lab_metal","lab_cabinet","lab_lockers_a","lab_lockers_b","lab_shelves","lab_cardboard","lab_instrument","lab_drawers","lab_whiteboard","lab_opening"};
        for(const auto& name:lab_names) textures.emplace_back(read_png(root/(name+".png")));
    }
}
} // namespace vio
