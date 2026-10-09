#pragma once
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
namespace vio {
struct Image8 { int width=0,height=0; std::vector<uint8_t> data; };
Image8 read_png(const std::filesystem::path&);
void write_png(const std::filesystem::path&,const Image8&,int compression=1);
void write_pgm(const std::filesystem::path&,const Image8&);
void write_pfm(const std::filesystem::path&,int width,int height,const std::vector<float>&);
void gaussian_blur(std::vector<float>& pixels,int width,int height,double sigma);
struct TextureLevel { int width=0,height=0; std::vector<float> data; };
struct Texture {
    std::vector<TextureLevel> levels;
    explicit Texture(const Image8&);
    double sample(double u,double v,double lod,int filter,bool repeat) const;
};
struct TextureBank {
    std::vector<Texture> textures;
    explicit TextureBank(const std::filesystem::path&,bool with_lab_room=false);
};
enum TextureId { WALL,FLOOR,CEILING,POSTER_LIGHT,POSTER_DARK,DOOR,CRATE_A,CRATE_B,CRATE_C,CORRIDOR_WALL,WAREHOUSE_WALL,APRILTAG_36H11_0,
    LAB_FLOOR,LAB_PLASTER,LAB_METAL,LAB_CABINET,LAB_LOCKERS_A,LAB_LOCKERS_B,
    LAB_SHELVES,LAB_CARDBOARD,LAB_INSTRUMENT,LAB_DRAWERS,LAB_WHITEBOARD,LAB_OPENING };
} // namespace vio
