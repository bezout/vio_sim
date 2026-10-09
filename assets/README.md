# Included textures

These eleven grayscale PNGs were generated procedurally for the earlier Python
package and are reused by this C++ port. They are included under the package MIT
license. No externally downloaded photographs or redistributable font files are
part of the package. Some text is already rasterized into the poster images.

The runtime expects these filenames under the texture directory:

wall_panel.png, floor_tiles.png, ceiling_tiles.png, poster_light.png,
poster_dark.png, door.png, crate_a.png, crate_b.png, crate_c.png,
corridor_wall.png, warehouse_wall.png.

Mipmap pyramids are computed when the textures are loaded. Built-in images are
512x512, but replacement PNGs do not have to share the same resolution.

The twelve `lab_*.png` textures were generated procedurally for the `lab_room`
environment with seed 61728. They contain generic grayscale patterns and labels;
no pixels, fonts or other assets from the reference photograph are included.
They are loaded only when `lab_room` is selected. The supplied PNG bytes should
be retained for reproducible renders because regeneration can depend on the
NumPy and Pillow versions used by the authoring script.
