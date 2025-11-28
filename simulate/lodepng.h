#pragma once

#include <cstddef>
#include <string>

// Minimal enum to satisfy simulate's screenshot integration.
enum LodePNGColorType {
  LCT_RGB = 2
};

namespace lodepng {

unsigned encode(const std::string& filename,
                const unsigned char* image,
                unsigned width,
                unsigned height,
                LodePNGColorType color_type);

const char* error_text(unsigned code);

}  // namespace lodepng

