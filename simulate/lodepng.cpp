#include "lodepng.h"

namespace lodepng {

unsigned encode(const std::string& /*filename*/,
                const unsigned char* /*image*/,
                unsigned /*width*/,
                unsigned /*height*/,
                LodePNGColorType /*color_type*/) {
  // Stub implementation: PNG encoding is not available in this trimmed build.
  return 1;
}

const char* error_text(unsigned /*code*/) {
  return "lodepng functionality is stubbed out in this build";
}

}  // namespace lodepng

