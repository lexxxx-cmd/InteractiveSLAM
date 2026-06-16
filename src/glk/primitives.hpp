#pragma once

#include <vector>
#include <memory>
#include "glk/drawble.hpp"

namespace glk {

class Primitives {
public:
    enum PrimitiveType {
        ICOSAHEDRON = 0,
        SPHERE,
        CUBE,
        CONE,
        GRID,
        COORDINATE_SYSTEM,
        NUM_PRIMITIVES
    };

    static Primitives* instance();

    const glk::Drawable& primitive(PrimitiveType type);

private:
    Primitives();
    static Primitives* instance_;
    std::vector<std::shared_ptr<glk::Drawable>> meshes;
};

}  // namespace glk
