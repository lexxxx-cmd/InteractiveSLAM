#include "glk/primitives.hpp"
#include "glk/mesh.hpp"

namespace glk {

Primitives* Primitives::instance_ = nullptr;

Primitives::Primitives() {
    meshes.resize(NUM_PRIMITIVES, nullptr);
}

Primitives* Primitives::instance() {
    if (instance_ == nullptr) {
        instance_ = new Primitives();
    }
    return instance_;
}

static Mesh buildCoordinateSystem() {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
    std::vector<int> indices;

    // X-axis (red) — 3 segments
    vertices.push_back(Eigen::Vector3f(0, 0, 0));
    normals.push_back(Eigen::Vector3f(1, 0, 0));
    vertices.push_back(Eigen::Vector3f(1, 0, 0));
    normals.push_back(Eigen::Vector3f(1, 0, 0));
    vertices.push_back(Eigen::Vector3f(0.9f, 0.05f, 0));
    normals.push_back(Eigen::Vector3f(1, 0, 0));
    vertices.push_back(Eigen::Vector3f(0.9f, -0.05f, 0));
    normals.push_back(Eigen::Vector3f(1, 0, 0));
    indices.insert(indices.end(), {0, 1, 1, 2, 1, 3});

    // Y-axis (green)
    vertices.push_back(Eigen::Vector3f(0, 0, 0));
    normals.push_back(Eigen::Vector3f(0, 1, 0));
    vertices.push_back(Eigen::Vector3f(0, 1, 0));
    normals.push_back(Eigen::Vector3f(0, 1, 0));
    vertices.push_back(Eigen::Vector3f(0.05f, 0.9f, 0));
    normals.push_back(Eigen::Vector3f(0, 1, 0));
    vertices.push_back(Eigen::Vector3f(-0.05f, 0.9f, 0));
    normals.push_back(Eigen::Vector3f(0, 1, 0));
    indices.insert(indices.end(), {4, 5, 5, 6, 5, 7});

    // Z-axis (blue)
    vertices.push_back(Eigen::Vector3f(0, 0, 0));
    normals.push_back(Eigen::Vector3f(0, 0, 1));
    vertices.push_back(Eigen::Vector3f(0, 0, 1));
    normals.push_back(Eigen::Vector3f(0, 0, 1));
    vertices.push_back(Eigen::Vector3f(0.05f, 0, 0.9f));
    normals.push_back(Eigen::Vector3f(0, 0, 1));
    vertices.push_back(Eigen::Vector3f(-0.05f, 0, 0.9f));
    normals.push_back(Eigen::Vector3f(0, 0, 1));
    indices.insert(indices.end(), {8, 9, 9, 10, 9, 11});

    return Mesh(vertices, normals, indices);
}

static Mesh buildGrid(int size = 10) {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
    std::vector<int> indices;

    for (int i = -size; i <= size; i++) {
        vertices.push_back(Eigen::Vector3f((float)i, 0, (float)-size));
        normals.push_back(Eigen::Vector3f(0, 1, 0));
        vertices.push_back(Eigen::Vector3f((float)i, 0, (float)size));
        normals.push_back(Eigen::Vector3f(0, 1, 0));
        indices.push_back((int)vertices.size() - 2);
        indices.push_back((int)vertices.size() - 1);

        vertices.push_back(Eigen::Vector3f((float)-size, 0, (float)i));
        normals.push_back(Eigen::Vector3f(0, 1, 0));
        vertices.push_back(Eigen::Vector3f((float)size, 0, (float)i));
        normals.push_back(Eigen::Vector3f(0, 1, 0));
        indices.push_back((int)vertices.size() - 2);
        indices.push_back((int)vertices.size() - 1);
    }

    return Mesh(vertices, normals, indices);
}

static Mesh buildSphere(float radius = 0.2f, int rings = 12, int sectors = 12) {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
    std::vector<int> indices;

    for (int r = 0; r <= rings; r++) {
        float phi = (float)r * 3.14159265f / (float)rings;
        for (int s = 0; s <= sectors; s++) {
            float theta = (float)s * 2.0f * 3.14159265f / (float)sectors;
            float x = radius * sinf(phi) * cosf(theta);
            float y = radius * cosf(phi);
            float z = radius * sinf(phi) * sinf(theta);
            vertices.push_back(Eigen::Vector3f(x, y, z));
            normals.push_back(Eigen::Vector3f(x, y, z).normalized());
        }
    }

    for (int r = 0; r < rings; r++) {
        for (int s = 0; s < sectors; s++) {
            int a = r * (sectors + 1) + s;
            int b = a + sectors + 1;
            indices.insert(indices.end(), {a, b, a + 1, b, b + 1, a + 1});
        }
    }

    return Mesh(vertices, normals, indices);
}

const Drawable& Primitives::primitive(PrimitiveType type) {
    if (!meshes[type]) {
        switch (type) {
        case COORDINATE_SYSTEM:
            meshes[type] = std::make_shared<Mesh>(buildCoordinateSystem());
            break;
        case GRID:
            meshes[type] = std::make_shared<Mesh>(buildGrid());
            break;
        case SPHERE:
            meshes[type] = std::make_shared<Mesh>(buildSphere());
            break;
        default:
            meshes[type] = std::make_shared<Mesh>(buildSphere());
            break;
        }
    }
    return *meshes[type];
}

}  // namespace glk
