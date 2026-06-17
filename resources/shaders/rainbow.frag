#version 330 core
uniform vec2 z_range;
uniform int z_clipping;
uniform int color_mode;

in vec4 frag_color;
flat in ivec4 frag_info;
in vec3 frag_world_position;

layout(location = 0) out vec4 color;
layout(location = 1) out vec4 info_encoded;

// Pack two ints (type|flags in .x, vertex_id in .y) into four bytes of RGBA8.
// Both attachments are standard GL_RGBA8 — no integer formats, no driver issues.
vec4 packId(ivec4 info) {
    return vec4(
        float(info.x & 0xFF)        / 255.0,
        float((info.x >> 8) & 0xFF) / 255.0,
        float(info.y & 0xFF)        / 255.0,
        float((info.y >> 8) & 0xFF) / 255.0
    );
}

void main() {
    if(z_clipping != 0 && color_mode == 0 && (frag_world_position.z < z_range[0] || frag_world_position.z > z_range[1])) {
        discard;
    }

    color         = frag_color;
    info_encoded  = packId(frag_info);
}