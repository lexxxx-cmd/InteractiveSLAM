#pragma once

#include <memory>
#include "glk/pointcloud_buffer.hpp"
#include "glk/primitives.hpp"
#include "viewport/drawable_object.hpp"
#include "viewport/vertex_view.hpp"
#include "data/hdl_graph_slam/interactive_keyframe.hpp"

namespace hdl_graph_slam {

class KeyFrameView : public VertexView {
public:
    using Ptr = std::shared_ptr<KeyFrameView>;

    KeyFrameView(const InteractiveKeyFrame::Ptr& kf)
        : VertexView(kf->node), keyframe(kf) {
        pointcloud_buffer.reset(new glk::PointCloudBuffer(kf->cloud));
    }

    InteractiveKeyFrame::Ptr lock() const { return keyframe.lock(); }

    virtual bool available() const override { return !keyframe.expired(); }

    virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
        InteractiveKeyFrame::Ptr kf = keyframe.lock();
        if (!kf) return;
        Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();

        // Point cloud (color_mode=0: rainbow from Z height)
        shader.set_uniform("color_mode", 0);
        shader.set_uniform("model_matrix", model_matrix);
        shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));
        pointcloud_buffer->draw(shader);

        if (!flags.draw_verticies || !flags.draw_keyframe_vertices) return;

        // Keyframe sphere (color_mode=1: solid red)
        shader.set_uniform("color_mode", 1);
        shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
        shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));
        shader.set_uniform("apply_keyframe_scale", true);
        Eigen::Matrix4f sphere_model = model_matrix;
        sphere_model.block<3, 3>(0, 0) *= 0.35f;
        shader.set_uniform("model_matrix", sphere_model);
        const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
        sphere.draw(shader);
        shader.set_uniform("apply_keyframe_scale", false);
    }

    virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader,
                      const Eigen::Vector4f& color,
                      const Eigen::Matrix4f& model_matrix) override {
        if (!available()) return;
        InteractiveKeyFrame::Ptr kf = keyframe.lock();
        if (!kf) return;

        shader.set_uniform("color_mode", 1);
        shader.set_uniform("material_color", color);
        shader.set_uniform("model_matrix", model_matrix);
        shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));
        pointcloud_buffer->draw(shader);
    }

private:
    std::weak_ptr<InteractiveKeyFrame> keyframe;
    std::unique_ptr<glk::PointCloudBuffer> pointcloud_buffer;
};

}  // namespace hdl_graph_slam
