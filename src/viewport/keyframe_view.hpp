#pragma once

#include <memory>
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/filters/voxel_grid.h>
#include "glk/pointcloud_buffer.hpp"
#include "glk/primitives.hpp"
#include "viewport/drawable_object.hpp"
#include "viewport/vertex_view.hpp"
#include "data/hdl_graph_slam/interactive_keyframe.hpp"

namespace hdl_graph_slam {

class KeyFrameView : public VertexView {
public:
    using Ptr = std::shared_ptr<KeyFrameView>;

    KeyFrameView(const InteractiveKeyFrame::Ptr& kf, float voxelLeafSize = 0.1f)
        : VertexView(kf->node), keyframe(kf) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud;

        // Step 1: optional initial downsampling
        pcl::PointCloud<pcl::PointXYZI>::Ptr step1 =
            std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        if (voxelLeafSize > 0.0f && kf->cloud->size() > 50000) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
            voxel.setInputCloud(kf->cloud);
            voxel.filter(*step1);
        } else {
            *step1 = *kf->cloud;  // copy (small cloud, cheap)
        }

        // Step 2: auto-raise leaf size if still too many points
        if (static_cast<int>(step1->size()) > 2000000) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel2;
            float biggerLeaf = voxelLeafSize
                * std::ceil(step1->size() / 2000000.0f);
            voxel2.setLeafSize(biggerLeaf, biggerLeaf, biggerLeaf);
            voxel2.setInputCloud(kf->cloud);  // re-filter from original
            finalCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            voxel2.filter(*finalCloud);
        } else {
            finalCloud = step1;
        }

        m_num_points = static_cast<int>(finalCloud->size());
        pointcloud_buffer.reset(new glk::PointCloudBuffer(finalCloud));

        // Compute bounding sphere from the cloud actually uploaded to GPU
        {
            Eigen::Vector3f mn(+1e9, +1e9, +1e9);
            Eigen::Vector3f mx(-1e9, -1e9, -1e9);
            for (size_t i = 0; i < finalCloud->size(); ++i) {
                const auto& pt = finalCloud->points[i];
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                mn = mn.cwiseMin(p);
                mx = mx.cwiseMax(p);
            }
            Eigen::Vector3f center = (mn + mx) * 0.5f;
            float maxDist = 0.0f;
            for (size_t i = 0; i < finalCloud->size(); ++i) {
                const auto& pt = finalCloud->points[i];
                float d = (Eigen::Vector3f(pt.x, pt.y, pt.z) - center).squaredNorm();
                if (d > maxDist) maxDist = d;
            }
            m_boundingSphereRadius = std::sqrt(maxDist) + 1.0f;
        }
    }

    int pointCount() const override { return m_num_points; }

    Eigen::Vector3f boundingSphereCenter() const override {
        auto kf = keyframe.lock();
        if (!kf) return {0, 0, 0};
        return kf->estimate().translation().cast<float>();
    }

    float boundingSphereRadius() const override {
        return m_boundingSphereRadius;
    }

    InteractiveKeyFrame::Ptr lock() const { return keyframe.lock(); }

    virtual bool available() const override { return !keyframe.expired(); }

    virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
        InteractiveKeyFrame::Ptr kf = keyframe.lock();
        if (!kf) return;
        Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();
        int vid = static_cast<int>(kf->id());

        bool isHighlighted = flags.highlightPass
            && flags.highlightedSet
            && flags.highlightedSet->count({VERTEX | KEYFRAME, vid});

        // Point cloud — skip in highlight pass (wireframe cloud is messy)
        if (!flags.highlightPass) {
            if (kf->id() == flags.selectedVertexId) {
                shader.set_uniform("color_mode", 1);
                shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.55f, 0.0f, 1.0f));
            } else {
                shader.set_uniform("color_mode", 0);
            }
            shader.set_uniform("model_matrix", model_matrix);
            shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));
            pointcloud_buffer->draw(shader);
        }

        if (!flags.draw_verticies || !flags.draw_keyframe_vertices) return;

        // Sphere
        shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | KEYFRAME, vid, 0, 0));
        if (isHighlighted) {
            // Bright yellow — stands out against red spheres and rainbow clouds
            shader.set_uniform("color_mode", 1);
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 0.0f, 1.0f));
            shader.set_uniform("apply_keyframe_scale", true);
            Eigen::Matrix4f sphere_model = model_matrix;
            sphere_model.block<3, 3>(0, 0) *= 2.0f;
            shader.set_uniform("model_matrix", sphere_model);
        } else {
            shader.set_uniform("color_mode", 1);
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
            shader.set_uniform("apply_keyframe_scale", true);
            Eigen::Matrix4f sphere_model = model_matrix;
            sphere_model.block<3, 3>(0, 0) *= 2.0f;
            shader.set_uniform("model_matrix", sphere_model);
        }
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
    int m_num_points = 0;
    float m_boundingSphereRadius = 100.0f;
};

}  // namespace hdl_graph_slam
