#include "viewport/graph_viewport.hpp"
#include "viewport/graph_viewport_renderer.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GraphViewport::GraphViewport(QQuickItem* parent)
    : QQuickFramebufferObject(parent) {}

GraphManager* GraphViewport::graphManager() const { return m_graphManager; }
void GraphViewport::setGraphManager(GraphManager* manager) {
    if (m_graphManager != manager) {
        m_graphManager = manager;
        emit graphManagerChanged();
    }
}

void GraphViewport::receiveFpsUpdate(float fps) {
    m_currentFps = fps;
    emit fpsUpdated(fps);
}

void GraphViewport::onMouseRotate(float dx, float dy) {
    m_camTheta -= dx * 0.01;
    m_camPhi   -= dy * 0.01;
    m_camPhi    = std::clamp(m_camPhi, -1.55, 1.55);
    m_cameraDirty = true;
    update();
}

void GraphViewport::onMousePan(float dx, float dy) {
    double theta_rad = m_camTheta + M_PI / 2.0;
    float cx = std::cos(theta_rad);
    float cy = std::sin(theta_rad);
    m_camCenter.x() += (-cx * dx + cy * dy) * m_camDistance * 0.001f;
    m_camCenter.y() += (-cy * dx - cx * dy) * m_camDistance * 0.001f;
    m_cameraDirty = true;
    update();
}

void GraphViewport::onMouseZoom(float delta) {
    if (delta > 0) m_camDistance *= 0.9;
    else           m_camDistance *= 1.1;
    m_camDistance = std::max(0.1, m_camDistance);
    m_cameraDirty = true;
    update();
}

void GraphViewport::resetCamera() {
    m_camCenter   = Eigen::Vector3f(0, 0, 0);
    m_camDistance = 10.0;
    m_camTheta    = -90.0 * M_PI / 180.0;   // look along -Y (typical top-down)
    m_camPhi      =  60.0 * M_PI / 180.0;   // high above, tilting down
    m_cameraDirty = true;
    update();
}

void GraphViewport::fitView(const Eigen::Vector3f& bboxMin,
                              const Eigen::Vector3f& bboxMax) {
    m_camCenter = (bboxMin + bboxMax) * 0.5f;
    float diag = (bboxMax - bboxMin).norm();
    m_camDistance = std::max(1.0, (double)(diag * 1.5));
    m_camTheta = -90.0 * M_PI / 180.0;   // look along -Y
    m_camPhi   =  60.0 * M_PI / 180.0;   // above, tilting down
    m_cameraDirty = true;
    update();
}

Eigen::Matrix4f GraphViewport::cameraViewMatrix() const {
    double theta = m_camTheta, phi = m_camPhi, dist = m_camDistance;
    Eigen::Vector3f center = m_camCenter;

    // Spherical → Cartesian eye position
    Eigen::Vector3f eye;
    eye.x() = center.x() + dist * std::cos(phi) * std::cos(theta);
    eye.y() = center.y() + dist * std::cos(phi) * std::sin(theta);
    eye.z() = center.z() + dist * std::sin(phi);

    // LookAt: eye → center, Z-up
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f u = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f s = f.cross(u).normalized();
    Eigen::Vector3f u2 = s.cross(f).normalized();

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 0) = s.x();   mat(0, 1) = s.y();   mat(0, 2) = s.z();   mat(0, 3) = -s.dot(eye);
    mat(1, 0) = u2.x();  mat(1, 1) = u2.y();  mat(1, 2) = u2.z();  mat(1, 3) = -u2.dot(eye);
    mat(2, 0) = -f.x();  mat(2, 1) = -f.y();  mat(2, 2) = -f.z();  mat(2, 3) = f.dot(eye);
    return mat;
}

void GraphViewport::setSelectedVertexId(int id) {
    if (m_selectedVertexId != id) {
        m_selectedVertexId = id;
        emit selectedVertexIdChanged();
        update();
    }
}

void GraphViewport::requestPick(float mouseX, float mouseY) {
    m_pickPending = true;
    m_pickMouseX = mouseX;
    m_pickMouseY = mouseY;
    update();  // trigger render → synchronize → render with pick
}

QQuickFramebufferObject::Renderer* GraphViewport::createRenderer() const {
    return new GraphViewportRenderer();
}
