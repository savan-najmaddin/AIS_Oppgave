#include "Objects.hpp"

MySpheres::MySpheres(float radius, int widthSegments, int heightSegments,
                      threepp::Color color, bool transparent, float opacity)
    : m_radius(radius),
      m_widthSegments(widthSegments),
      m_heightSegments(heightSegments),
      m_color(color),
      m_transparent(transparent),
      m_opacity(opacity) {
        createMesh();
    }

void MySpheres::createMesh() {

    m_geometry = threepp::SphereGeometry::create(m_radius, m_heightSegments, m_widthSegments);

    m_material = threepp::MeshBasicMaterial::create();
    m_material->color = m_color;
    m_material->transparent = m_transparent;
    m_material->opacity = m_opacity;

    m_mesh = threepp::Mesh::create(m_geometry, m_material);
}



float MySpheres::getRadius() {
    return m_radius;
}

void MySpheres::setRadius(float radius) {
    m_radius = radius;
}

std::shared_ptr<threepp::Mesh> MySpheres::getMesh() {
    return m_mesh;
}

void MySpheres::addToScene(const std::shared_ptr<threepp::Scene> &scene) {
    scene->add(m_mesh);
}

