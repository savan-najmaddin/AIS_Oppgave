#include "Objects.hpp"

MySpheres::MySpheres(const float radius, const threepp::Color color, const std::shared_ptr<threepp::Scene>& scene,
    const bool transparent, const float opacity)
    : m_radius(radius),
      m_color(color),
      m_transparent(transparent),
      m_opacity(opacity) {
    createMesh();
    addToScene(scene);
}

void MySpheres::createMesh() {

    m_geometry = threepp::SphereGeometry::create(m_radius);

    m_material = threepp::MeshBasicMaterial::create();
    m_material->color = m_color;
    m_material->transparent = m_transparent;
    m_material->opacity = m_opacity;

    m_mesh = threepp::Mesh::create(m_geometry, m_material);
}


float MySpheres::getRadius() const {
    return m_radius;
}

void MySpheres::setRadius(const float radius) {
    m_radius = radius;
}

std::shared_ptr<threepp::Mesh> MySpheres::getMesh() const {
    return m_mesh;
}

void MySpheres::addToScene(const std::shared_ptr<threepp::Scene> &scene) const {
    scene->add(m_mesh);
}
