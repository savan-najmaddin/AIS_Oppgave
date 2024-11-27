#include "Objects.hpp"


MySpheres::MySpheres(float radius, int widthSegments, int heightSegments, threepp::Color color, bool transparent, float opacity)
    : m_radius(radius),
      m_widthSegments(widthSegments),
      m_heightSegments(heightSegments),
      m_color(color),
      m_material(threepp::MeshBasicMaterial::create()) {
      m_material->color = color;
    m_material->transparent = transparent;
    m_material->opacity = opacity;
}

void MySpheres::setRadius(float radius) {
    m_radius = radius;
}

float MySpheres::getRadius() const {
    return m_radius;
}

int MySpheres::getWidthSegments() const {
    return m_widthSegments;
}

int MySpheres::getHeightSegments() const {
    return m_heightSegments;
}

threepp::Color MySpheres::getColor() {
    return m_color;
}

void MySpheres::setMaterial(std::shared_ptr<threepp::MeshBasicMaterial> material) {
    m_material = material;
}

std::shared_ptr<threepp::MeshBasicMaterial> MySpheres::getMaterial() const {
    return m_material;
}

void MySpheres::setMesh(std::shared_ptr<threepp::Mesh> mesh) {
    m_mesh = mesh;
}

std::shared_ptr<threepp::Mesh> MySpheres::getMesh() const {
    return m_mesh;
}



std::shared_ptr<threepp::Mesh> SphereInitializer::createSphere(MySpheres &sphere) {
    std::shared_ptr<threepp::SphereGeometry> geometry = threepp::SphereGeometry::create(
            sphere.getRadius(), sphere.getHeightSegments(), sphere.getWidthSegments());

    std::shared_ptr<threepp::MeshBasicMaterial> material = sphere.getMaterial();
    material->color = sphere.getColor();

    std::shared_ptr<threepp::Mesh> mesh = threepp::Mesh::create(geometry, material);

    sphere.setMesh(mesh);

    return mesh;
}

void SphereInitializer::circleInitializer(std::shared_ptr<threepp::Scene> scene, MySpheres &sphere) {
    auto mesh = createSphere(sphere);
    scene->add(mesh);
}