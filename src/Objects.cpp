#include "Objects.hpp"


MySpheres::MySpheres(float radius, int widthSegments, int heightSegments, threepp::Color color)
    : m_radius(radius),
      m_widthSegments(widthSegments),
      m_heightSegments(heightSegments),
      m_color(color),
    material(threepp::MeshBasicMaterial::create()) {
    material->color = color;
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

threepp::Color MySpheres::getColor()  {
    return m_color;
}


std::shared_ptr<threepp::Mesh> SphereInitializer::createSphere(MySpheres& sphere) {
    std::shared_ptr<threepp::SphereGeometry> geometry = threepp::SphereGeometry::create(
        sphere.getRadius(), sphere.getHeightSegments(), sphere.getWidthSegments()
        );

    std::shared_ptr<threepp::MeshBasicMaterial> material = sphere.material;
    material->color = sphere.getColor() ;

    std::shared_ptr<threepp::Mesh> mesh = threepp::Mesh::create(geometry, material);

    sphere.mesh = mesh; //gpt kode

    return mesh;
}

void SphereInitializer::circleInitializer(std::shared_ptr<threepp::Scene> scene, MySpheres& sphere ) {
    auto mesh = createSphere(sphere);
    scene->add(mesh);

}