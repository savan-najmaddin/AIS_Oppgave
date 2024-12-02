#ifndef OBJECTS_HPP
#define OBJECTS_HPP

#include "threepp/threepp.hpp"


class MySpheres {
public:
    MySpheres(float radius, threepp::Color color,
        const std::shared_ptr<threepp::Scene>& scene, bool transparent = false, float opacity = 1.0f);

    void setRadius(float radius);
    float getRadius() const;

    std::shared_ptr<threepp::Mesh> getMesh() const;

    void addToScene(const std::shared_ptr<threepp::Scene> &scene) const;

private:
    float m_radius;
    threepp::Color m_color;
    bool m_transparent;
    float m_opacity;

    void createMesh();

    std::shared_ptr<threepp::Mesh> m_mesh;
    std::shared_ptr<threepp::SphereGeometry> m_geometry;
    std::shared_ptr<threepp::MeshBasicMaterial> m_material;
};

#endif// OBJECTS_HPP
