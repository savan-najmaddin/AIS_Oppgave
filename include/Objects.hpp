#ifndef OBJECTS_HPP
#define OBJECTS_HPP

#include "threepp/threepp.hpp"

struct MySpheres {
public:
    MySpheres(float radius, int widthSegments, int heightSegments,
        threepp::Color color, bool transparent = false, float opacity = 1.0f);

    void setRadius(float radius);

    float getRadius() const;

    int getWidthSegments() const;

    int getHeightSegments() const;

    threepp::Color getColor() ;

    void setMaterial(std::shared_ptr<threepp::MeshBasicMaterial> material);
    std::shared_ptr<threepp::MeshBasicMaterial> getMaterial() const;

    void setMesh(std::shared_ptr<threepp::Mesh> mesh);
    std::shared_ptr<threepp::Mesh> getMesh() const;

private :
    float m_radius;
    const int m_widthSegments;
    const int m_heightSegments;
    threepp::Color m_color;

    std::shared_ptr<threepp::MeshBasicMaterial> m_material;
    std::shared_ptr<threepp::Mesh> m_mesh;
};


class SphereInitializer {
public:
    void circleInitializer(std::shared_ptr<threepp::Scene> scene, MySpheres& spheres);


private:
    std::shared_ptr<threepp::Mesh> createSphere(MySpheres& sphere);
};

#endif// OBJECTS_HPP
