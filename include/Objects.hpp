#ifndef OBJECTS_HPP
#define OBJECTS_HPP

#include "threepp/threepp.hpp"

struct MySpheres {
public:
    MySpheres(float radius, int widthSegments, int heightSegments, threepp::Color color);

    void setRadius(float radius);

    float getRadius() const;

    int getWidthSegments() const;

    int getHeightSegments() const;

    threepp::Color getColor() ;

    std::shared_ptr<threepp::MeshBasicMaterial> material;
    std::shared_ptr<threepp::Mesh> mesh;

private :
    float m_radius;
    const int m_widthSegments;
    const int m_heightSegments;
    threepp::Color m_color;

};


class SphereInitializer {
public:
    void circleInitializer(std::shared_ptr<threepp::Scene> scene, MySpheres& spheres);


private:
    std::shared_ptr<threepp::Mesh> createSphere(MySpheres& sphere);
};

#endif// OBJECTS_HPP
