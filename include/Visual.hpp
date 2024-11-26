#ifndef VISUAL_HPP
#define VISUAL_HPP

#include "Logic.hpp"
#include "iostream"
#include "threepp/threepp.hpp"

class VisualJoints {
public:
    void setChain(threepp::Scene &scene, const KinematicChain &chain);

    // Kan være const fordi pekeren ikke blir endret på direkte
    void update(const KinematicChain &chain) const;

private:
    static constexpr float WIDTH = 0.2f;

    std::vector<std::unique_ptr<threepp::Mesh>> joints;
};

class MySpheres {
public:
    static auto createSphere(float const radius, int const widthSegments, int const heightSegments, auto color) {
        const auto geometry = threepp::SphereGeometry::create(radius, widthSegments, heightSegments);
        const auto material = threepp::MeshBasicMaterial::create();
        material->color = color;
        return threepp::Mesh::create(geometry, material);
    }
};
#endif
