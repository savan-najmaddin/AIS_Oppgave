#ifndef VISUAL_HPP
#define VISUAL_HPP

#include "threepp/threepp.hpp"
#include "Logikk.hpp"
#include "iostream"

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
  auto createSphere(float radius, int widthSegments, int heightSegments, auto color ) {
    auto geometry = threepp::SphereGeometry::create( radius, widthSegments, heightSegments );
    auto material = threepp::MeshBasicMaterial::create();
    material->color = color;
    return threepp::Mesh::create(geometry, material);
  }
private:


};
#endif
