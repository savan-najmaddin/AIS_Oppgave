#ifndef OBJEKT_HPP
#define OBJEKT_HPP

#include "threepp/threepp.hpp"
#include "Eigen/Core"

using namespace threepp;

class circleObject {
public:
  circleObject() {

    auto circleGeometry = CircleGeometry::create(0.5f, 32);
    auto circleMaterial = MeshBasicMaterial::create();
    circleMaterial->color = Color::red;

    circleMesh = Mesh::create(circleGeometry, circleMaterial);
  }

  void setPosition(const Eigen::Vector2f& position) {
    circleMesh->position.set(position.x(), position.y(), 0);
  }

  std::shared_ptr<Mesh> getMesh() const {
    return circleMesh;
  }


private:
  std::shared_ptr<Mesh> circleMesh;

};



#endif //OBJEKT_HPP
