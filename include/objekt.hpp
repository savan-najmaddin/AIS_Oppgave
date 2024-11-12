#ifndef OBJEKT_HPP
#define OBJEKT_HPP

#include "threepp/threepp.hpp"
#include "Eigen/Core"

using namespace threepp;

std::shared_ptr<Mesh> inline createBox() {
  const auto boxGeometry = BoxGeometry::create();
  const auto boxMaterial = MeshBasicMaterial::create();
  boxMaterial->color.setRGB(1, 0, 0);
  boxMaterial->transparent = true;
  boxMaterial->opacity = 0.1f;
  auto box = Mesh::create(boxGeometry, boxMaterial);

  auto wiredBox = LineSegments::create(WireframeGeometry::create(*boxGeometry));
  wiredBox->material()->as<LineBasicMaterial>()->depthTest = false;
  wiredBox->material()->as<LineBasicMaterial>()->color = Color::gray;
  box->add(wiredBox);

  return box;
};




class circleObject {
public:
  circleObject() {

    auto circleGeometry = CircleGeometry::create(2.0f, 32);
    auto circleMaterial = MeshBasicMaterial::create();
    circleMaterial->color = Color::red;

    circleMesh = Mesh::create(circleGeometry, circleMaterial);

  }
/*
  void setPosition(const Eigen::Vector2f& position) {
    circleMesh->position.set(position.x(), position.y(), 0);
    }
    */


  std::shared_ptr<Mesh> getMesh() const {
    return circleMesh;
  }

private:
  std::shared_ptr<Mesh> circleMesh;

};



#endif //OBJEKT_HPP
