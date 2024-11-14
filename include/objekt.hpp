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






#endif //OBJEKT_HPP
