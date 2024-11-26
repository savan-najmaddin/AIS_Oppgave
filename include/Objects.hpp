#ifndef OBJECTS_HPP
#define OBJECTS_HPP

#include <memory>
#include "threepp/threepp.hpp"


struct Spheres {
  std::shared_ptr<threepp::Mesh> centerCircle;
  std::shared_ptr<threepp::Mesh> targetCircle;
  std::shared_ptr<threepp::Mesh> reachCircle;
};

class MySpheres {
public:
   Spheres circleInitializer(std::shared_ptr<threepp::Scene> scene);

private:

  auto createSphere(float radius, int widthSegments, int heightSegments, auto color);
  void sphereToScene(std::shared_ptr<threepp::Scene> scene, auto sphere);
};

#endif // OBJECTS_HPP
