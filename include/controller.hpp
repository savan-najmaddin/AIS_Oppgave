#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP


#include "threepp/threepp.hpp"
#include "objekt.hpp"



using namespace threepp;

class controller : public KeyListener, public MouseListener {

public:
  controller(Scene& scene, std::vector<armSegment>& ) ;

  void onMouseDown(int key, const Vector2 &pos) override;
  Vector2 lastMousePosition(float x, float y);



private:
  std::vector<armSegment>&armVec;
  Scene& scene;
  Mesh* myAddedMesh = nullptr;
  bool addedMesh = false;

  void handleMouseClick();

  std::shared_ptr<Mesh> createMesh();

};



#endif //CONTROLLER_HPP
