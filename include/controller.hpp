#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP


#include "objekt.hpp"
#include "threepp/threepp.hpp"
#include "Eigen/Core"
#include "Logikk.hpp" //må denne være her?


using namespace threepp;

class controller  {

public:
  struct MyMouseListener: MouseListener {
    float& t;
    kinematicChain chain;

    explicit MyMouseListener(float& t): t(t) {}

    void onMouseDown(int button, const Vector2& pos) override {
      Eigen::Vector2f target(pos.x, pos.y);
      chain.insideCircle(target);

    }
  };


  controller(Scene& scene, std::vector<armSegment>& ) ;

  void onMouseDown(int key, const Vector2 &pos) ;
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
