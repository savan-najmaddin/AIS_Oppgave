#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP



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
      Eigen::Vector2f const target(pos.x, pos.y);

      std::cout << "position.x()" << target.x() << "position.y()" << target.y() << std::endl;

    }
  };



  void onMouseDown(int key, const Vector2 &pos) ;
  Vector2 lastMousePosition(float x, float y);



private:
  Mesh* myAddedMesh = nullptr;
  bool addedMesh = false;
  void handleMouseClick();

  std::shared_ptr<Mesh> createMesh();

};



#endif //CONTROLLER_HPP
