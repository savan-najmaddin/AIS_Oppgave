#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP



#include "threepp/threepp.hpp"
#include "Eigen/Core"
#include "Logikk.hpp" //må denne være her?


using namespace threepp;

inline kinematicChain chain;

class controller  {

public:
  struct MyMouseListener: MouseListener {
    float& t;


    explicit MyMouseListener(float& t): t(t) {}

    void onMouseDown(int button, const Vector2& pos) override {
      Eigen::Vector2f target(pos.x, pos.y);
      chain.targetPosition(target); //oversetter fra threepp til eigen og sender til logikk.hpp


    }
  };


private:
  Mesh* myAddedMesh = nullptr;
  bool addedMesh = false;
  void handleMouseClick();

  std::shared_ptr<Mesh> createMesh();

};



#endif //CONTROLLER_HPP
