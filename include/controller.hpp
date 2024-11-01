#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "threepp/threepp.hpp"
using namespace threepp;

class controller : KeyListener, MouseListener {

public:
  controller(Scene& scene);

  void onMouseDown(int key, const Vector2 & pos) override;
  void onKeyPressed(KeyEvent) override;
  void onKeyReleased(KeyEvent) override;

private:
  Scene& scene;
  Mesh* myAddedMesh = nullptr;
  bool addedMesh = false;

  void handleAKey();
  void handleSKey();
  void handleEnterKey();
  void wrongKeyEnter();
  std::shared_ptr<Mesh> createMesh();

};
#endif //CONTROLLER_HPP
