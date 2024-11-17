#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP



#include "threepp/threepp.hpp"
#include "Eigen/Core"
#include "Logikk.hpp" //må denne være her?

using namespace threepp;

class controller  {

  public:
  struct MyMouseListener: MouseListener {

    KinematicChain& chain;
    float& t;
    Canvas& canvas;
    OrthographicCamera& camera;

    float windowWidth = 800.0f;
    float windowHeight = 800.0f;

    MyMouseListener(float& t, KinematicChain& chain, Canvas& canvas, OrthographicCamera& camera)
        : t(t), chain(chain), canvas(canvas), camera(camera) {}

    void onMouseDown(int button, const Vector2& pos) override {
      // Convert screen coordinates to NDC
      float ndcX = (2.0f * pos.x) / 800.0f - 1.0f;
      float ndcY = 1.0f - (2.0f * pos.y) / 800.0f;

      // Unproject to world coordinates
      Vector3 ndcCoords(ndcX, ndcY, 0.0f); // Z = 0 for the XY-plane
      Vector3 worldCoords = ndcCoords.unproject(camera);

      // Update the target position in the kinematic chain
      Eigen::Vector2f target(worldCoords.x, worldCoords.y);
      chain.targetPosition(target);
    }
  };
  // ...



private:
  Mesh* myAddedMesh = nullptr;
  bool addedMesh = false;
  void handleMouseClick();

  std::shared_ptr<Mesh> createMesh();

};



#endif //CONTROLLER_HPP
