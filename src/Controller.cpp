#include "controller.hpp"


controller::MyMouseListener::MyMouseListener(float &t, KinematicChain &chain, threepp::Canvas &canvas, threepp::OrthographicCamera &camera)
    : t(t),
      chain(chain),
      canvas(canvas),
      camera(camera) {}

void controller::MyMouseListener::onMouseDown(int button, const threepp::Vector2 &pos) {
    // adjusting the mouse position relative to the canvas size, PS adjusted for square canvas
    threepp::WindowSize const windowSize = canvas.size();


    float const NDC_X = (2.0f * pos.x) / windowSize.height() - 1.0f;
    float const NDC_Y = 1.0f - (2.0f * pos.y) / windowSize.height();

    // for targetposition
    threepp::Vector3 ndcCoords(NDC_X, NDC_Y, 0.0f); // Z = 0 for the XY-plane
    const threepp::Vector3 worldCoords = ndcCoords.unproject(camera);


    Eigen::Vector2f target(worldCoords.x, worldCoords.y);
    chain.targetPosition(target);
}