/**
 * @brief Mouse listner returns a value in threepp::Vector2, which is then converted to NDC coordinates, due to
 * the cordinates originally being world cordinates. later the ndc cordinates are put in Eigen::Vector2f form
 * and this is used to set the target position of the chain
 *
 * Sources: GPT was used to let me know that World Cordinates and NDC cordinates are a thing. and to also get
 * the aspects right
 */
#include "Controller.hpp"


controller::MyMouseListener::MyMouseListener(float &t, KinematicChain &chain, threepp::Canvas &canvas, threepp::OrthographicCamera &camera)
    : chain(chain),
      t(t),
      canvas(canvas),
      camera(camera) {}

void controller::MyMouseListener::onMouseDown(int button, const threepp::Vector2 &pos) {
    // adjusting the mouse position relative to the canvas size, PS adjusted for square canvas
    threepp::WindowSize const windowSize = canvas.size();


    float const NDC_X = 2.0f * pos.x / windowSize.height() - 1.0f;
    float const NDC_Y = 1.0f - 2.0f * pos.y / windowSize.height();

    // for targetposition
    threepp::Vector3 ndcCoords(NDC_X, NDC_Y, 0.0f);//Z = 0 for the XY-plane
    const threepp::Vector3 worldCoords = ndcCoords.unproject(camera);


    const Eigen::Vector2f target(worldCoords.x, worldCoords.y);
    chain.setTargetPosition(target);
}