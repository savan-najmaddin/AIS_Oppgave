
#include "Handler.hpp"

#include <MyScene.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("testing if visual joints are updated correctly", "[Test#1]") {
    Canvas canvas;
    MyUI ui(canvas);
    ui.numJoints = 0;  // numJoints is zero

    KinematicChain chain;
    VisualJoints visualC;
    auto scene = createScene();
    MySpheres mySphere(0.0f, Color(0x0000FF), scene);
    Eigen::Vector2f targetPosition(0.0f, 0.0f);
    float learningRate = 0.1f;

    // Create a Handler instance
    Handler handler(chain, ui, visualC, *scene, mySphere, targetPosition, learningRate);

    // Verify that the chain has zero joints
    REQUIRE(chain.getJoints().size() == visualC.visualJoints.size());


}
