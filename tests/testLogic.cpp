#define CATCH_CONFIG_MAIN
#include "Logic.hpp"
#include "JacobianSolver.hpp"
#include "catch2/catch_all.hpp"

//gpt was used here
TEST_CASE("Testing for accuracy if out of reach ", "[Effector to target test]") {
    // Create a KinematicChain with two joints of length 1 each
    KinematicChain chain;
    chain.addJoint(Joint(0.0f, 1.0f));
    chain.addJoint(Joint(0.0f, 1.0f));

    const Eigen::Vector2f targetPosition{3.0f, 0};
    chain.setTargetPosition(targetPosition);

    JacobianSolver solver;
    solver.solve(chain, 0.8f, 0.1f, 10);

    Eigen::Vector2f expectedPosition(2.0f, 0.0f);
    Eigen::Vector2f effectorPosition = chain.findEffectorPosition();

    REQUIRE_THAT(effectorPosition.y(), Catch::Matchers::WithinRel(expectedPosition.y(), 0.01f));
    REQUIRE_THAT(effectorPosition.x(), Catch::Matchers::WithinRel(expectedPosition.x(), 0.01f));
}

TEST_CASE("check for convegence", "[Test For Convergence]") {
    KinematicChain chain;
    chain.addJoint(Joint(0.0f, 1.0f));
    chain.addJoint(Joint(0.0f, 1.0f));

    const Eigen::Vector2f targetPosition(0.5f, 1.0f);
    chain.setTargetPosition(targetPosition);

    JacobianSolver solver2;
    solver2.solve(chain, 2.0f, 0.1f, 100000);

    Eigen::Vector2f endPos = chain.findEffectorPosition();
    Eigen::Vector2f error = targetPosition - endPos;
    REQUIRE(error.norm() < 0.1f);
}
//gpt was used here
TEST_CASE("Testing exception handling for joint removal", "[Kinematic joint exception test]") {
    KinematicChain chain;
    REQUIRE_THROWS_AS(chain.removeJoint(), std::out_of_range);
}
