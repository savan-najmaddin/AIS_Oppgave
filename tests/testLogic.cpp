#define CATCH_CONFIG_MAIN
#include "catch2/catch_all.hpp"
#include "Logic.hpp"

TEST_CASE("Testing the initial condition of the endeffector", "[effector test]") {

    KinematicChain chain(2);
    chain.addJoint(Joint(0.0f, 0.0f));
    chain.addJoint(Joint(0.0f, 0.0f));

    Eigen::Vector2f endEffectorPosition = chain.findEffectorPosition();

    Eigen::Vector2f expectedPosition = chain.findEffectorPosition();


    REQUIRE(endEffectorPosition.isApprox(expectedPosition, 0.01f));
}

//lag test for maks antall joint og lengde

