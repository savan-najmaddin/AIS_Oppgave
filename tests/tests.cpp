#define CATCH_CONFIG_MAIN
#include "catch2/catch_test_macros.hpp"


/*
#include "Eigen/Core"

TEST_CASE("kinematicChain inverse kinematics moves end effector towards target", "[kinematicChain]") {
    kinematicChain arm(3);
    arm.joints[0] = Joint(0.0f, 1.0f);
    arm.joints[1] = Joint(0.0f, 1.0f);
    arm.joints[2] = Joint(0.0f, 1.0f);

    Eigen::Vector2f targetPosition(2.0f, 1.0f);
    arm.inverseKinematics(targetPosition, 0.1f);

    Eigen::Vector2f newPosition = arm.findEffectorPosition();
    float distance = (newPosition - targetPosition).norm();

    REQUIRE(distance < 0.01f); // Expect the end effector to be within 0.01 units of the target
}
*/