#define CATCH_CONFIG_MAIN
#include "Logic.hpp"
#include "catch2/catch_all.hpp"


TEST_CASE("Testing for accuracy if out of reach ", "[Test#2]") {
    // Create a KinematicChain with two joints of length 1 each
    KinematicChain chain;
    chain.addJoint(Joint(0.0f, 1.0f));
    chain.addJoint(Joint(0.0f, 1.0f));

    const Eigen::Vector2f targetPosition{3.0f, 0};
    chain.setTargetPosition(targetPosition);

    chain.inverseKinematicsHandler(0.8f, 0.1f, 10);

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

    chain.inverseKinematicsHandler(2.0f, 0.1f, 100000);

    REQUIRE(chain.getHasConverged(0.1f) == true);
}

TEST_CASE("Testing exception handling for joint removal", "[Joint Removal]") {
    KinematicChain chain;
    REQUIRE_THROWS_AS(chain.removeJoint(), std::out_of_range);
}
