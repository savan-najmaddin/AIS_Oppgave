#include "Visual.hpp"
#include "catch2/catch_all.hpp"

TEST_CASE("Testing exception handling for joint removal", "[Visual joint exception test]") {
    KinematicChain chain;
    REQUIRE_THROWS_AS(chain.removeJoint(), std::out_of_range);
}


TEST_CASE("Removing Joints", "[Visual joints removal]") {
    KinematicChain chain;
    VisualJoints visualJoints;
    const auto scene = threepp::Scene::create();

    chain.addJoint(Joint(0, 1.0f));
    chain.addJoint(Joint(0, 1.0f));
    visualJoints.setChain(*scene, chain);

    REQUIRE(visualJoints.joints.size() == 2);
}