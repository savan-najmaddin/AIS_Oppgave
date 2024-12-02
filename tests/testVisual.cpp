#include "Visual.hpp"
#include "catch2/catch_all.hpp"


TEST_CASE("Testing exception handling for joint removal", "[Joint Removal]") {
    KinematicChain chain;
    REQUIRE_THROWS_AS(chain.removeJoint(), std::out_of_range);
}