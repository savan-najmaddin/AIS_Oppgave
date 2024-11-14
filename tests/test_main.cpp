#define CATCH_CONFIG_MAIN
#include "catch2/catch_test_macros.hpp"




int add(int x, int y);

TEST_CASE("Addition function test", "[add]") {
    REQUIRE(add(2, 3) == 5);
    REQUIRE(add(-1, 1) == 0);
    REQUIRE(add(0, 0) == 0);
    REQUIRE(add(-5, -3) == -8);
}