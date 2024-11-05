#include <catch2/catch_test_macros.hpp>


// Declaration of the add function

TEST_CASE("test add") {
    int a = 1;
    int b = 3;

    int add = add(a, b);

    CHECK(add == a+b);
}