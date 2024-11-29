#define CATCH_CONFIG_MAIN
#include "Logic.hpp"
#include "catch2/catch_all.hpp"



TEST_CASE("Testing findEffectorPosition", "[findEffectorPosition]") {
    KinematicChain chain(0);
    chain.addJoint(Joint(0.0f, 1.0f));
    chain.addJoint(Joint(0.0f, 1.0f));


    Eigen::Vector2f expectedPosition(2.0f, 0.0f);

    // Get the effector position from the chain
    Eigen::Vector2f effectorPosition = chain.findEffectorPosition();

    REQUIRE((effectorPosition - expectedPosition).norm() < 0.001f);
}

TEST_CASE("Testing for accuracy if out of reach ", "[Test#2]") {
    // Create a KinematicChain with two joints of length 1 each
    KinematicChain chain(0);
    chain.addJoint(Joint(0.0f, 1.0f));
    chain.addJoint(Joint(0.0f, 1.0f));

     Eigen::Vector2f targetPosition{3.0f, 0};

    chain.updateInverseKinematics(targetPosition, 0.8f, 0.1f, 10);

    Eigen::Vector2f expectedPosition(2.0f, 0.0f);
    Eigen::Vector2f effectorPosition = chain.findEffectorPosition();

    REQUIRE_THAT(effectorPosition.y(), Catch::Matchers::WithinRel(expectedPosition.y(), 0.01f));
    REQUIRE_THAT(effectorPosition.x(), Catch::Matchers::WithinRel(expectedPosition.x(), 0.01f));
}

TEST_CASE(" Testing for joint limit before crash", "[Test#3]") { //ble ikke kræsj hos meg

        KinematicChain chain;
        size_t count = 0;
        try {
            while (true) {
                chain.addJoint(Joint(0.0f, 1.0f));
                ++count;
                if (count % 10000 == 0) {
                    std::cout << "Added " << count << " joints." << std::endl;
                }
            }
        } catch (const std::bad_alloc& e) {
            std::cerr << "Memory allocation failed after adding " << count << " joints: " << e.what() << std::endl;
        }
    }




