#include "Handler.hpp"

/*

void Handler::refreshChain() {
    if(ui.initializeChain)
    {
        while (chain.joints.size() > ui.numJoints) {
            chain.removeJoint();
        }
        while (chain.joints.size() < ui.numJoints) {
            chain.addJoint(Joint(std::numbers::pi, ui.jointLength));
        }

        if (chain.joints.size() != prevNumJoints) {

            chain.updateMaxReach();
            visualJoints.setChain(*scene, chain);
            auto const geometry = SphereGeometry::create(chain.getMaxReach());
            reachCircle.getMesh()->setGeometry(geometry);
            prevNumJoints = chain.joints.size();
        }

        chain.updateInverseKinematics(targetPosition, learningRate);
        visualJoints.update(chain);
    }
}*/