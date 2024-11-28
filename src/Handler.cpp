#include "Handler.hpp"
#include <iostream>
#include <Objects.hpp>

int prevNumJoints = 0;

Handler::Handler( KinematicChain& chain, const MyUI& ui, VisualJoints& visualJoints, Scene& scene,
                 MySpheres& mySphere)
    {
    jointResize(chain, ui);
    updateMesh(chain, visualJoints, scene, mySphere );
}

void Handler::jointResize(KinematicChain& chain, const  MyUI& ui) {
    while (chain.joints.size() > ui.numJoints ) {
        chain.removeJoint();
    }
    while (chain.joints.size() < ui.numJoints) {
        chain.addJoint(Joint(std::numbers::pi /2 , ui.jointLength)); //don´t let angle be same as starting angle
    }
}

void Handler::updateMesh(KinematicChain& chain, VisualJoints& visualJoints,  Scene& scene,
                         MySpheres& mySphere) {
    if (chain.joints.size() != prevNumJoints) {
        chain.updateMaxReach();
        visualJoints.setChain(scene, chain);
        auto const geometry = SphereGeometry::create(chain.getMaxReach());
        mySphere.getMesh()->setGeometry(geometry);
        prevNumJoints = chain.joints.size();
    }

}
