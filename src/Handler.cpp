#include "Handler.hpp"

#include <Objects.hpp>

Handler::Handler( KinematicChain& chain, const MyUI& ui, VisualJoints& visualJoints, Scene& scene,
                 MySpheres& mySphere)
    : m_geometry(std::shared_ptr<SphereGeometry>()),
      m_prevNumJoints(0) {
    jointResize(chain, ui);
    updateMesh(chain, visualJoints, scene, mySphere, m_prevNumJoints);
}


void Handler::jointResize(KinematicChain& chain, const  MyUI& ui) {
    while (chain.joints.size() > ui.numJoints) {
        chain.removeJoint();
    }
    while (chain.joints.size() < ui.numJoints) {
        chain.addJoint(Joint(std::numbers::pi, ui.jointLength));
    }
}

void Handler::updateMesh(KinematicChain& chain, VisualJoints& visualJoints,  Scene& scene,
                         MySpheres& mySphere, int& prevNumJoints) {
    if (chain.joints.size() != prevNumJoints) {

        chain.updateMaxReach();
        visualJoints.setChain(scene, chain);
        auto const geometry = SphereGeometry::create(chain.getMaxReach());
        mySphere.getMesh()->setGeometry(geometry);
        prevNumJoints = chain.joints.size();
    }
}
