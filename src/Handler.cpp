#include "Handler.hpp"
#include <iostream>
#include <Objects.hpp>
#include <cstdlib>


int prevNumJoints{0};


Handler::Handler( KinematicChain& chain, const MyUI& ui, VisualJoints& visualJoints, Scene& scene,
                 MySpheres& mySphere, Eigen::Vector2f targetPosition, float learningRate)
    {
    unlimitedGems(ui );
    jointResize(chain, ui);
    updateMesh(chain, visualJoints, scene, mySphere );
    chain.updateInverseKinematics(targetPosition, learningRate);
    visualJoints.updateJointVisual(chain);

}

int Handler::getPrevNumJoints() {
    return m_prevNumJoints;
}

void Handler::setPrevNumJoints(int prevNumJoints) {
    m_prevNumJoints = prevNumJoints;
}

void Handler::unlimitedGems(const MyUI& ui) {
    if (ui.dontClick) {
        std::string kommando = " open " + std::string("https://www.youtube.com/watch?v=dQw4w9WgXcQ");
        system(kommando.c_str());
    }
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

    if (chain.joints.size() > m_prevNumJoints || chain.joints.empty() ) { //todo bli kvitt chain.joints.empty()
        chain.updateMaxReach();
        visualJoints.setChain(scene, chain);
        auto const geometry = SphereGeometry::create(chain.getMaxReach());
        mySphere.getMesh()->setGeometry(geometry);
        m_prevNumJoints = chain.joints.size() +1 ;
    }

}
