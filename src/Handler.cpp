#include "Handler.hpp"
#include <Objects.hpp>
#include <cstdlib>
#include <iostream>


void Handler::update(KinematicChain &chain, const MyUI &ui, VisualJoints &visualJoints, Scene &scene,
                 MySpheres &mySphere, Eigen::Vector2f targetPosition, float learningRate) {
    unlimitedGems(ui);
    jointResize(chain, ui);
    updateMesh(chain, visualJoints, scene, mySphere);
    chain.inverseKinematicsHandler(targetPosition, learningRate);
    visualJoints.updateJointVisual(chain);
}

int Handler::getPrevNumJoints() const {
    return m_prevNumJoints;
}

void Handler::setPrevNumJoints(int prevNumJoints) {
    m_prevNumJoints = prevNumJoints;
}

void Handler::unlimitedGems(const MyUI &ui) {
    if (ui.dontClick) {
        std::string kommando = " open " + std::string("https://www.youtube.com/watch?v=dQw4w9WgXcQ");
        system(kommando.c_str());
    }
}

void Handler::jointResize(KinematicChain &chain, const MyUI &ui) const {
    while (chain.getJoints().size() > ui.numJoints) {
        chain.removeJoint();
    }
    while (chain.getJoints().size() < ui.numJoints) {
        chain.addJoint(Joint(std::numbers::pi / 2, ui.jointLength));//don´t let new joint angle be same as starting angle
    }
}

void Handler::updateMesh(KinematicChain &chain, VisualJoints &visualJoints, Scene &scene,
                         MySpheres &mySphere) {

    if (chain.getJoints().size() != m_prevNumJoints) {
        chain.updateMaxReach();
        visualJoints.setChain(scene, chain);
        auto const geometry = SphereGeometry::create(chain.getMaxReach());
        mySphere.getMesh()->setGeometry(geometry);
        m_prevNumJoints = chain.getJoints().size();
    }
}
