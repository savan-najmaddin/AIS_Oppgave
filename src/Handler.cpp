#include "Handler.hpp"
#include <Objects.hpp>
#include <cstdlib>
#include <iostream>


using namespace threepp;

// This function is responsible for updating the kinematic chain based on user input.
void Handler::update(KinematicChain &chain, const MyUI &ui, VisualJoints &visualJoints, Scene &scene,
                     const MySpheres &mySphere, const float learningRate) {
    unlimitedGems(ui);
    jointResize(chain, ui);
    updateMesh(chain, visualJoints, scene, mySphere);
    chain.inverseKinematicsHandler(learningRate);
    visualJoints.updateJointVisual(chain);
}

int Handler::getPrevNumJoints() const {
    return m_prevNumJoints;
}

void Handler::setPrevNumJoints(const int prevNumJoints) {
    m_prevNumJoints = prevNumJoints;
}

void Handler::unlimitedGems(const MyUI &ui) {
    if (ui.dontClick) {
        const std::string kommandoWin = "start " + std::string("https://www.youtube.com/watch?v=dQw4w9WgXcQ");
        const std::string kommandoMac = "open " + std::string("https://www.youtube.com/watch?v=dQw4w9WgXcQ");
        const std::string kommandoLinux = "xdg-open " + std::string("https://www.youtube.com/watch?v=dQw4w9WgXcQ");
        system(kommandoWin.c_str());
        system(kommandoMac.c_str());
        system(kommandoLinux.c_str());
        ui.dontClick = false;
    }
}

void Handler::jointResize(KinematicChain &chain, const MyUI &ui) {
    while (chain.getJoints().size() > ui.numJoints) {
        chain.removeJoint();
    }
    while (chain.getJoints().size() < ui.numJoints) {
        chain.addJoint(Joint(std::numbers::pi / 2, ui.jointLength));//avoid joints alligning, set different from start angle
    }
}

void Handler::updateMesh(KinematicChain &chain, VisualJoints &visualJoints, Scene &scene,
                         const MySpheres &mySphere) {
    if (chain.getJoints().size() != m_prevNumJoints) {
        chain.updateMaxReach();
        visualJoints.setChain(scene, chain);
        auto const geometry = SphereGeometry::create(chain.getMaxReach());
        mySphere.getMesh()->setGeometry(geometry);//gpt was used here
        m_prevNumJoints = chain.getJoints().size();
    }
}
