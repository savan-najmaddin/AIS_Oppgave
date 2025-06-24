#include "Logic.hpp"
#include <chrono>
#include <numbers>
#include <stdexcept>


Joint::Joint(const float ang, const float len) : angle(ang), length(len) {}


//bevegelsen til IK kjeden gjort numerisk
/**
 * @brief Inverse Kinematic motion done numerically using the Jacobian
 */

/*
 * sources: https://www.youtube.com/watch?v=2_cdDGwnl80&t=211s&ab_channel=Geat
 * https://www.youtube.com/watch?v=2iPahF1c4bQ&t=147s&ab_channel=KeitaroNaruse
 * and chatGPT
 *
 * i also used the base code from:  https://github.com/raphaellevisse/Minimal_Inverse_Kinematics
 *characteristic of the jacobian is that changes in the joint angles map to changes in position of the end effector.
 * The Jacobian matrix represent the relationship between the joint velcoities and end effector velocities.
 * It´s more efficient to use the transpose of the Jacobian, since it´s more efficient than the inverse.
 *
 * We take the transpose of the Jacobian, that way the rows represent joints, and columns the end effector position.
 * then we take the transpose and multiply it by the error vector to get the angle adjustments.
 * in our case the error vector is the difference between the target position and the current effector position
 * when computing the Jacobian Transpose, we do so iteratively, since each item in the array is independent.
 * we get the deriviatives of the rotation by adding a small value and seeing how it correlates with the change in position.
 * its imperative to record the changes made under the calculations.
 *
 * to solve for change in each angle, we multiply by the dot product of transpose matrix and error vector
 *
 * Good to know: the jacobian isn´t necessarily 'correct' but it is a good approximation, hence the need for small step.
 * in my program, there are 'irrelevant' methods that may not be relevant to the Solver itself,
 * but rather for the integration of the solver into the program.
 *
 * throughout the program, i´ve used GPT to help with structure and logic, the code has been consistently updated and improved.
 * gpt was especially critical in the start, but it also brought up serious issues that weren´t discovered until later.
 *  i´ve also gotten help from fellow students, especially seniors who´ve helped me with the logic and structure of the in the later stages.
 */



void KinematicChain::setTargetPosition(const Eigen::Vector2f &position) {
    m_targetPosition = position;
}
Eigen::Vector2f &KinematicChain::getTargetPosition() {
    return m_targetPosition;
}

KinematicChain::KinematicChain(const size_t n) : m_joints(n) {
    float totalLength = 0.0f;
    for (const auto &joint: m_joints) {
        totalLength += joint.length;
    }

    m_maxReach = totalLength;
}

const std::vector<Joint> &KinematicChain::getJoints() const {
    return m_joints;
}

std::vector<Joint> &KinematicChain::getJoints() {
    return m_joints;
}

void KinematicChain::addJoint(const Joint &joint) {
    m_joints.emplace_back(joint);
}
void KinematicChain::removeJoint() {
    if (m_joints.empty()) {
        throw std::out_of_range("cannot remove joints if there aren´t any");
    }
    if (!m_joints.empty())
        m_joints.pop_back();
}


float KinematicChain::getMaxReach() const {
    return m_maxReach;
}
void KinematicChain::updateMaxReach() {
    float totalLength{0};
    for (const auto &joint: m_joints) {
        totalLength += joint.length;
    }
    m_maxReach = totalLength;
}

Eigen::Vector2f KinematicChain::findEffectorPosition() const {
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativAngle = 0.0f;

    for (const auto &joint: m_joints) {
        cumulativAngle += joint.angle;
        position.x() += joint.length * std::cos(cumulativAngle);
        position.y() += joint.length * std::sin(cumulativAngle);
    }
    return position;
}


/* @brief limit angle change to 0-2π tp prevent unwated behavior */
float KinematicChain::clampAngle(float angle) {
    angle = std::fmod(angle, 2 * std::numbers::pi);//gpt recommended fmod

    if (angle < 0.0f) {
        angle += 2 * std::numbers::pi;
    }
    return angle;
}

