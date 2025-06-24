#include "Logic.hpp"
#include <chrono>
#include <numbers>
#include <stdexcept>


Joint::Joint(const float ang, const float len) : angle(ang), length(len) {}


//bevegelsen til IK kjeden gjort numerisk
/**
 * @brief Inverse Kinematic motion done numerically using the Jacobian
 */


void KinematicChain::setTargetPosition(const Eigen::Vector2f &position) {
    m_targetPosition = position;
}
const Eigen::Vector2f  &KinematicChain::getTargetPosition() const{
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

