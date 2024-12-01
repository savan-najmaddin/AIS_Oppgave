#include "Logic.hpp"

#include <numbers>
#include <chrono>
#include <stdexcept>


//todo integrated testing
//todo test idea, does visual joint update correctly
//todo assertions

Joint::Joint(float ang, float len) : angle(ang), length(len) {}



KinematicChain::KinematicChain(size_t n) : m_joints(n) {
    float totalLength = 0.0f;
    for (const auto &joint: m_joints) {
        totalLength += joint.length;
    }

    m_maxReach = totalLength;
}

std::vector<Joint> KinematicChain::getJoints() const {
    return m_joints;
}

void KinematicChain::addJoint(const Joint &joint) {
    m_joints.emplace_back(joint);
}
void KinematicChain::removeJoint() {
    if(m_joints.empty()) {
        throw std::out_of_range("cannot remove joints if there aren´t any");
    }
    m_joints.pop_back();
}

void KinematicChain::targetPosition(Eigen::Vector2f &position) {
    m_newVectorPosition = position;
}

 Eigen::Vector2f &KinematicChain::getTargetPosition()  {
    return m_newVectorPosition;
}

void KinematicChain::circularMotion(Eigen::Vector2f &position, float radius) {

    static auto start = std::chrono::steady_clock::now(); //gpt
    float time = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count(); //gpt

    float angle = 1.0f;
    position.x() = std::sin(time * angle)  *  radius *1.25 ;
    position.y() = std::cos(time * angle)  *  radius * 1.25 ;

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
    if(m_joints.size() < 0) {
        throw std::invalid_argument ( " there is no effector " );
    }
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativAngle = 0.0f;

    for (const auto &joint: m_joints) {
        cumulativAngle += joint.angle;
        position.x() += joint.length * std::cos(cumulativAngle);
        position.y() += joint.length * std::sin(cumulativAngle);
    }
    return position;
}

std::vector<float> KinematicChain::computeCumulativeAngles() const {

    std::vector<float> cumulativeAngles(m_joints.size());
    float cumulativeAngle = 0.0f;

    for (size_t i = 0; i < m_joints.size(); ++i) {
        cumulativeAngle += m_joints[i].angle;
        cumulativeAngles[i] = cumulativeAngle;
    }
    return cumulativeAngles;
}

Eigen::MatrixXf KinematicChain::computeJacobianTranspose() const {
    if (m_joints.size() < 0) {
        throw std::invalid_argument(" there are no negative joints ");
    }
    Eigen::MatrixXf jacobianTranspose(m_joints.size(), 2);
    jacobianTranspose.setZero();
    std::vector<float> cumulativeAngles = computeCumulativeAngles();

    for(size_t i = 0; i < m_joints.size() ; ++i ) {
        auto [partialX, partialY] = computePartialDerivates(i, cumulativeAngles);
        jacobianTranspose(i, 0) = partialX;
        jacobianTranspose(i, 1) = partialY;
    }
    return jacobianTranspose;
}

void KinematicChain::updateJointAngles(const Eigen::VectorXf &angleAdjustments) {
    for (size_t i = 0; i < m_joints.size(); ++i) {
        m_joints[i].angle += angleAdjustments(i);
        m_joints[i].angle = clampAngle(m_joints[i].angle);
    }
}

void KinematicChain::inverseKinematicsHandler(const Eigen::Vector2f &targetPosition, float learningRate, float threshold, int maxIteration) {
    for (size_t i = 0; i < maxIteration; ++i) {
        if(m_joints.size() < 0) {
            throw std::invalid_argument("negative joints");
        }
        for (size_t j = 0; j < maxIteration; ++j) {
            if(hasConverged(targetPosition, threshold)) {
                break;
            }
            errorHandler(targetPosition, learningRate);
        }
    }
}

Eigen::Vector2f KinematicChain::computeError(const Eigen::Vector2f &targetPosition) const {
    Eigen::Vector2f currentPosition = findEffectorPosition();
    Eigen::Vector2f error = targetPosition - currentPosition;

    return error;
}

Eigen::VectorXf KinematicChain::computeAngleAdjustments(const Eigen::Vector2f &error, float learningRate) const {
    Eigen::MatrixXf jacobianTranspose = computeJacobianTranspose();
    Eigen::VectorXf angleAdjustments = learningRate * jacobianTranspose * error;

    float maxAngleChange = 0.002f;//limit angle change, this is to slow it down once it´s close to the target
    for (int i = 0; i < angleAdjustments.size(); ++i) {
        angleAdjustments(i) = std::clamp(angleAdjustments(i), -maxAngleChange, maxAngleChange);
    }
    return angleAdjustments;
}


std::pair<float, float> KinematicChain::computePartialDerivates(
    size_t i, const std::vector<float>& cumulativeAngles) const {
        float partialX = 0.0f;
        float partialY = 0.0f;

        for (size_t j = i; j < m_joints.size(); ++j) {
            float angleSum = cumulativeAngles[j];
            float dx_dtheta = -m_joints[j].length * std::sin(angleSum);
            float dy_dtheta = m_joints[j].length * std::cos(angleSum);

            partialX += dx_dtheta;
            partialY += dy_dtheta;
        }
        return {partialX, partialY};
}


float KinematicChain::clampAngle(float angle) {
    angle = std::fmod(angle, 2 * std::numbers::pi); //mod av 2π

    if (angle < 0.0f) {
        angle += 2 * std::numbers::pi;
    }

    return angle;
}

bool KinematicChain::hasConverged(const Eigen::Vector2f &targetPosition, float threshold) const {
    Eigen::Vector2f error = computeError(targetPosition);
    return error.norm() < threshold;
}


void KinematicChain::adjustErrorMagnitude( Eigen::Vector2f &error) const {
    float errorMagnitude = error.norm();
    float maxErrorMagnitutde = 0.001f;//define how precise the end effector should be

    if (errorMagnitude > maxErrorMagnitutde) {
        error *= maxErrorMagnitutde / errorMagnitude;
    }
}

void KinematicChain::errorHandler(const Eigen::Vector2f &targetPosition, float learningRate) {
    Eigen::Vector2f error = computeError(targetPosition);
    adjustErrorMagnitude(error);

    Eigen::VectorXf angleAdjustments = computeAngleAdjustments(error, learningRate);
    updateJointAngles(angleAdjustments);
}