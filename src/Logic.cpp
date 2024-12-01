#include "Logic.hpp"

#include <numbers>
#include <chrono>
#include <stdexcept>
#include <cassert>


//todo integrated testing
//todo test idea, does visual joint update correctly
//todo assertions

Joint::Joint(float ang, float len) : angle(ang), length(len) {}

//important, for loops are used often since arrays are independent of each other

KinematicChain::KinematicChain(size_t n) : m_joints(n) {
    float totalLength = 0.0f;
    for (const auto &joint: m_joints) {
        totalLength += joint.length;
    }

    m_maxReach = totalLength;
}

const std::vector<Joint> &KinematicChain::getJoints() const {
    return m_joints;
}

void KinematicChain::addJoint(const Joint &joint) {
    m_joints.emplace_back(joint);
}

void KinematicChain::removeJoint() {
    if(m_joints.empty()) {
        throw std::out_of_range("cannot remove joints if there aren´t any");
    }
    if (!m_joints.empty())
        m_joints.pop_back();
}

void KinematicChain::targetPosition(const Eigen::Vector2f &position) {
    m_newVectorPosition = position;
}
Eigen::Vector2f &KinematicChain::getTargetPosition() {
    return m_newVectorPosition;
}
void KinematicChain::circularMotion(Eigen::Vector2f &position, float radius) {
    //static auto start = std::chrono::steady_clock::now(); //gpt
    //float time = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count(); //gpt
    // Gotten here: https://stackoverflow.com/questions/15957805/extract-year-month-day-etc-from-stdchronotime-point-in-c/15958113#15958113
    // Get the current time as a time_point
    auto now = std::chrono::system_clock::now();

    // Convert to time_t for use with C APIs
    std::time_t nowCTime = std::chrono::system_clock::to_time_t(now);

    // Convert to tm struct (local time)
    std::tm localTime = *std::localtime(&nowCTime);

    int hours = localTime.tm_hour;
    int minutes = localTime.tm_min;
    int seconds = localTime.tm_sec;
    float timeStepMin = 2 * std::numbers::pi / 60;

    position.x() = std::sin(seconds * timeStepMin)  *  radius * 1.25 ;
    position.y() = std::cos(seconds * timeStepMin)  *  radius * 1.25 ;
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

Eigen::MatrixXf KinematicChain::computeJacobianTranspose() const {
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

std::vector<float> KinematicChain::computeCumulativeAngles() const {
    std::vector<float> cumulativeAngles(m_joints.size());
    float cumulativeAngle = 0.0f;

    for (size_t i = 0; i < m_joints.size(); ++i) {
        cumulativeAngle += m_joints[i].angle;
        cumulativeAngles[i] = cumulativeAngle;
    }
    return cumulativeAngles;
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

void KinematicChain::inverseKinematicsHandler(const Eigen::Vector2f &targetPosition, float learningRate, float threshold, int maxIteration) {
    for (size_t i = 0; i < maxIteration; ++i) {
            if(!hasConverged(targetPosition, threshold)) {
                errorHandler(targetPosition, learningRate);
            }
        }
    }

bool KinematicChain::hasConverged(const Eigen::Vector2f &targetPosition, float threshold) const {
    Eigen::Vector2f error = computeError(targetPosition);
    return error.norm() < threshold;
}

void KinematicChain::errorHandler(const Eigen::Vector2f &targetPosition, float learningRate) {
    Eigen::Vector2f error = computeError(targetPosition);
    adjustErrorMagnitude(error);

    Eigen::VectorXf angleAdjustments = computeAngleAdjustments(error, learningRate);
    updateJointAngles(angleAdjustments);
}

Eigen::Vector2f KinematicChain::computeError(const Eigen::Vector2f &targetPosition) const {
    Eigen::Vector2f currentPosition = findEffectorPosition();
    Eigen::Vector2f error = targetPosition - currentPosition;

    return error;
}

void KinematicChain::adjustErrorMagnitude( Eigen::Vector2f &error) const {
    float errorMagnitude = error.norm();
    float maxErrorMagnitutde = 0.001f;//define how precise the end effector should be

    if (errorMagnitude > maxErrorMagnitutde) {
        error *= maxErrorMagnitutde / errorMagnitude;
    }
}

Eigen::VectorXf KinematicChain::computeAngleAdjustments(const Eigen::Vector2f &error, float learningRate) const {
    Eigen::MatrixXf jacobianTranspose = computeJacobianTranspose();
    Eigen::VectorXf angleAdjustments = learningRate * jacobianTranspose * error;

    float maxAngleChange = 0.002f;//avoid abrupt angle changes
    for (int i = 0; i < angleAdjustments.size(); ++i) {
        angleAdjustments(i) = std::clamp(angleAdjustments(i), -maxAngleChange, maxAngleChange);
    }
    return angleAdjustments;
}

void KinematicChain::updateJointAngles(const Eigen::VectorXf &angleAdjustments) {
    for (size_t i = 0; i < m_joints.size(); ++i) {
        m_joints[i].angle += angleAdjustments(i);
        m_joints[i].angle = clampAngle(m_joints[i].angle);
    }
}

/* @brief limit angle change to 0-2π tp prevent unwated behavior */
float KinematicChain::clampAngle(float angle) {
    angle = std::fmod(angle, 2 * std::numbers::pi);

    if (angle < 0.0f) { //er dette nødvendig?
        angle += 2 * std::numbers::pi;
    }
    return angle;
}



