#include "Logic.hpp"
#include <chrono>
#include <numbers>
#include <stdexcept>

//bevegelsen til IK kjeden gjort numerisk
/**
 * @brief Inverse Kinematic motion done numerically using the Jacobian
 *
 *
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


Joint::Joint(const float ang, const float len) : angle(ang), length(len) {}

void KinematicChain::setTargetPosition(const Eigen::Vector2f &position) {
    m_targetPosition = position;
}
Eigen::Vector2f &KinematicChain::getTargetPosition() {
    return m_targetPosition;
}
void KinematicChain::showTime(const TimeUnit unit) {
    // Gotten here: https://stackoverflow.com/questions/15957805/extract-year-month-day-etc-from-stdchronotime-point-in-c/15958113#15958113
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowCTime = std::chrono::system_clock::to_time_t(now);
    const std::tm localTime = *std::localtime(&nowCTime);

    int t = 0;
    float timeStep = 2 * std::numbers::pi;

    switch (unit) {
        case TimeUnit::SECONDS:
            t = localTime.tm_sec;
            timeStep /= 60.0f;
            break;
        case TimeUnit::MINUTES:
            t = localTime.tm_min;
            timeStep /= 60.0f;
            break;
        case TimeUnit::HOURS:
            t = localTime.tm_hour;
            timeStep /= 12.0f;
            break;
    }

    m_targetPosition.x() = std::sin(t * timeStep) * m_maxReach * 1.25;
    m_targetPosition.y() = std::cos(t * timeStep) * m_maxReach * 1.25;
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

Eigen::MatrixXf KinematicChain::computeJacobianTranspose() const {
    Eigen::MatrixXf jacobianTranspose(m_joints.size(), 2);
    jacobianTranspose.setZero();
    const std::vector<float> cumulativeAngles = computeCumulativeAngles();

    for (size_t i = 0; i < m_joints.size(); ++i) {
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
        const size_t i, const std::vector<float> &cumulativeAngle) const {
    float partialX = 0.0f;
    float partialY = 0.0f;

    for (size_t j = i; j < m_joints.size(); ++j) {
        const float angleSum = cumulativeAngle[j];
        const float dx_dtheta = -m_joints[j].length * std::sin(angleSum);
        const float dy_dtheta = m_joints[j].length * std::cos(angleSum);

        partialX += dx_dtheta;
        partialY += dy_dtheta;
    }
    return {partialX, partialY};
}

void KinematicChain::inverseKinematicsHandler(const float learningRate, const float threshold, const int maxIteration) {
    for (size_t i = 0; i < maxIteration; ++i) {
        if (!hasConverged(threshold)) {
            errorHandler(learningRate);
        }
    }
}

bool KinematicChain::hasConverged(const float threshold) const {
    Eigen::Vector2f const error = computeError();
    return error.norm() < threshold;
}
bool KinematicChain::getHasConverged(const float threshold) const {
    return hasConverged(threshold);
}

void KinematicChain::errorHandler(const float learningRate) {
    Eigen::Vector2f error = computeError();
    adjustErrorMagnitude(error);

    Eigen::VectorXf const angleAdjustments = computeAngleAdjustments(error, learningRate);
    updateJointAngles(angleAdjustments);
}
Eigen::Vector2f KinematicChain::computeError() const {
    Eigen::Vector2f const currentPosition = findEffectorPosition();
    Eigen::Vector2f error = m_targetPosition - currentPosition;

    return error;
}
void KinematicChain::adjustErrorMagnitude(Eigen::Vector2f &error) {
    const float errorMagnitude = error.norm();
    constexpr float maxErrorMagnitutde = 0.0001f;//define how precise the end effector should be

    if (errorMagnitude > maxErrorMagnitutde) {
        error *= maxErrorMagnitutde / errorMagnitude;
    }
}

Eigen::VectorXf KinematicChain::computeAngleAdjustments(const Eigen::Vector2f &error, float learningRate) const {
    const Eigen::MatrixXf jacobianTranspose = computeJacobianTranspose();
    Eigen::VectorXf angleAdjustments = learningRate * jacobianTranspose * error;

    for (int i = 0; i < angleAdjustments.size(); ++i) {
        constexpr float maxAngleChange = 0.002f;//limits abrupt changes in angle
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
    angle = std::fmod(angle, 2 * std::numbers::pi);//gpt recommended fmod

    if (angle < 0.0f) {
        angle += 2 * std::numbers::pi;
    }
    return angle;
}

