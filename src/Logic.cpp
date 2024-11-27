#include "Logic.hpp"
//bytt til std pi
#ifndef M_PI
#define M_PI 3.14159265358979323846 //pga windows
#endif

Joint::Joint(float ang, float len) : angle(ang), length(len) {}

//vet at det burde være 2 klasser, men jeg har dårlig tid
KinematicChain::KinematicChain(size_t n) : joints(n) {
    float totalLength = 0.0f;
    for (const auto &joint: joints) {
        totalLength += joint.length;
    }

    m_maxReach = totalLength;
}

void KinematicChain::addJoint(const Joint &joint) {
    joints.emplace_back(joint);
}
void KinematicChain::removeJoint() {
    joints.pop_back();
}

void KinematicChain::targetPosition(Eigen::Vector2f &position) {
    m_newVectorPosition = position;
}

const Eigen::Vector2f &KinematicChain::getTargetPosition() const {
    return m_newVectorPosition;
}

float KinematicChain::getMaxReach() const {
    return m_maxReach;
}
void KinematicChain::updateMaxReach() {
    float totalLength{0};
    for (const auto &joint: joints) {
        totalLength += joint.length;
    }
    m_maxReach = totalLength;
}

Eigen::Vector2f KinematicChain::findEffectorPosition() const {
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativAngle = 0.0f;

    for (const auto &joint: joints) {
        cumulativAngle += joint.angle;
        position.x() += joint.length * std::cos(cumulativAngle);
        position.y() += joint.length * std::sin(cumulativAngle);
    }
    return position;
}

std::vector<float> KinematicChain::computeCumulativeAngels() const {

    std::vector<float> cumulativeAngles(joints.size());
    float cumulativeAngle = 0.0f;

    for (size_t i = 0; i < joints.size(); ++i) {
        cumulativeAngle += joints[i].angle;
        cumulativeAngles[i] = cumulativeAngle;
    }
    return cumulativeAngles;
}

Eigen::MatrixXf KinematicChain::computeJacobianTranspose() const {
    Eigen::MatrixXf jacobianTranspose(joints.size(), 2);
    jacobianTranspose.setZero();

    std::vector<float> cumulativeAngles = computeCumulativeAngels();

    for (size_t i = 0; i < joints.size(); ++i) {
        float partialX = 0.0f;
        float partialY = 0.0f;


        for (size_t j = i; j < joints.size(); ++j) {
            float angleSum = cumulativeAngles[j];
            float dx_dtheta = -joints[j].length * std::sin(angleSum);
            float dy_dtheta = joints[j].length * std::cos(angleSum);

            partialX += dx_dtheta;
            partialY += dy_dtheta;
        }

        jacobianTranspose(i, 0) = partialX;
        jacobianTranspose(i, 1) = partialY;
    }
    return jacobianTranspose;
}

void KinematicChain::updateJointAngles(const Eigen::VectorXf &angleAdjustments) {
    for (size_t i = 0; i < joints.size(); ++i) {
        joints[i].angle += angleAdjustments(i);
        joints[i].angle = clampAngle(joints[i].angle);
    }
}

void KinematicChain::updateInverseKinematics(const Eigen::Vector2f &targetPosition, float learningRate, float threshold, int maxIteration) {
    for (size_t i = 0; i < maxIteration; ++i) {
        Eigen::Vector2f error = computeError(targetPosition);
        float errorMagnitude = error.norm();

        if (error.norm() < threshold) {
            break;
        }

        float maxErrorMagnitutde = 0.001f;//define how precise the end effector should be
        if (errorMagnitude > maxErrorMagnitutde) {
            error *= maxErrorMagnitutde / errorMagnitude;
        }

        Eigen::VectorXf angleAdjustments = computeAngleAdjustments(error, learningRate);
        updateJointAngles(angleAdjustments);
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

std::vector<Joint> KinematicChain::getJoints() const {
    return joints;
}


float KinematicChain::clampAngle(float angle) {
    angle = std::fmod(angle, 2 * M_PI); //mod av 2π

    if (angle < 0.0f) {
        angle += 2 * M_PI;
    }

    return angle;
}