#include "Logikk.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Joint::Joint(float ang, float len) : angle(ang), length(len) {}
KinematicChain::KinematicChain(size_t n) : numJoints(n), joints(n) {//beholde size_t ?
    float totalLength = 0.0f;
    for (const auto &joint: joints) {
        totalLength += joint.length;
    }

    maxReach = totalLength;
}
void KinematicChain::addJoint(const Joint &joint) {
    joints.emplace_back(joint);
    numJoints = joints.size();
}
float getMaxReach(KinematicChain& chain) { //dette burde kunne skrives bedre
    float maxReach{0};
    for (size_t i = 0; i < chain.numJoints; ++i) {
        maxReach += chain.joints[i].length;
    }
    return maxReach;
}
void KinematicChain::targetPosition(Eigen::Vector2f &position) {
    newVectorPosition = position;
}
const Eigen::Vector2f &KinematicChain::getTargetPosition() const {
    return newVectorPosition;
}
float KinematicChain::clampAngle(float angle) {
    angle = std::fmod(angle, 2 * M_PI);//mod av 2π

    if (angle < 0.0f) {
        angle += 2 * M_PI;
    }

    return angle;
}
Eigen::Vector2f KinematicChain::findEffectorPosition() {
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativAngle = 0.0f;

    for (const auto &joint: joints) {
        cumulativAngle += joint.angle;
        position.x() += joint.length * std::cos(cumulativAngle);
        position.y() += joint.length * std::sin(cumulativAngle);
    }
    return position;
}
Eigen::MatrixXf KinematicChain::computeJacobianTranspose() const {
    Eigen::MatrixXf jacobianTranspose(numJoints, 2);
    jacobianTranspose.setZero();

    // Precompute cumulative angles up to each joint
    std::vector<float> cumulativeAngles(numJoints);
    float cumulativeAngle = 0.0f;

    for (size_t i = 0; i < numJoints; ++i) {
        cumulativeAngle += joints[i].angle;
        cumulativeAngles[i] = cumulativeAngle;
    }

    // For each joint i
    for (size_t i = 0; i < numJoints; ++i) {
        float partialX = 0.0f;
        float partialY = 0.0f;

        // For k from i to numJoints - 1
        //funksjonen burde være mindre
        for (size_t k = i; k < numJoints; ++k) {
            float angleSum = cumulativeAngles[k];
            float dx_dtheta = -joints[k].length * std::sin(angleSum);
            float dy_dtheta = joints[k].length * std::cos(angleSum);

            partialX += dx_dtheta;
            partialY += dy_dtheta;
        }

        jacobianTranspose(i, 0) = partialX;
        jacobianTranspose(i, 1) = partialY;
    }
    return jacobianTranspose;
}

void KinematicChain::updateJointAngles(const Eigen::VectorXf &angleAdjustments) {
    for (size_t i = 0; i < numJoints; ++i) {
        joints[i].angle += angleAdjustments(i);
        joints[i].angle = clampAngle(joints[i].angle);
    }
}

//funksjonen burde være mindre
void KinematicChain::updateInverseKinematics(const Eigen::Vector2f &targetPosition, float learningRate, float threshold, int maxIteration) {
    for (int iter = 0; iter < maxIteration; ++iter) {
        Eigen::Vector2f currentPosition = findEffectorPosition();
        Eigen::Vector2f error = targetPosition - currentPosition;
        float errorMagnitude = error.norm();

        if (errorMagnitude < threshold) {
            break;
        }

        // dette gjør sånn at armen ikke beveger seg raskt
        float maxErrorMagnitude = 0.01f;
        if (errorMagnitude > maxErrorMagnitude) {
            error = error.normalized() * maxErrorMagnitude;
        }

        Eigen::MatrixXf JacobianTranspose = computeJacobianTranspose();
        Eigen::VectorXf angleAdjustments = learningRate * JacobianTranspose * error;

        // Limit the angle adjustments
        float maxAngleChange = 0.05f;
        for (int i = 0; i < angleAdjustments.size(); ++i) {
            if (angleAdjustments(i) > maxAngleChange) {
                angleAdjustments(i) = maxAngleChange;
            } else if (angleAdjustments(i) < -maxAngleChange) {
                angleAdjustments(i) = -maxAngleChange;
            }
        }

        updateJointAngles(angleAdjustments);
    }
}