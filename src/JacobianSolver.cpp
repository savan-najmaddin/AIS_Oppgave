#include "JacobianSolver.hpp"
#include <algorithm>

void JacobianSolver::solve(KinematicChain &chain, float learningRate,
                           float threshold, int maxIterations) {
    for (int i = 0; i < maxIterations; ++i) {
        if (hasConverged(chain, threshold)) {
            break;
        }
        Eigen::Vector2f error = computeError(chain);
        adjustErrorMagnitude(error);
        Eigen::VectorXf adjustments =
                computeAngleAdjustments(chain, error, learningRate);
        updateJointAngles(chain, adjustments);
    }
}

bool JacobianSolver::hasConverged(const KinematicChain &chain, float threshold) const {
    Eigen::Vector2f error = computeError(chain);
    return error.norm() < threshold;
}

Eigen::Vector2f JacobianSolver::computeError(const KinematicChain &chain) const {
    Eigen::Vector2f currentPosition = chain.findEffectorPosition();
    Eigen::Vector2f error = chain.getTargetPosition() - currentPosition;
    return error;
}

void JacobianSolver::adjustErrorMagnitude(Eigen::Vector2f &error) {
    const float errorMagnitude = error.norm();
    constexpr float maxErrorMagnitude = 0.0001f;
    if (errorMagnitude > maxErrorMagnitude) {
        error *= maxErrorMagnitude / errorMagnitude;
    }
}

Eigen::MatrixXf JacobianSolver::computeJacobianTranspose(const KinematicChain &chain) const {
    Eigen::MatrixXf jacobianTranspose(chain.getJoints().size(), 2);
    jacobianTranspose.setZero();
    std::vector<float> cumulativeAngles = computeCumulativeAngles(chain);

    for (size_t i = 0; i < chain.getJoints().size(); ++i) {
        auto [partialX, partialY] = computePartialDerivatives(chain, i, cumulativeAngles);
        jacobianTranspose(i, 0) = partialX;
        jacobianTranspose(i, 1) = partialY;
    }
    return jacobianTranspose;
}

std::vector<float> JacobianSolver::computeCumulativeAngles(const KinematicChain &chain) const {
    std::vector<float> cumulativeAngles(chain.getJoints().size());
    float cumulativeAngle = 0.0f;

    for (size_t i = 0; i < chain.getJoints().size(); ++i) {
        cumulativeAngle += chain.getJoints()[i].angle;
        cumulativeAngles[i] = cumulativeAngle;
    }
    return cumulativeAngles;
}

std::pair<float, float>
JacobianSolver::computePartialDerivatives(const KinematicChain &chain, size_t i,
                                          const std::vector<float> &cumulativeAngles) const {
    float partialX = 0.0f;
    float partialY = 0.0f;

    for (size_t j = i; j < chain.getJoints().size(); ++j) {
        float angleSum = cumulativeAngles[j];
        float dx_dtheta = -chain.getJoints()[j].length * std::sin(angleSum);
        float dy_dtheta = chain.getJoints()[j].length * std::cos(angleSum);
        partialX += dx_dtheta;
        partialY += dy_dtheta;
    }
    return {partialX, partialY};
}

Eigen::VectorXf
JacobianSolver::computeAngleAdjustments(const KinematicChain &chain,
                                        const Eigen::Vector2f &error,
                                        float learningRate) const {
    Eigen::MatrixXf jacobianTranspose = computeJacobianTranspose(chain);
    Eigen::VectorXf angleAdjustments = learningRate * jacobianTranspose * error;
    for (int i = 0; i < angleAdjustments.size(); ++i) {
        constexpr float maxAngleChange = 0.002f;
        angleAdjustments(i) = std::clamp(angleAdjustments(i), -maxAngleChange, maxAngleChange);
    }
    return angleAdjustments;
}

void JacobianSolver::updateJointAngles(KinematicChain &chain,
                                       const Eigen::VectorXf &angleAdjustments) const {
    for (size_t i = 0; i < chain.getJoints().size(); ++i) {
        chain.getJoints()[i].angle += angleAdjustments(i);
        chain.getJoints()[i].angle = clampAngle(chain.getJoints()[i].angle);
    }
}

float JacobianSolver::clampAngle(float angle) {
    angle = std::fmod(angle, 2 * std::numbers::pi);
    if (angle < 0.0f) {
        angle += 2 * std::numbers::pi;
    }
    return angle;
}

