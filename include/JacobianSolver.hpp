#ifndef JACOBIANSOLVER_HPP
#define JACOBIANSOLVER_HPP

#include "IKSolver.hpp"
#include <numbers>

class JacobianSolver : public IKSolver {
public:
    void solve(KinematicChain &chain, float learningRate,
               float threshold = 0.01f, int maxIterations = 1000) override;

private:
    static float clampAngle(float angle);
    static void adjustErrorMagnitude(Eigen::Vector2f &error);

    Eigen::Vector2f computeError(const KinematicChain &chain) const;
    Eigen::MatrixXf computeJacobianTranspose(const KinematicChain &chain) const;

    Eigen::VectorXf computeAngleAdjustments(const KinematicChain &chain,
                            const Eigen::Vector2f &error,
                            float learningRate) const;

    std::vector<float> computeCumulativeAngles(const KinematicChain &chain) const;

    std::pair<float, float> computePartialDerivatives(const KinematicChain &chain, size_t i,
                              const std::vector<float> &cumulativeAngles) const;

    void updateJointAngles(KinematicChain &chain,
                           const Eigen::VectorXf &angleAdjustments) const;
    bool hasConverged(const KinematicChain &chain, float threshold) const;
};

#endif // JACOBIANSOLVER_HPP
