#include "CCDSolver.hpp"

void CCDSolver::solve(KinematicChain &chain, float learningRate,
                      float threshold, int maxIterations) {
    for (int iter = 0; iter < maxIterations; ++iter) {
        Eigen::Vector2f end = chain.findEffectorPosition();
        Eigen::Vector2f error = chain.getTargetPosition() - end;
        if (error.norm() < threshold) {
            break;
        }
        for (int i = static_cast<int>(chain.getJoints().size()) - 1; i >= 0; --i) {
            Eigen::Vector2f jointPos(0.f, 0.f);
            float angleSum = 0.f;
            for (int j = 0; j < i; ++j) {
                angleSum += chain.getJoints()[j].angle;
                jointPos.x() += chain.getJoints()[j].length * std::cos(angleSum);
                jointPos.y() += chain.getJoints()[j].length * std::sin(angleSum);
            }
            end = chain.findEffectorPosition();
            Eigen::Vector2f toEnd = end - jointPos;
            Eigen::Vector2f toTarget = chain.getTargetPosition() - jointPos;

            float angleEnd = std::atan2(toEnd.y(), toEnd.x());
            float angleTarget = std::atan2(toTarget.y(), toTarget.x());
            float delta = angleTarget - angleEnd;
            chain.getJoints()[i].angle += learningRate * delta;
        }
    }
}
