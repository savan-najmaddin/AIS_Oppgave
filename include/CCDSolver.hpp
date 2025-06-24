#ifndef CCDSOLVER_HPP
#define CCDSOLVER_HPP

#include "IKSolver.hpp"
#include <numbers>
//Todo create a CCD solver
class CCDSolver : public IKSolver {
public:
    void solve(KinematicChain &chain, float learningRate,
               float threshold = 0.01f, int maxIterations = 1000) override;
};

#endif // CCDSOLVER_HPP
