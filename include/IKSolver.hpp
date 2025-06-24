#ifndef IKSOLVER_HPP
#define IKSOLVER_HPP

#include "Logic.hpp"

class IKSolver {
public:
    virtual ~IKSolver() = default;
    virtual void solve(KinematicChain &chain, float learningRate,
                       float threshold = 0.01f, int maxIterations = 1000) = 0;
};

#endif // IKSOLVER_HPP
