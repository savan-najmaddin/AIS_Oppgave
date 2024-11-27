//a. Beregning av Jacobian-transposematrisen
//b. Multiplikasjon av en vektor med Jacobian-transposen
//c. Beregning av Jacobian multiplisert med Jacobian-transposen (J * J^T)
//d. Selve løseren.

//bevegelsen til IK kjeden gjort numerisk

#ifndef LOGIKK_HPP
#define LOGIKK_HPP


#include "Eigen/Core"
#include <iostream>
#include <vector>


class KinematicChain;
struct Joint {
    float angle;
    float length;

    explicit Joint(float ang = 0.0f, float len = 1.0f); //explicit for å forhindre feil konvertering
};

class KinematicChain {
public:
    std::vector<Joint> joints;

    explicit KinematicChain(size_t n = 0);

    void addJoint(const Joint &joint);
    void removeJoint();

    void targetPosition(Eigen::Vector2f &position);
    const Eigen::Vector2f& getTargetPosition() const;

    float getMaxReach() const;
    void updateMaxReach();

    // Google når nodiscard skal brukes
    [[nodiscard]] Eigen::Vector2f findEffectorPosition() const;

    [[nodiscard]] std::vector<float> computeCumulativeAngels() const;

    [[nodiscard]] Eigen::MatrixXf computeJacobianTranspose() const;

    void updateJointAngles(const Eigen::VectorXf &angleAdjustments);

    void updateInverseKinematics(const Eigen::Vector2f &targetPosition, float learningRate,
                                 float threshold = 0.1f, int maxIteration = 10);

    Eigen::Vector2f computeError(const Eigen::Vector2f& targetPosition) const;

    Eigen::VectorXf computeAngleAdjustments(const Eigen::Vector2f& error, float learningRate) const;

    std::vector<Joint> getJoints() const;

private:
    static float clampAngle(float angle);

    Eigen::Vector2f m_newVectorPosition{6.0f, 3.0f};
    float m_maxReach;
};

#endif