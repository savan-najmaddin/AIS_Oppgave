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

    explicit Joint(float ang = 0.0f, float len = 1.0f);//explicit for å forhindre feil konvertering
};

class KinematicChain {
public:
    enum class TimeUnit {
        SECONDS,
        MINUTES,
        HOURS
    };

    explicit KinematicChain(size_t n = 0);

    const std::vector<Joint> &getJoints() const;
    void addJoint(const Joint &joint);
    void removeJoint();

    void setTargetPosition(const Eigen::Vector2f &position);
    Eigen::Vector2f &getTargetPosition();

    void showTime(TimeUnit unit);

    float getMaxReach() const;
    void updateMaxReach();

    Eigen::Vector2f findEffectorPosition() const;

    void inverseKinematicsHandler(float learningRate,
                                  float threshold = 0.1f, int maxIteration = 10);

private:
    std::vector<Joint> m_joints;
    float m_maxReach;
    Eigen::Vector2f m_targetPosition;

    static float clampAngle(float angle);
    bool hasConverged(float threshold) const;
    void errorHandler(float learningRate);
    void adjustErrorMagnitude(Eigen::Vector2f& error) const;
    void updateJointAngles(const Eigen::VectorXf& angleAdjustments);

    Eigen::MatrixXf computeJacobianTranspose() const;
    Eigen::VectorXf computeAngleAdjustments(const Eigen::Vector2f& error, float learningRate) const;
    Eigen::Vector2f computeError() const;
    std::vector<float> computeCumulativeAngles() const;
    std::pair<float, float> computePartialDerivates(size_t i, const std::vector<float> &cumulativeAngle) const;

};

#endif