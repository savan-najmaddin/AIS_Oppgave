/**
 * @brief this class is responsible for the inverse kinematics and targetpositions
 */

#ifndef LOGIKK_HPP
#define LOGIKK_HPP

#include "Eigen/Core"
#include <vector>


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

    // This code is used to get the current time of the system
    void showTime(TimeUnit unit);

    float getMaxReach() const;
    void updateMaxReach();

    Eigen::Vector2f findEffectorPosition() const;

    void inverseKinematicsHandler(float learningRate,
                                  float threshold = 0.01f, int maxIteration = 1000);

    bool getHasConverged(float threshold) const;

private:
    std::vector<Joint> m_joints;
    float m_maxReach;
    Eigen::Vector2f m_targetPosition;

    static float clampAngle(float angle);
    bool hasConverged(float threshold) const;
    void errorHandler(float learningRate);
    static void adjustErrorMagnitude(Eigen::Vector2f &error);
    void updateJointAngles(const Eigen::VectorXf &angleAdjustments);

    Eigen::MatrixXf computeJacobianTranspose() const;
    Eigen::VectorXf computeAngleAdjustments(const Eigen::Vector2f &error, float learningRate) const;
    Eigen::Vector2f computeError() const;
    std::vector<float> computeCumulativeAngles() const;
    std::pair<float, float> computePartialDerivates(size_t i, const std::vector<float> &cumulativeAngle) const;
};

#endif
