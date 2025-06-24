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

    explicit KinematicChain(size_t n = 0);

    const std::vector<Joint> &getJoints() const;
    std::vector<Joint> &getJoints();
    void addJoint(const Joint &joint);
    void removeJoint();

    void setTargetPosition(const Eigen::Vector2f &position);
    const Eigen::Vector2f  &getTargetPosition() const;

    float getMaxReach() const;
    void updateMaxReach();

    Eigen::Vector2f findEffectorPosition() const;

private:
    std::vector<Joint> m_joints;
    float m_maxReach;
    Eigen::Vector2f m_targetPosition;

    static float clampAngle(float angle);
};

#endif
