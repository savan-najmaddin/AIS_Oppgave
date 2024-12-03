/**
 * @brief this class is responsible for the visual representation of the joints
 */

#ifndef VISUAL_HPP
#define VISUAL_HPP

#include "Logic.hpp"
#include "threepp/threepp.hpp"


class VisualJoints {
public:
    void setChain(threepp::Scene &scene, const KinematicChain &chain);

    //this can be a const since the pointer doesn´t change any values directly
    void updateJointVisual(const KinematicChain &chain) const;

    std::vector<std::shared_ptr<threepp::Mesh>> joints;

private:
    void addJoint(threepp::Scene &scene, const KinematicChain &chain);
    void removeJoint(threepp::Scene &scene);
    static constexpr float m_width = 0.2f;
};


#endif
