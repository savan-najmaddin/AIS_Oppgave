#ifndef HANDLER_HPP
#define HANDLER_HPP

#include "ImGui.hpp"
#include "Logic.hpp"
#include "Visual.hpp"
#include "threepp/threepp.hpp"

#include <Objects.hpp>


class Handler {

public:
    Handler() = default;
    void update(KinematicChain &chain, const MyUI &ui, VisualJoints &visualJoints, Scene &scene,
            MySpheres &mySphere, float learningRate);


private:
    std::shared_ptr<SphereGeometry> m_geometry;
    int m_prevNumJoints = 0;

    void jointResize(KinematicChain &chain, const MyUI &ui) const;

    void updateMesh(KinematicChain &chain, VisualJoints &visualJoints, Scene &scene,
                    MySpheres &mySphere);

    int getPrevNumJoints() const;

    void setPrevNumJoints(int prevNumJoints);

    void unlimitedGems(const MyUI &ui);
};


#endif//HANDLER_HPP