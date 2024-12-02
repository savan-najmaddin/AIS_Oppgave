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
    void update(KinematicChain &chain, const MyUI &ui, VisualJoints &visualJoints, threepp::Scene &scene,
            const MySpheres &mySphere, float learningRate);


private:
    std::shared_ptr<threepp::SphereGeometry> m_geometry;
    int m_prevNumJoints = 0;

    static void jointResize(KinematicChain &chain, const MyUI &ui) ;

    void updateMesh(KinematicChain &chain, VisualJoints &visualJoints, threepp::Scene &scene,
                    const MySpheres &mySphere);

    int getPrevNumJoints() const;

    void setPrevNumJoints(int prevNumJoints);

    static void unlimitedGems(const MyUI &ui);
};


#endif//HANDLER_HPP