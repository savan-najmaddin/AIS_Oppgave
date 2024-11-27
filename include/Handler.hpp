#ifndef HANDLER_HPP
#define HANDLER_HPP

#include "ImGui.hpp"
#include "Logic.hpp"
#include "Visual.hpp"
#include "threepp/threepp.hpp"

#include <Objects.hpp>


class Handler {

public:

  Handler(KinematicChain& chain, const MyUI& ui, VisualJoints& visualJoints,  Scene& scene,
    MySpheres& mySphere);

  static void jointResize(KinematicChain& chain,const MyUI& ui);

  static void updateMesh(KinematicChain& chain, VisualJoints& visualJoints,  Scene& scene,
   MySpheres& mySphere, int& prevNumJoints);

private:

  std::shared_ptr<SphereGeometry> m_geometry;
  int m_prevNumJoints;






};


#endif//HANDLER_HPP