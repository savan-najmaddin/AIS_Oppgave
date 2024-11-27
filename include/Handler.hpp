#ifndef HANDLER_HPP
#define HANDLER_HPP

#include "ImGui.hpp"
#include "Logic.hpp"
#include "Visual.hpp"
#include "threepp/threepp.hpp"


class Handler {

public:

  Handler(Scene& scene, KinematicChain& chain, MyUI& ui, VisualJoints& visualJoints)
    :m_prevNumJoints{0} ,
  m_scene(scene),
  m_chain(chain), m_ui(ui),
  m_visualJoints(visualJoints) {}

  void jointResize(KinematicChain& chain, MyUI& ui);

  void updateMesh(KinematicChain& chain, VisualJoints& visualJoints, Scene& scene);

private:
  std::shared_ptr<SphereGeometry> m_geometry;
  int m_prevNumJoints;

  Scene& m_scene;
  KinematicChain& m_chain;
  MyUI& m_ui;
  VisualJoints& m_visualJoints;




};


#endif//HANDLER_HPP