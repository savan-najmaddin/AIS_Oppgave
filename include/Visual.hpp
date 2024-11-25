#ifndef VISUAL_HPP
#define VISUAL_HPP

#include "threepp/threepp.hpp"
#include "Logikk.hpp"
#include "iostream"

class VisualJoints {
public:
  void setChain(threepp::Scene &scene, const KinematicChain &chain);

  void update(const KinematicChain &chain);

private:
    static constexpr float WIDTH = 0.2f;

    std::vector<std::shared_ptr<threepp::Mesh>> joints;
};
#endif
