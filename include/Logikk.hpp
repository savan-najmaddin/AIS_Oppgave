#ifndef LOGIKK_HPP
#define LOGIKK_HPP



//a. Beregning av Jacobian-transposematrisen
//b. Multiplikasjon av en vektor med Jacobian-transposen
//c. Beregning av Jacobian multiplisert med Jacobian-transposen (J * J^T)
//d. Selve løseren.

#include <cstddef> //for size_t



struct Joint {
  float angle;
  float length;

  Joint(float ang = 0.0f, float len = 1.0f) : angle(ang), length(len) {}
};

class kinematicChain {
public:
  size_t numJoints; //antall ledd i kjeden
  std::vector<Joint> joints; //array av ledd

  kinematicChain(size_t n = 0) : numJoints(n), joints(n){}

  void addJoint(const Joint& joint) {
    joints.emplace_back(joint);
    numJoints = joints.size();
  }

  static float clampAngle (float angle) {
    //legg til metode for å begrense vinkler på joints, [0, 2π]
    return 0; //fjern senere
  }

  Eigen::Vector2f findEndEffectorPosition() {
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativAngle = 0.0f;

    for (const auto& joint : joints) {
      cumulativAngle += joint.angle;
      position.x() += joint.length * std::cos(cumulativAngle);
      position.y() += joint.length * std::sin(cumulativAngle);
    }
    return position;
  }


};

#endif