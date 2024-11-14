#ifndef LOGIKK_HPP
#define LOGIKK_HPP



//a. Beregning av Jacobian-transposematrisen
//b. Multiplikasjon av en vektor med Jacobian-transposen
//c. Beregning av Jacobian multiplisert med Jacobian-transposen (J * J^T)
//d. Selve løseren.

//bevegelsen til IK kjeden gjort numerisk

#include "Eigen/Core"
#include <cmath>
#include <iostream>
#include <vector>


struct Joint {
  float angle;
  float length;

  Joint(float ang = 0.0f, float len = 1.0f) : angle(ang), length(len) {}
};

class kinematicChain {
public:
  size_t numJoints; //antall ledd i kjeden
  std::vector<Joint> joints; //array av ledd
  float maxReach;

  explicit kinematicChain(size_t n = 0) : numJoints(n), joints(n) {
    float totalLength = 0.0f;
    for (const auto &joint : joints) {
      totalLength += joint.length;
    }

    maxReach = totalLength;
  }

  void addJoint(const Joint& joint) {
    joints.emplace_back(joint);
    numJoints = joints.size();
  }


  Eigen::Vector2f targetPosition(Eigen::Vector2f position = {2, 3}) {
    std::cout << "pos x: " << position.x() << "pos y: " << position.y() << std::endl;
    return position;
  }



  static float clampAngle (float angle) {
    constexpr float Two_PI = 2.0f * M_PI;

    angle = std::fmod(angle, Two_PI); //mod av 2π

    if (angle < 0.0f) {
      angle += Two_PI;
    }

    return angle;
  }


  // Google når nodiscard skal brukes
  [[nodiscard]] Eigen::Vector2f findEffectorPosition() {
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativAngle = 0.0f;

    for (const auto& joint : joints) {
      cumulativAngle += joint.angle;
      position.x() += joint.length * std::cos(cumulativAngle);
      position.y() += joint.length * std::sin(cumulativAngle);
    }
    return position;
  }


  [[nodiscard]] Eigen::MatrixXf computeJacobianTranspose() const { //returnerer transposen til matrisen
    Eigen::MatrixXf jacobianTranspose(  numJoints, 2); //matrixXf er dynamisk
    jacobianTranspose.setZero(); //setter alle elementer i matrisen til 0

    float cumulativeAngle = 0.0f; //summen av vinkler

    for (size_t i = 0; i < numJoints; ++i) {
      cumulativeAngle = 0.0f;
      // Beregn summen av vinkler fra ledd 0 til i
      for (size_t j = 0; j <= i; ++j) {
        cumulativeAngle += joints[j].angle;
      }

      // Beregn partiellderivertene
      float partialX = 0.0f;
      float partialY = 0.0f;

      float angleSum = cumulativeAngle;
      for (size_t j = i; j < numJoints; ++j) {
        angleSum += joints[j].angle;
        partialX -= joints[j].length * std::sin(angleSum);
        partialY += joints[j].length * std::cos(angleSum);
      }

      jacobianTranspose( i, 0) = partialX;
      jacobianTranspose( i, 1) = partialY;
    }
    return jacobianTranspose;
  }


  void updateJointAngles(const Eigen::VectorXf& angleAdjustments) {
    for (size_t i = 0; i < numJoints; ++i) {
      joints[i].angle += angleAdjustments(i);
      joints[i].angle = clampAngle(joints[i].angle);
    }
  }


  void updateInverseKinematics(Eigen::Vector2f& targetPosition, float learningRate,
                            float const threshold = 0.01f, int maxIteration = 10000000) {

    Eigen::Vector2f currentPosition = findEffectorPosition();
    Eigen::Vector2f error = targetPosition - currentPosition;
    float errorMagnitude = error.norm(); //absolutt verdien til error mellom target  og current position

    //kan skrives status til konsoll

    if (errorMagnitude < threshold) {
      return ;
    }

    Eigen::MatrixXf JacobianTranspose = computeJacobianTranspose();

    Eigen::VectorXf angleAdjustments = learningRate * JacobianTranspose * error ; //learningrate er skalar til hvor mye vi skal justere error vinkel
    updateJointAngles(angleAdjustments);
  }



};

#endif