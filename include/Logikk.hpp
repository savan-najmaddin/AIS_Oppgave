#ifndef LOGIKK_HPP
#define LOGIKK_HPP



//a. Beregning av Jacobian-transposematrisen
//b. Multiplikasjon av en vektor med Jacobian-transposen
//c. Beregning av Jacobian multiplisert med Jacobian-transposen (J * J^T)
//d. Selve løseren.

//bevegelsen til IK kjeden gjort numerisk

#include <cstddef> //for size_t
#include <cmath>



struct Joint {
  float angle;
  float length;

  Joint(float ang = 0.0f, float len = 1.0f) : angle(ang), length(len) {}
};

class kinematicChain {
public:
  size_t numJoints; //antall ledd i kjeden
  std::vector<Joint> joints; //array av ledd

  explicit kinematicChain(size_t n = 0) : numJoints(n), joints(n){}

  void addJoint(const Joint& joint) {
    joints.emplace_back(joint);
    numJoints = joints.size();
  }

  static float clampAngle (float angle) {
    const float Two_PI = 2.0f * M_PI;

    while (angle >= Two_PI) {
      angle -= Two_PI;
    }

    while(angle < Two_PI) {
      angle += Two_PI;
    }

    return angle;
  }


  [[nodiscard]] Eigen::Vector2f findEffectorPosition() const{
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
    Eigen::MatrixXf jacobianTranspose(2, numJoints); //matrixXf er dynamisk
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

      jacobianTranspose(0, i) = partialX;
      jacobianTranspose(1, i) = partialY;
    }
    return jacobianTranspose;
  }


  void updateJointAngles(const Eigen::VectorXf& angleAdjustments) {
    for (size_t i = 0; i < numJoints; ++i) {
      joints[i].angle += angleAdjustments(i);
      joints[i].angle = clampAngle(joints[i].angle);
    }
  }

  void inverseKinematics(const Eigen::Vector2f& targetPosition, float learningRate, //targetPosition skal hentes fra mouselistner
                          float threshold = 0.001f, int maxIteration = 1000) {
    for (int iteration = 0; iteration < maxIteration; ++iteration) {
      Eigen::Vector2f currentPosition = findEffectorPosition();
      Eigen::Vector2f error = targetPosition - currentPosition;
      float errorMagnitude = error.norm(); //absolutt verdien til error mellom target  og current position

      //kan skrives status til konsoll

      if (errorMagnitude < threshold) {
        break;
      }

      Eigen::MatrixXf JacobianTranspose = computeJacobianTranspose();

      Eigen::VectorXf angleAdjustments = JacobianTranspose * error * learningRate; //learningrate er skalar til hvor mye vi skal justere error vinkel
      updateJointAngles(angleAdjustments);
    }
  }

};

#endif