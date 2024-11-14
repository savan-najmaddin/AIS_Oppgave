#ifndef MOTIONMODEL_HPP
#define MOTIONMODEL_HPP

#include "threepp/threepp.hpp"
#include <Eigen/Dense>

using namespace threepp;

class motionModel {
public:


private:
Eigen::Matrix2d skew(double omega);

};

#endif //MOTIONMODEL_HPP



