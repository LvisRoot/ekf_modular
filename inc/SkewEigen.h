/*
 * SkewEigen.h
 *
 *  Created on: Dec 28, 2017
 *      Author: l_vis
 */

#ifndef INC_SKEWEIGEN_H_
#define INC_SKEWEIGEN_H_

#include <eigen3/Eigen/Core>

using namespace Eigen;

#define EIGEN_SKEW(sVect) ((MatrixXd(3,3) << 0, -sVect[2], sVect[1], sVect[2], 0, -sVect[0], -sVect[1], sVect[0], 0).finished())

#endif /* INC_SKEWEIGEN_H_ */
