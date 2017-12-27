/*
 * gMeas_rBias.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include "measModels/gMeas_rBias.h"

gMeas_rBias::gMeas_rBias() {
	_Rmeas = Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

gMeas_rBias::gMeas_rBias(const MatrixXd& Rmeas) {
	_Rmeas = Rmeas;
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

void gMeas_rBias::update(const VectorXd& zMeas) {
}



