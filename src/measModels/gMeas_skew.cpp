/*
 * gMeas_skew.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include "measModels/gMeas_skew.h"

gMeas_skew::gMeas_skew() {
	_Rmeas = Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

gMeas_skew::gMeas_skew(const MatrixXd& Rmeas) {
	_Rmeas = Rmeas;
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

void gMeas_skew::update(const VectorXd& zMeas) {
}



