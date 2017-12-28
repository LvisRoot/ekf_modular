/*
 * gMeas_skew.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include <measModels/gMeas.h>

gMeas::gMeas() {
	_Rmeas = Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

gMeas::gMeas(const MatrixXd& Rmeas) {
	_Rmeas = Rmeas;
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

gMeas::gMeas(const double gDev) {
	_Rmeas = gDev * gDev * Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

void gMeas::update(const VectorXd& zMeas) {
}



