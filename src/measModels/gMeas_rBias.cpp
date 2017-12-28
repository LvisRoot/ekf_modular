/*
 * gMeas_skew.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include <measModels/gMeas_rBias.h>

gMeas_rBias::gMeas_rBias() {
	_Rmeas = Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = MatrixXd(3,6);
	_Hmeas.block<3,3>(0,0) = Matrix3d::Zero();
	_Hmeas.block<3,3>(0,3) = -Matrix3d::Identity();
}

gMeas_rBias::gMeas_rBias(const MatrixXd& Rmeas) {
	_Rmeas = Rmeas;
	_Gmeas = Matrix3d::Identity();
	_Hmeas = MatrixXd(3,6);
	_Hmeas.block<3,3>(0,0) = Matrix3d::Zero();
	_Hmeas.block<3,3>(0,3) = -Matrix3d::Identity();
}

gMeas_rBias::gMeas_rBias(const double gDev) {
	_Rmeas = gDev * gDev * Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = MatrixXd(3,6);
	_Hmeas.block<3,3>(0,0) = Matrix3d::Zero();
	_Hmeas.block<3,3>(0,3) = -Matrix3d::Identity();
}

void gMeas_rBias::update(const VectorXd& zMeas) {
}

