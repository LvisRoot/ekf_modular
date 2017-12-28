/*
 * gSys_rBias.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include "sysModels/gSys_rBias.h"
#include "SkewEigen.h"


//TODO initial bias!!!!!!!
gSys_rBias::gSys_rBias(const double wSigma, const double dBwSigma):
		EkfSysModel(MatrixXd::Identity(6,6),
					MatrixXd::Identity(6,6),
					MatrixXd::Identity(6,6)),
		_gRot(AngleAxisd::Identity()),
		_wCov(wSigma * wSigma),
		_dBwCov(dBwSigma * dBwSigma){};



void gSys_rBias::predict(VectorXd & xEst, const VectorXd & uIn, const double dt){

	Vector3d rotVect = dt * (uIn - xEst.head(3));

	if (rotVect.norm() == 0)
		_gRot = AngleAxisd::Identity();
	else
		_gRot = AngleAxisd(rotVect.norm(), rotVect / rotVect.norm());

	xEst.tail(3) = _gRot.matrix() * xEst.tail(3);

	_Amod.block<3,3>(3,0) = dt * EIGEN_SKEW(xEst.tail(3));
	_Amod.block<3,3>(3,3) = _gRot.matrix();

	_Lmod.block<3,3>(3,3) = - dt * EIGEN_SKEW(xEst.tail(3));

	_Qmod.block<3,3>(0,0) = _dBwCov * dt * dt *  MatrixXd::Identity(3,3);
	_Qmod.block<3,3>(3,3) = _wCov * MatrixXd::Identity(3,3);
}



