/*
 * gSys_rBias.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include "sysModels/gSys_rBias.h"

gSys_rBias::gSys_rBias() {

	_Qmod = Matrix3d::Identity();

	_Lmod = Matrix3d::Identity();

	_gRot = AngleAxisd(0, -Vector3d::Zero());
}

gSys_rBias::gSys_rBias(const MatrixXd& Qmod) {

	_Qmod = Qmod;

	_gRot = AngleAxisd(0, -Vector3d::Zero());
}

void gSys_rBias::predict(VectorXd & xEst, const VectorXd & uIn, const double dt){

	Vector3d rotVect = dt * (uIn + xEst.head(3));

	if (rotVect.norm() == 0)
		_gRot = AngleAxisd::Identity();
	else
		_gRot = AngleAxisd(rotVect.norm(), rotVect / rotVect.norm());

	_Amod = _gRot.toRotationMatrix();

	xEst = _gRot.matrix() * xEst;

	_Lmod  <<	0			,-xEst[2]	,xEst[1]	,
				xEst[2]		,0			,-xEst[0]	,
				-xEst[1]	,xEst[0]	,0			;

	_Lmod *= dt;
}



