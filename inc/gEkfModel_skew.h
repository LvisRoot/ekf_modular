/*
 * EkfModelBasic.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_GEKFMODEL_SKEW_H_
#define INC_GEKFMODEL_SKEW_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ekfSysModel.h>

using namespace Eigen;

class gEkfModel_skew : public EkfSysModel {
public:

	gEkfModel_skew();
	gEkfModel_skew(const MatrixXd & Qmod);
	~gEkfModel_skew(){};

	/**
	 * Perform EKF'S prediction phase with current input sample prediction, according to the states dynamic model.
	 * This method takes a reference to the estimated prior state and overwrites it with the current prediction.
	 * @param xEst Prior estimated state
	 * @param uIn Current input vector sample
	 * @param dt Current sample time interval
	 */
	void predict(VectorXd & xEst, const VectorXd & uIn, const double dt);

private:

	/// Estimated gravity vector transformationn espressed as Angle Axis
	AngleAxisd _gRot;
};

inline gEkfModel_skew::gEkfModel_skew() {

	_Qmod = Matrix3d::Identity();

	_Lmod = Matrix3d::Identity();

	_gRot = AngleAxisd(0, -Vector3d::Zero());
}

inline gEkfModel_skew::gEkfModel_skew(const MatrixXd& Qmod) {

	_Qmod = Qmod;

	_gRot = AngleAxisd(0, -Vector3d::Zero());
}

inline void gEkfModel_skew::predict(VectorXd & xEst, const VectorXd & uIn, const double dt){

	Vector3d rotVect = dt * uIn;

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

#endif /* INC_GEKFMODEL_SKEW_H_ */
