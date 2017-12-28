/*
 * EkfModelBasic.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_GEKFMODEL_RBIAS_H_
#define INC_GEKFMODEL_RBIAS_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ekfSysModel.h>

using namespace Eigen;

class gSys_rBias : public EkfSysModel {
public:

	//TODO initial bias!!!!!!!
	gSys_rBias(const double wSigma, const double dBwSigma);
	~gSys_rBias(){};

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

	/// Gyroscope's noise deviation
	double _wCov;
	/// Gyroscope's bias derivative deviation (random walk noise)
	double _dBwCov;
};

#endif /* INC_GEKFMODEL_SKEW_H_ */
