/*
 * EkfModelBasic.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_GEKFMODEL_WNOISE_H_
#define INC_GEKFMODEL_WNOISE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ekfSysModel.h>

using namespace Eigen;

class gSys_wNoise : public EkfSysModel {
public:

	gSys_wNoise();
	gSys_wNoise(const MatrixXd & Qmod);
	~gSys_wNoise(){};

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

#endif /* INC_GEKFMODEL_SKEW_H_ */
