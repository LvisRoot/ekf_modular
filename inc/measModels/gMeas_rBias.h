/*
 * gMeas_rBias.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_GMEAS_H_
#define INC_GMEAS_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <EkfMeasModel.h>

using namespace Eigen;

class gMeas_rBias : public EkfMeasModel{
public:

	gMeas_rBias();
	gMeas_rBias(const MatrixXd& Rmeas);
	gMeas_rBias(const double gDev);
	~gMeas_rBias(){};

	/**
	 * Perform EKF's update phase of the estimated state with current measurement, according to the measurement model.
	 *
	 * Interface for Measurement model linearization, according to system's measurement model.
	 * @param zMeas Current measurement vector
	 */
	void update(const VectorXd & zMeas);
};

#endif /* INC_GEKFMEAS_SKEW_H_ */
