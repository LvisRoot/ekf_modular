/*
 * gMeas.h
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

class gMeas : public EkfMeasModel{
public:

	gMeas();
	gMeas(const MatrixXd& Rmeas);
	gMeas(const double gDev);
	~gMeas(){};

	/**
	 * Perform EKF's update phase of the estimated state with current measurement, according to the measurement model.
	 *
	 * Interface for Measurement model linearization, according to system's measurement model.
	 * @param zMeas Current measurement vector
	 */
	void update(const VectorXd & zMeas);
};

#endif /* INC_GEKFMEAS_SKEW_H_ */
