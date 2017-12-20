/*
 * gEkfMeas_skew.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_GEKFMEAS_SKEW_H_
#define INC_GEKFMEAS_SKEW_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <EkfMeasModel.h>

using namespace Eigen;

class gEkfMeas_skew : public EkfMeasModel{
public:

	gEkfMeas_skew();
	gEkfMeas_skew(const MatrixXd& Rmeas);
	~gEkfMeas_skew(){};

	/**
	 * Perform EKF's update phase of the estimated state with current measurement, according to the measurement model.
	 *
	 * Interface for Measurement model linearization, according to system's measurement model.
	 * @param zMeas Current measurement vector
	 */
	void update(const VectorXd & zMeas);
};

inline gEkfMeas_skew::gEkfMeas_skew() {
	_Rmeas = Matrix3d::Identity();
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

inline gEkfMeas_skew::gEkfMeas_skew(const MatrixXd& Rmeas) {
	_Rmeas = Rmeas;
	_Gmeas = Matrix3d::Identity();
	_Hmeas = - Matrix3d::Identity();
}

inline void gEkfMeas_skew::update(const VectorXd& zMeas) {
}

#endif /* INC_GEKFMEAS_SKEW_H_ */
