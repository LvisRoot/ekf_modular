/*
 * ekfMeasModel.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_EKFMEASMODEL_H_
#define INC_EKFMEASMODEL_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

class EkfMeasModel {
public:

	//EkfMeasModel(){};
	virtual
	~EkfMeasModel(){};

	/**
	 * Get measurements's linearized state matrix _Hmeas
	 * @return Measurements's linearized state matrix
	 */
	MatrixXd Hmeas() const {
		return _Hmeas;
	}

	/**
	 * Get Measurements's linearized noise matrix
	 * @return Measurements's linearized noise matrix _Gmeas
	 */
	MatrixXd Gmeas() const {
		return _Gmeas;
	}

	/**
	* Get Measurements's noise coverience matrix
	* @return Measurements's noise coverience matrix _Rmeas
	*/
	const MatrixXd& Rmeas() const {
		return _Rmeas;
	}

	/**
	* Set Measurements's noise coverience matrix
	* @param Rmeas Measurements's noise coverience matrix
	*/
	void setRmeas(const MatrixXd& Rmeas) {
		_Rmeas = Rmeas;
	}

	/**
	 * Perform EKF's update phase of the estimated state with current measurement, according to the measurement model.
	 *
	 * Interface for Measurement model linearization, according to system's measurement model.
	 * @param zMeas Current measurement vector
	 */
	virtual void update(const VectorXd &) = 0;

protected:

	/// Measurements's noise covariance matrix
	MatrixXd _Rmeas;
	/// Measurements's linearized state matrix
	MatrixXd _Hmeas;
	/// Measurements's linearized noise matrix
	MatrixXd _Gmeas;
};

#endif /* INC_EKFMEASMODEL_H_ */
