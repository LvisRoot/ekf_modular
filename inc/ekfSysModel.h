/*
 * ekfSysModel.h
 *
 *  Created on: Dec 20, 2017
 *      Author: l_vis
 */

#ifndef INC_EKFSYSMODEL_H_
#define INC_EKFSYSMODEL_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

class EkfSysModel {
public:

	EkfSysModel(const MatrixXd Qmod, const MatrixXd Amod, const MatrixXd Lmod)
				: _Qmod(Qmod), _Amod(Amod), _Lmod(Lmod){};
	EkfSysModel(){};
	virtual
	~EkfSysModel(){};

	/**
	 * Get model's noise covariance matrix
	 * @return Model's noise covariance matrix
	 */
	MatrixXd Qmod() const {
		return _Qmod;
	}

	/**
	 * Set model's noise covariance matrix
	 * @param Qmod model's noise covariance matrix
	 */
	void setQmod(const MatrixXd Qmod) {
		_Qmod = Qmod;
	}

	/**
	 * Get linearized state matrix _Amod
	 * @return Model's linearized state matrix
	 */
	const MatrixXd& Amod() const {
		return _Amod;
	}

	/**
	 * Get linearized model's noise matrix _Lmod
	 * @return Linearized model's noise matrix
	 */
	const MatrixXd& Lmod() const {
		return _Lmod;
	}

	/**
	 * Perform EKF'S prediction phase with current input sample prediction, according to the states dynamic model.
	 * This method takes a reference to the estimated prior state and overwrites it with the current prediction.
	 * @param xEst Prior estimated state
	 * @param uIn Current input vector sample
	 * @param dt Current sample time interval
	 */
	virtual void predict(VectorXd & xEst, const VectorXd & uIn, const double dt) = 0;

protected:

	/// Model's noise covariance matrix
	MatrixXd _Qmod;
	/// Model's linearized state matrix
	MatrixXd _Amod;
	/// Model's linearized noise matrix
	MatrixXd _Lmod;
};

#endif /* INC_EKFSYSMODEL_H_ */
