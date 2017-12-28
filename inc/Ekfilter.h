/*
 * ekf.h
 *
 *  Created on: Dec 6, 2017
 *      Author: l_vis
 */

#ifndef INC_EKFILTER_H_
#define INC_EKFILTER_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ekfSysModel.h>
#include <EkfMeasModel.h>

using namespace Eigen;

/**
 * Base abstract class of a generic EKF intended to be extended by concrete implementations for each
 * application. Concrete implementations must override the interfaces modelPredict() & updateLin().
 *
 * modelPredict() must compute the state prediction #_xEst as well as #_Amod & #_Lmod by linearizing the model.
 *
 * updateLin() must compute #_Hmeas by linearizing measurement model.
 */
class Ekfilter {

public:

	/**
	 *
	 * @param Qmod Model's noise covariance matrix
	 * @param Rmeas Measurement's noise covariance matrix
	 */
	Ekfilter(EkfSysModel* model, EkfMeasModel* meas, const MatrixXd & Qmod, const MatrixXd & Rmeas);

	Ekfilter(EkfSysModel* model, EkfMeasModel* meas) :
			Ekfilter(model, meas, model->Qmod(), meas->Rmeas()){};

	virtual ~Ekfilter(){};

/**
 * Set state & state's covariance matrix priors
 * @param xPrior State prior
 * @param Pprior State's covariance matrix
 */
	void setPrior(VectorXd xPrior, MatrixXd Pprior){
		_xEst = xPrior;
		_Pest = Pprior;
	}

	/**
	 *Set estimated state
	 * @param xEst  Estimated state
	 */
	void setState(VectorXd xEst) {
			_xEst = xEst;
	}

	/**
	 * Get estimated state
	 * @return Estimated state
	 */
	VectorXd getState() {
		return _xEst;
	}

	/**
	 * Get state's covariance matrix
	 * @return state's covariance matrix
	 */
	MatrixXd getCov() {
		return _Pest;
	}

	/**
	 * Get model's noise covariance matrix
	 * @return Model's noise covariance matrix
	 */
	MatrixXd getQmod() const {
		return _model->Qmod();
	}

	/**
	 * Set model's noise covariance matrix
	 * @param Qmod model's noise covariance matrix
	 */
	void setQmod(const MatrixXd Qmod) {
		_model->setQmod(Qmod);
	}

	/**
	 * Get measurement's noise covariance matrix
	 * @return measurement's noise covariance matrix
	 */
	MatrixXd getRmeas() const {
		return _meas->Rmeas();
	}

	/**
	 * Set measurement's noise covariance matrix
	 * @param Rmeas measurement's noise covariance matrix
	 */
	void setRmeas(const MatrixXd Rmeas) {
		_meas->setRmeas(Rmeas);
	}



	/**
	 *	Perform EKF'S prediction phase with current input sample prediction, according to the states dynamic model.
	 *
	 *	The state prediction & model linearization are implemented inside concrete implementations derived from this class
	 *	in modelPredict() method.
	 * @param uIn Current input vector sample
	 * @param dt Current sample time interval
	 */
	void predict(const VectorXd & uIn, const double dt);

	/**
	 * Perform EKF's update phase of the estimated state with current measurement, according to the measurement model.
	 *
	 * Measurement model linearization is implemented inside concrete implementations derived from this class
	 * in updateLin() method.
	 * @param zMeas Current measurement vector
	 */
	void update(const VectorXd & zMeas);

	void setMeas(EkfMeasModel* meas) {
		_meas = meas;
	}

	void setModel(EkfSysModel* model) {
		_model = model;
	}

private:

	const double _INITIAL_POST_COV = 5000;

	/// Estimated state
	VectorXd _xEst;
	/// Estimated state covariance matrix
	MatrixXd _Pest;

	/// Innovation Covariance (for Kalmen gain)
	MatrixXd _Sgain;
	/// Innovation Covariance pre-multiplier(for Kalmen gain)
	MatrixXd _Pgain;


	EkfSysModel* _model;
	EkfMeasModel * _meas;
};

#endif /* INC_EKFILTER_H_ */
