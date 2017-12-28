/*
 * Ekfilter.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: l_vis
 */
#include "Ekfilter.h"

Ekfilter::Ekfilter(EkfSysModel* model, EkfMeasModel* meas,
		const MatrixXd& Qmod, const MatrixXd& Rmeas){

	setModel(model);
	setMeas(meas);

	// Model matrices
	_model->setQmod(Qmod); // Model Covariance matrix assignation
	// Measurement matrices
	_meas->setRmeas(Rmeas); // Measurements Covariance matrix assignation

	// Initialize state and covariance
	_xEst = VectorXd::Zero(_model->Qmod().rows());
	_Pest = _INITIAL_POST_COV * MatrixXd::Identity(_model->Qmod().rows(), _model->Qmod().cols());
}

void Ekfilter::predict(
		const VectorXd & uIn, const double dt) {

	_model->predict(_xEst, uIn, dt);

	_Pest = _model->Amod() * _Pest * _model->Amod().transpose() +
			_model->Lmod() * _model->Qmod() * _model->Lmod().transpose();
}

void Ekfilter::update(const VectorXd & zMeas) {

	_meas->update(zMeas);

	// Compute inovation covariance & pre-multiplier for Kalman gain
	_Pgain = _Pest * _meas->Hmeas().transpose();
	_Sgain = (_meas->Hmeas() * _Pest * _meas->Hmeas().transpose() + _meas->Rmeas());

	// General x estimation
	_xEst = _xEst + _Pgain * _Sgain.llt().solve(zMeas - _meas->Hmeas() * _xEst);

	// General P estimation
	_Pest = (MatrixXd::Identity(_xEst.rows(),_xEst.rows()) - _Pgain * _Sgain.llt().solve(_meas->Hmeas())) * _Pest;

}
