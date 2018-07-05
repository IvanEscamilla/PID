#include <stdio.h>
#include "PIDLib.h"



double PIDController::nowMs() {
//	tps tp = clock::now();
	auto current_time = clock::now();
	auto duration_in_seconds = dsec(current_time.time_since_epoch());
	return duration_in_seconds.count();;
}


double PIDController::conv( double signal, double h[], int sizeOfH ) {

	double result = 0;
	
	for (int i = 0; i < sizeOfH; ++i) {
		result = result + h[i] * signal;
	}
	if(signal > 0)
		printf("%lf - %lf\n",signal, result );
	return result;
}

/*Constructor (...)*********************************************************
 *    The parametzxers specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIDController::PIDController(double pGain, double iGain, double dGain, double fs = 10) {
    PIDController::setPIDGains(pGain, iGain, dGain);	//Set gains
    _fs = fs;				//set sample rate, defaults 10 Hert
    _St = 1/_fs;				//Calculate sample time
    _lastTime = nowMs() - _St;
    _integratMax = 1500;	
    _integratMin = 0;
    _cotrolSignal = 0;		//Reset variables
    _feedbackSignal = 0;
	_hasFilter = false;
}


/*Constructor (...)*********************************************************
 *    The parametzxers specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIDController::PIDController(double pGain, double iGain, double dGain, double fs = 10, double lffCutoff = 100) {
    PIDController::setPIDGains(pGain, iGain, dGain);	//Set gains
    _fs = fs;					//set sample rate, defaults 10 Hert
    _St = 1/_fs;				//Calculate sample time
    _lastTime = nowMs() - _St;
    _integratMax = 1500;	
    _integratMin = 0;
    _cotrolSignal = 0;		//Reset variables
    _feedbackSignal = 0;
	_hasFilter = true;
    //Creates a FIR Low Filter 40th Order by default
    // TODO: need user interface that allow to specify the filter order and recualculation of the sync response
    _hLowFilter = new double[40];
    _lf = new FIRLowFilter(lffCutoff, _fs, 40);
	_lf->calculate(_hLowFilter);
}

PIDController::~PIDController(){
	delete _hLowFilter;
	delete _lf;
} 

double PIDController::updatePID( double error, double desiredState){

	double pTerm, dTerm, iTerm, eFiltered, output;
	double now = nowMs();
 	double timeChange = (now - _lastTime);

   	if(timeChange >= _St) {

   		//Calculate P term
		pTerm = _pGain * error; // calculate the proportional term

		//Calculate I term
		_iState += error;		// calculate the integral state with appropriate limiting
		// Limit the integrator state if necessary
		if ( _iState > _integratMax) {
			_iState = _integratMax;
		} else if ( _iState < _integratMin) {
			_iState = _integratMin;
		}
		
		iTerm = _iGain * _iState;
		
		// calculate D term
		if(_hasFilter){
			//Filter the error signal
			eFiltered = conv(_dState - desiredState, _hLowFilter, _lf->getOrderFilter());
			dTerm = _dGain * eFiltered;
		} else {
			// calculate D term
			dTerm = _dGain * (_dState - desiredState);
		}
		
		_dState = desiredState;

		output = pTerm + dTerm + iTerm;

		_lastTime = now;

	}
	
	return output;

}

void PIDController::setPIDGains( double newPGain, double newIGain, double newDGain) {

	//if some of the gains is less than 0 brake and keep the latest gains
	if ( newPGain < 0 || newPGain < 0 || newDGain < 0) return;

	_pGain = newPGain;	// set proportional gain
	_iGain = newIGain; 	// set integral gain
	_dGain = newDGain;	// set derivative gain
}

void PIDController::setPGain( double newPGain ) {
	_pGain = newPGain;	// set proportional gain
}

void PIDController::setIGain( double newIGain ) {
	_iGain = newIGain;	// set integral gain
}

void PIDController::setDGain( double newDGain ) {
	_dGain = newDGain;	// set derivative gain
}

void PIDController::setFS( double newFS ) {
	_fs = newFS;			//set sample rate, defaults 10 Hert
    _St = 1/_fs;				//Calculate sample time
}

double PIDController::getCotrolSignal() {
	return _cotrolSignal;
}

double PIDController::getFeedbackSignal() {
	return _feedbackSignal;
}