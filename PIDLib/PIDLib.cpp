#include <stdio.h>
#include "PIDLib.h"



double PIDController::nowMs() {
//	tps tp = clock::now();
	auto current_time = clock::now();
	auto duration_in_seconds = dsec(current_time.time_since_epoch());
	return duration_in_seconds.count();
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
PIDController::PIDController(double pGain, double iGain, double dGain, int fs = 10) {
    PIDController::setPIDGains(pGain, iGain, dGain, fs);	//Set gains
    _fs = fs;				//set sample rate, defaults 10 Hert
    _St = (double)1/_fs;				//Calculate sample time
    _lastTime = nowMs() - _St;

    _integratMax = 250;	
    _integratMin = 0;

    _outMax = 255;
    _outMin = 0;

    _cotrolSignal = 0;		//Reset variables
    _feedbackSignal = 0;
	_hasFilter = false;
}


/*Constructor (...)*********************************************************
 *    The parametzxers specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIDController::PIDController(double pGain, double iGain, double dGain, int fs = 10, int lffCutoff = 100, AbstractPlantModel* model) {

	//Sample freq minimum 10 Hertz
	if( fs < 10 ) fs = 10;

    PIDController::setPIDGains(pGain, iGain, dGain, fs);	//Set gains
    _fs = fs;					//set sample rate, defaults 10 Hert
    _St = (double)1/_fs;				//Calculate sample time
    _lastTime = nowMs() - _St;

    _integratMax = 1000;	
    _integratMin = 0;
    
    _outMax = 255;
    _outMin = 0;

    _cotrolSignal = 0;		//Reset variables
    _feedbackSignal = 0;
	_hasFilter = true;
    //Creates a FIR Low Filter 40th Order by default
    // TODO: need user interface that allow to specify the filter order and recualculation of the sync response
    _hLowFilter = new double[40];
    _lf = new FIRLowFilter(lffCutoff, _fs, 40);
	_lf->calculate(_hLowFilter);
	_model = model;
}

PIDController::~PIDController(){
	delete _hLowFilter;
	delete _lf;
} 

bool PIDController::updatePID( double error, double desiredState, double* controlSignal, double* feedbackSignal){

	double pTerm, dTerm, iTerm, eFiltered;
	double now = nowMs();
 	double timeChange = (now - _lastTime);

   	if(timeChange >= _St) {

   		//Calculate P term
		pTerm = _pGain * error; // calculate the proportional term

		//Calculate I term
		_iState += error;		// calculate the integral state with appropriate limiting
		// Limit the integrator state if necessary, Avoid integral windup by constraining integral term to its limits

		if ( _iState > _integratMax) {
			_iState = _integratMax;
		} else if ( _iState < _integratMin) {
			_iState = _integratMin;
		}
		
		iTerm = _iGain * _iState;
		
		// calculate D term
		if(_hasFilter){
			//Filter the error signal
			eFiltered = conv( desiredState - _dState , _hLowFilter, _lf->getOrderFilter());
			dTerm = _dGain * eFiltered;
		} else {
			// calculate D term
			dTerm = _dGain * ( desiredState - _dState );
		}
		
		_dState = desiredState;

		_cotrolSignal = pTerm + iTerm + dTerm;

		if(_cotrolSignal > _outMax) {
			_cotrolSignal = _outMax;
		} else if(_cotrolSignal < _outMin) {
			_cotrolSignal = _outMin;
		}
		
		_lastTime = now;
		*controlSignal = _cotrolSignal;

		// calculate fbsignal
		printf("%f noise generated\n", _model->generateNoise());
		_feedbackSignal = _model->calculate(_cotrolSignal, _model->generateNoise(), (int)now);

		*feedbackSignal = _feedbackSignal;

		return true;
	}
	
	return false;

}

void PIDController::setPIDGains( double newPGain, double newIGain, double newDGain, int fs = 1) {

	//if some of the gains is less than 0 brake and keep the latest gains
	if ( newPGain < 0 || newIGain < 0 || newDGain < 0) return;

	double sampleTime = (double)1/fs;
	printf("Ganancias Kp: %f Ki: %f y Kd: %f St: %f \n", newPGain, newIGain, newDGain, sampleTime);
	_pGain = newPGain;              // set proportional gain
	_iGain = newIGain; // set integral gain
	_dGain = newDGain; // set derivative gain
	printf("Ganancias Kp: %f Ki: %f y Kd: %f St: %f \n", _pGain, _iGain, _dGain, sampleTime);

}

void PIDController::setPGain( double newPGain ) {
	_pGain = newPGain;	// set proportional gain
	printf("_pGain: %f \n", _pGain);
}

void PIDController::setIGain( double newIGain ) {
	_iGain = newIGain;	// set integral gain
	printf("_iGain: %f , _St: %f \n", _iGain, _St);
}

void PIDController::setDGain( double newDGain ) {
	_dGain = newDGain;	// set derivative gain
	printf("_dGain: %f , _St: %f \n", _dGain, _St);
}

void PIDController::setFS( int newFS ) {

	if (newFS >= 10) {
	
		double newSampleTime = (double)1/newFS;  //Calculate sample time
	    //adjusts the Ki and kd gains 
	    //double ratio = (double)newSampleTime / (double) _St;

	    //_iGain *= ratio; // set integral gain
		//_dGain /= ratio; // set derivative gain

		_fs = newFS;           // set sample rate
	    _St = newSampleTime;   // set new sample time
	    _lf->setSampleFreq(newFS);
	} 
}

void PIDController::setFilterCutOffFreq( double newFc) {
	_lf->setFCutoff(newFc);
}

double PIDController::getCotrolSignal() {
	return _cotrolSignal;
}

double PIDController::getFeedbackSignal() {
	return _feedbackSignal;
}
