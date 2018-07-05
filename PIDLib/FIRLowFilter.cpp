//*********************************************************************************
// Headers
//*********************************************************************************
#include "FIRLowFilter.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
FIRLowFilter::FIRLowFilter( double fc, double fs, int N ){
    _fs = fs;
    _fc = fc;
    //Normalizing the cutoff frequency so that pi is equal to the Nyquist angular frequency
    _fc /= fs;
    _N = N;
}
  
/*Destructors (...)*********************************************************
 ***************************************************************************/
FIRLowFilter::~FIRLowFilter() {
	
} 

/**
* Calculates a normalized Sinc (sine cardinal) function, (f(πx) = sinc(x) = sin(πx)/πx)
* @param	sinc	sinc vector that will be written with the sinc function
* @return 	void
*/
void FIRLowFilter::calculateSincFunction( double sinc[]) {

	int i;
    const double M = _N-1;
    double n, middle = M/2.0;
    // Normalized ωc so that pi is equal to the Nyquist angular frequency
    double wc = 2.0*PI*_fc;
    // Generate sinc delayed by (N-1)/2
    for (i = 0; i < _N; i++) {
        if (i == middle) {
            sinc[i] = 2.0 * _fc;
        } else {
        	n = (double)i - middle;
        	sinc[i] = sin(wc*n) / (PI*n);
        }
    }        
    
    return;
}

/**
* Compute the H(k) output of the filter, using the convolution 
* @param	h 		H(k) output of the filter	
* @return 	void
*/
void FIRLowFilter::calculate( double h[] ) {

	int i;
	double *w = new double[_N];          // window function
   	double *sinc = new double[_N];       // sinc function

	// Generate Sinc function and window the vector
	calculateSincFunction(sinc);
	windowBlackman(w);
	// Make lowpass filter
	for (i = 0; i < _N; i++) {
	        h[i] = sinc[i] * w[i];
	}

	// Delete dynamic storage
	delete []w;
	delete []sinc;
	return;
}

/**
* Gets the lenght of the H array
* @param	newN	value of the desired proportional gain
* @return 	void
*/
int FIRLowFilter::getOrderFilter( void ){
	return _N;
}


/**
* returns the N-point symmetric Blackman window in the column vector w, where N is a positive integer.
* @param	w	 	w vector that will be written with N-point symmetric Blackman window calculations
* @param	N		the order of the filter, Size
* @return 	void
*/
void FIRLowFilter::windowBlackman( double w[] ){

	int i;
    const double M = _N-1;
    
    for (i = 0; i < _N; i++) {
            w[i] = 0.42 - (0.5 * cos(2.0*PI*(double)i/M)) + (0.08*cos(4.0*PI*(double)i/M));
    }
    
    return;
}        

/**
* Updates frecuency cutoff the FIR LowFilter instance
* @param	FIRLowFilter	value of the desired proportional gain
* @return 	void
*/
void FIRLowFilter::setFCutoff( double newFc ) {
	_fc = newFc;
	_fc /= _fs;
	return;
}

/**
* Updates frecuency cutoff the FIR LowFilter instance
* @param	FIRLowFilter	value of the desired proportional gain
* @return 	void
*/
void FIRLowFilter::setSampleFreq( double newFs ) {
	_fs = newFs;
	_fc /= _fs;
	return;
}

/**
* Updates frecuency cutoff the FIR LowFilter instance
* @param	FIRLowFilter	value of the desired proportional gain
* @return 	void
*/
void FIRLowFilter::setOrderFilter( int newN ) {
	_N = newN;
	return;
}
