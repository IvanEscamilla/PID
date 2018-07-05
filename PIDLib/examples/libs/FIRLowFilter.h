#ifndef _FIRLOWFILTER_H__
#define _FIRLOWFILTER_H__

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <math.h> 

//*********************************************************************************
// Defines
//*********************************************************************************
#ifndef PI
#define PI 3.14159265359
#endif

//*********************************************************************************
// Class
//*********************************************************************************
class FIRLowFilter
{
    public:
    	/**
		* Contructor
		* Initialize the FIR LowFilter Object, 
		* @param  	fc  	filter frecuency cutoff
		* @param  	fs  	filter sample frecuency
		* @param  	N 		the order of the filter; assume N is odd
		* @return void
		*/
        FIRLowFilter( double fc, double fs, int N); 

        /**
		* Destructor
		* destructs FIR LowFilter Object
		* @param  none
		* @return none
		*/
        ~FIRLowFilter();
        
        /**
		* Compute the H(k) output of the filter, using the convolution 
		* @param	h	H(k) output of the filter	
		* @return 	void		
		*/
        void calculate( double h[] );

		/**
		* Updates frecuency cutoff the FIR LowFilter instance
		* @param	newFc	value of the desired proportional gain
		* @return 	void
		*/
        void setFCutoff( double newFc );

        /**
		* Updates frecuency cutoff the FIR LowFilter instance
		* @param	newFs	value of the desired proportional gain
		* @return 	void
		*/
        void setSampleFreq( double newFs );

        /**
		* Updates frecuency cutoff the FIR LowFilter instance
		* @param	newN	value of the desired proportional gain
		* @return 	void
		*/
        void setOrderFilter( int newN );

        /**
		* Gets the lenght of the H array
		* @param	newN	value of the desired proportional gain
		* @return 	void
		*/
        int getOrderFilter( void );

    private:
		int 	_N;	// low filter frecuency cutoff
		double 	_fc;	// low filter frecuency cutoff
		double 	_fs;	// low filter sample frecuency

		/**
		* Calculates a normalized Sinc (sine cardinal) function, (f(πx) = sinc(x) = sin(πx)/πx)
		* @param	sinc	sinc vector that will be written with the sinc function
		* @return 	void
		*/
        void calculateSincFunction( double sinc[] );

		/**
		* returns the N-point symmetric Blackman window in the column vector w, where N is a positive integer.
		* @param	w	 	w vector that will be written with N-point symmetric Blackman window calculations
		* @return 	void
		*/
        void windowBlackman( double w[] );

};


#endif // _FIRLOWFILTER_H__