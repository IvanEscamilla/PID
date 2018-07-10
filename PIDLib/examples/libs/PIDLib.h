#ifndef _PIDLIB_H__
#define _PIDLIB_H__

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <chrono>
#include <ctime>
#include "FIRLowFilter.h"
#include "AbstractPlantModel.h"

//*********************************************************************************
// Class
//*********************************************************************************
class PIDController
{
    public:
    	using clock = std::chrono::system_clock;
		using dsec = std::chrono::duration<double>;
		using tps = std::chrono::time_point<clock, dsec>;
    	
    	/**
		* Contructor
		* Initialize the PID Controller without filter, 
		* @param  	pGain  	proportional gain constant POSITIVE VALUE
		* @param  	iGain  	integral gain constant POSITIVE VALUE
		* @param  	dGain  	derivative gain constant POSITIVE VALUE
		* @param  	fs  	Sample Rate in Hz, minimum value of 10 Hz.
		* @return void
		*/
        PIDController(double pGain, double iGain, double dGain, int fs); 

        /**
		* Contructor
		* Initialize the PID Controller, 
		* @param  	pGain  	proportional gain constant POSITIVE VALUE
		* @param  	iGain  	integral gain constant POSITIVE VALUE
		* @param  	dGain  	derivative gain constant POSITIVE VALUE
		* @param  	fs  	Sample Rate in Hz, minimum value of 10 Hz.
		* @param  	ffc  	Low pass filter cut-off frecuency for "D" signal.
		* @return 	void
		*/
        PIDController(double pGain, double iGain, double dGain, int fs, int lffCutoff, AbstractPlantModel* model = 0);
        
        /**
		* Destructor
		* destructs PID Controller Object
		* @param  none
		* @return none
		*/
        ~PIDController();
        
        /**
		* Compute the output of the system, should be called on interval defined by Fs
		* @param	error			error
		* @param	desiredSttate	value of the desired state
		* @return 	double			calculated feedback signal
		*/
        bool updatePID( double error, double desiredSttate, double* controlSignal, double* feedbackSignal);

		/**
		* Updates proportional gain of the system
		* @param	newPGain	value of the desired proportional gain
		* @return 	void
		*/
        void setPGain( double newPGain );

        /**
		* Updates integral gain of the system
		* @param	newIGain	value of the desired integral gain
		* @return 	void
		*/
        void setIGain( double newIGain );

        /**
		* Updates derivative gain of the system
		* @param	newDGain	value of the desired derivative gain	
		* @return 	void
		*/
        void setDGain( double newDGain );
	
		/**
		* Updates the sample frequency of the system
		* @param	newFS	value of the desired FS	
		* @return 	void
		*/
		void setFS( int newFS );

        /**
		* Updates proportinal, integral and derivative gains of the system
		* @param	newPGain    value of the desired proportional gain
		* @param	newIGain    value of the desired integral gain
		* @param	newDGain    value of the desired derivative gain
		* @param	fs          value of the desired fs
		* @return 	void
		*/
        void setPIDGains( double newPGain, double newIGain, double newDGain, int fs);

		/**
		* Get the control signal
		* @param	none
		* @return 	double
		*/
        double getCotrolSignal(void);

		/**
		* Get Feedback signal
		* @param	none
		* @return 	double
		*/
        double getFeedbackSignal(void);

		/**
		* Get the actual time in ms
		* @param	none
		* @return 	double
		*/
        double nowMs(void);
	
		/**
		* calculates the convolution of a signal
		* @param	signal
		* @param	h[]		array of 	
		* @return 	double
		*/
		double conv( double signal, double h[], int sizeOfH );

		/**
		* calculates the convolution of a signal
		* @param	signal
		* @param	h[]		array of 	
		* @return 	double
		*/
		void setFilterCutOffFreq( double newFc);


    private:

		double	_dState,           // Last position input
				_iState,           // Integrator state
				_integratMax,      // Maximum allowable integrator state
				_integratMin,      // Minimum allowable integrator state
				_outMax,           // Maximum allowable output value 
				_outMin,           // Minimum allowable output value
				_iGain,            // integral gain
				_pGain,            // proportional gain
				_dGain,            // derivative gain
				_cotrolSignal,     // control signal
				_feedbackSignal,   // feedback signal
				_lastTime,         // Last time sampled in miliseconds
				_St;               // Sample time in sec
		int 	_lffCutoff,        // derivative low filter frecuency cutoff
		        _fs;               // Sample Rate in Hert
		double*	_hLowFilter;       // low filter sinc response
		FIRLowFilter* _lf;          // low Filter instance var
		AbstractPlantModel* _model; // plant model
		bool	_hasFilter;        // filter flag

		
};


#endif // _PIDLIB_Hsssss__
