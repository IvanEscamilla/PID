//*********************************************************************************
// Abstract plant model definition
//*********************************************************************************
#include <random>

class AbstractPlantModel {

public:
 	explicit
 	AbstractPlantModel(double noiseAmplitud = 0) : 
 	_noiseAmplitud(noiseAmplitud) {}

    ~AbstractPlantModel() {}  

    virtual double calculate( double input, double noise = 0, double dt = 0) = 0;

    double generateNoise() {
		// Define random generator with Gaussian distribution
		std::default_random_engine generator;
		std::normal_distribution<double> dist(_mean, _stddev);
		return dist(generator) * _noiseAmplitud;
    }

    void updateNoiseAmplitud( double newAmplitud ){
    	_noiseAmplitud = newAmplitud;
    }
 
 private:
     double _noiseAmplitud,
     		_mean   = 0.0,
     		_stddev = 0.1;
 };