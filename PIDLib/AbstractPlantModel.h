//*********************************************************************************
// Abstract plant model definition
//*********************************************************************************
#include <random>

class AbstractPlantModel {

public:
    double _state = 0.0;

 	explicit
 	AbstractPlantModel(double noiseAmplitud = 0) : 
 	_noiseAmplitud(noiseAmplitud) {}

    ~AbstractPlantModel() {}  

    virtual double calculate( double input, double noise = 0, double dt = 0) = 0;

    double generateNoise() {
		// Define random generator with Gaussian distribution
        int random_number = std::rand()/((RAND_MAX + 1u)/4); // rand() return a number between ​0​ and RAND_MAX
		printf("%d rand\n", random_number );
        return random_number * _noiseAmplitud;
    }

    void updateNoiseAmplitud( double newAmplitud ){
    	_noiseAmplitud = newAmplitud;
    }
 
 private:
     double _noiseAmplitud = 0,
     		_mean   = 0,
     		_stddev = 2.1;
 };