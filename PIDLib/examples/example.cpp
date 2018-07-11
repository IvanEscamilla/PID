//*********************************************************************************
// Headers
//*********************************************************************************
#include "GUI.h"
#include "PIDLib.h"
#include "FIRLowFilter.h"
#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
#include <list>
#include <fstream>
#include <random>
#include <iostream>
#include <string>

//*********************************************************************************
// Defines
//*********************************************************************************

#ifndef WINDOWWIDTH
#define WINDOWWIDTH 1025
#endif

#ifndef WINDOWHEIGHT
#define WINDOWHEIGHT 768
#endif

#ifndef PI
#define PI 3.14159265359
#endif

//*********************************************************************************
// namespaces
//*********************************************************************************

using namespace std;

inline std::string date_string()
{
    time_t rawtime;
    std::time(&rawtime);
    struct tm *tinfo = std::localtime(&rawtime);
    char buffer[20];
    strftime(buffer, 20, "%F-%T", tinfo);
    return std::string(buffer);
}
//*********************************************************************************
// Global Variables
//*********************************************************************************

std::mutex syncMutex;			// a global instance of std::mutex to protect global variable
list<double> errorSignal;	    // a global instance of std::mutex to protect global variable
list<double> controlSignal;	    // a global instance of std::mutex to protect global variable
list<double> feedbackSignal;    // a global instance of std::mutex to protect global variable
list<int> timeSample;           // a global instance of std::mutex to protect global variable
bool   killThraeds = false;     // a global instance of std::mutex to protect global variable
double dgSetPoint = 255.0;
double dgPGain = 0.6;
double dgIGain = 0.3;
double dgDGain = 0.01;
double dgErrorSignal = 0;
double dgPIDControlSignal = 0;
double dgPIDFeedbackSignal = 0;
int    igSampleFreq = 250;
int    igFilterCutOff = 100;
int    igFilterOrder = 40;
bool   run = false;
std::clock_t start;
double duration;

class fOBilinearFunction: public AbstractPlantModel {
  public:
    
    double calculate( double controlSignal, double noise, double dt) {
    	// Function to implement the discretization of a continuous time first
		// order lag sys = coeff/(s+coeff) using the Tustin (Bilinear) transformation.
    	double coeff = 2;

	    double num = (1/(1+2/coeff/dt)); // numerator
	    double den = (1-2/coeff/dt)*num; // denominator
	    double temp;
	    double output;

	    temp = controlSignal - den * _state + noise;
	    output = num*(temp + _state);
	    _state = temp;

	    return output;
    };

};

fOBilinearFunction* model = new fOBilinearFunction();

PIDController pid(dgPGain,dgIGain,dgDGain, igSampleFreq, igFilterCutOff, model);

//*********************************************************************************
// Defines
//*********************************************************************************
void GUIThread();
void PIDThread();
void bindWindowElements();
void exportDataHandler(Fl_Widget*, void*);
void stopSimulationHandler(Fl_Widget*, void*);
void onCutOffFreqChangeHandle(Fl_Widget*, void*);
void getFeedbackSignalHandler(Fl_Widget*, void*);
void startSimulationHandler(Fl_Widget* btn, void*);
void getControlSignalHandler(Fl_Widget*, void*);
void onDesiredStateChange(Fl_Widget*, void*);
void onPGainChange(Fl_Widget*, void*);
void onIGainChange(Fl_Widget*, void*);
void onDGainChange(Fl_Widget*, void*);
void onSampleFreqChange(Fl_Widget*, void*);
void onFilterCutoffChange(Fl_Widget*, void*);
void onFilterOrderChange(Fl_Widget*, void*);

//*********************************************************************************
// GUI Elements
//*********************************************************************************

/*Windows ******************************************************************
 ***************************************************************************/
Windows window(Coord(0, 0), WINDOWWIDTH, WINDOWHEIGHT, "PID Simulator");

/*Shapes ******************************************************************
 ***************************************************************************/
Line divisor(Coord(0,WINDOWHEIGHT/3), Coord(WINDOWWIDTH,WINDOWHEIGHT/3), 2 , FL_WHITE);

/*Input Boxes **************************************************************
 ***************************************************************************/
InputStepper pGainInput(Coord(20,30),70,30,"P Gain",0.1, Range(0,300), onPGainChange);
InputStepper iGainInput(Coord(20,70),70,30,"I Gain",0.1, Range(0,150),onIGainChange);
InputStepper dGainInput(Coord(20,110),70,30,"D Gain",0.001, Range(0,100),onDGainChange);
InputStepper sampleFreqInput(Coord(20,150),70,30,"Sample Frequency",10, Range(0,100000), onSampleFreqChange);
InputStepper cutOffFreqInput(Coord(200,30),50,30,"Cut-Off Frequency",10, Range(0,100000), onFilterCutoffChange);
InputStepper orderFilterInput(Coord(200,70),50,30,"Order",1, Range(0,300), onFilterOrderChange);
InputBox controlSignalInput(Coord(430,30),90,30,"Control Signal Value");
InputBox feedbackSignalInput(Coord(430,70),90,30,"Feedback Signal Value");
InputStepper SetPointInput(Coord(430,110),67,30,"Desired State",50, Range(0,1500), onDesiredStateChange);

/*Labels* ******************************************************************
 ***************************************************************************/
Text PIDText(Coord(20,20),FL_HELVETICA,18,FL_BLACK,"PID Tuning");
Text FilterText(Coord(200,20),FL_HELVETICA,18,FL_BLACK,"Low Filter Tuning");
Text OutputsText(Coord(430,20),FL_HELVETICA,18,FL_BLACK,"Signals values");

/*Buttons ******************************************************************
 ***************************************************************************/
Button btnExportSimulation(Coord(590,150),140,30,"Export Simulation", exportDataHandler);
Button btnStartSimulation(Coord(300,150),130,30,"Run Simulation", startSimulationHandler);
Button btnStopSimulation(Coord(445,150),130,30,"Stop Simulation", stopSimulationHandler);
Button btnGetControllSignal(Coord(680,30),40,30,"GET", getControlSignalHandler);
Button btnGetFeedbackSignal(Coord(680,70),40,30,"GET", getFeedbackSignalHandler);

/*Buttons ******************************************************************
 ***************************************************************************/
Graph ControlSignalGraph(Coord(20,(WINDOWHEIGHT/3) + 20), WINDOWWIDTH - 40, 2*(WINDOWHEIGHT/3) - 60, "PID Signal Out");
Graph ErrorSignalGraph(Coord(20,(WINDOWHEIGHT/3) + 20), WINDOWWIDTH - 40, 2*(WINDOWHEIGHT/3) - 60, "");
Graph FeedbackSignalGraph(Coord(20,(WINDOWHEIGHT/3) + 20), WINDOWWIDTH - 40, 2*(WINDOWHEIGHT/3) - 60, "");
Graph SetPointSignalGraph(Coord(20,(WINDOWHEIGHT/3) + 20), WINDOWWIDTH - 40, 2*(WINDOWHEIGHT/3) - 60, "");

//*********************************************************************************
// Main
//*********************************************************************************

int main() {

	Fl::lock(); /* "start" the FLTK lock mechanism */

	thread guiThread(GUIThread);
    thread pidThread(PIDThread);
  	
  	int result = Fl::run();

  	killThraeds = true;
  	
  	guiThread.join();
    pidThread.join();

  	return result;
}


//*********************************************************************************
// Methods
//*********************************************************************************

void bindWindowElements() {
	window.attach(divisor);
	window.attach(pGainInput);
	pGainInput.setValue(dgPGain);

	window.attach(iGainInput);
	iGainInput.setValue(dgIGain);
	
	window.attach(dGainInput);
	dGainInput.setValue(dgDGain);

	window.attach(sampleFreqInput);
	sampleFreqInput.setValue(igSampleFreq);

	//Filter inputs
	window.attach(cutOffFreqInput);
	cutOffFreqInput.setValue(igFilterCutOff);

	//window.attach(orderFilterInput);
	//orderFilterInput.setValue(igFilterOrder);

	window.attach(controlSignalInput);
	controlSignalInput.setValue("N/A");

	window.attach(feedbackSignalInput);
	feedbackSignalInput.setValue("N/A");

	window.attach(PIDText);
	window.attach(FilterText);
	window.attach(OutputsText);

	window.attach(SetPointInput);
	SetPointInput.setValue(dgSetPoint);
	// Attach buttons
	window.attach(btnStartSimulation);
	window.attach(btnStopSimulation);
	window.attach(btnExportSimulation);
	window.attach(btnGetControllSignal);
	window.attach(btnGetFeedbackSignal);

	window.attach(ControlSignalGraph);
	window.attach(ErrorSignalGraph);
	window.attach(FeedbackSignalGraph);
	window.attach(SetPointSignalGraph);

	FeedbackSignalGraph.setNoBoxType();
	ErrorSignalGraph.setNoBoxType();
	SetPointSignalGraph.setNoBoxType();

}

//*********************************************************************************
// Thread
//*********************************************************************************

void GUIThread(){

	bindWindowElements();

	while (!killThraeds) {
		// refresh rate to 30fps
		std::this_thread::sleep_for(std::chrono::milliseconds(33));
	    // the access to this function is mutually exclusive
		std::lock_guard<std::mutex> guard(syncMutex);
	    /* compute new values for widgets */
		if(run){
			Fl::lock();      // acquire the lock
		    ControlSignalGraph.addValue(dgPIDControlSignal, SIGNALCOLOR(CONTROL));
		   	FeedbackSignalGraph.addValue(dgPIDFeedbackSignal, SIGNALCOLOR(FEEDBACK));
		    ErrorSignalGraph.addValue(dgErrorSignal, SIGNALCOLOR(ERROR));
		   	SetPointSignalGraph.addValue(dgSetPoint, SIGNALCOLOR(SETPOINT));
		    Fl::unlock();    // release the lock; allow other threads to access FLTK again
		    Fl::awake();     // use Fl::awake() to signal main thread to refresh the GUI	
		}
	    
  	}

  	printf("ready to kill GUIThread\n");


}

void PIDThread() {


	while (!killThraeds) {
		// the access to this function is mutually exclusive
		std::lock_guard<std::mutex> guard(syncMutex);


		if(run){

			dgErrorSignal = dgSetPoint - dgPIDControlSignal;
			if( pid.updatePID( dgErrorSignal, dgSetPoint, &dgPIDControlSignal, &dgPIDFeedbackSignal) ) {
		 		timeSample.push_back((int)( std::clock() - start ) / (CLOCKS_PER_SEC/1000));
		 		controlSignal.push_back(dgPIDControlSignal);
		 		feedbackSignal.push_back(dgPIDFeedbackSignal);
		 		errorSignal.push_back(dgErrorSignal);
		 	}
		}	
  	}

  	/*
  	for(std::list<double>::iterator it=calculatedValues.begin(); it != calculatedValues.end(); ++it)
  		  	printf("list: %f\n", *it);

	for(std::list<int>::iterator it=timeSample.begin(); it != timeSample.end(); ++it)
	  	printf("list time: %d\n", *it);
	*/
  	printf("ready to kill PIDThread\n");

}



//*********************************************************************************
// Event Handlers
//*********************************************************************************

void exportDataHandler(Fl_Widget*, void*) {
	printf("export simulation data\n");
	// create and open the .csv file
	ofstream outputFile;

	std::string filename = "output_" + date_string() +".csv";
    outputFile.open(filename);
    // write the file headers
    outputFile << "time(ms)" << "," << "error signal" << "," <<"control signal" << "," << "feedback signal" << std::endl;
    std::list<double>::iterator it2 = errorSignal.begin();
	std::list<double>::iterator it3 = controlSignal.begin();
	std::list<double>::iterator it4 = feedbackSignal.begin();

	for(std::list<int>::iterator it1 = timeSample.begin(); it1 != timeSample.end(); ++it1 ){
		outputFile << *it1 << "," << *it2 << "," << *it3 << "," << *it4 << std::endl;
		++it2;
		++it3;
		++it4;
	}
    // close the output file
    outputFile.close();
    
}

void stopSimulationHandler(Fl_Widget*, void*) {
	btnStartSimulation.enable();
	printf("stop Simulation\n");
	run = false;
}

void onCutOffFreqChangeHandle(Fl_Widget*, void*) {
	printf("onCutOffFreqChangeHandle\n");
	double newFCutoff = cutOffFreqInput.getValue();
	pid.setFilterCutOffFreq(newFCutoff);
}

void getFeedbackSignalHandler(Fl_Widget*, void*) {
	printf("Getting feedback signal...\n");
	feedbackSignalInput.setValue(std::to_string(pid.getFeedbackSignal()));
	//float a = -1.23456;
	//double b = -12;
	//printf("test negatives \n\n float: %f -- double: %f \n", b, sin(2*PI*300*b));

}

void startSimulationHandler(Fl_Widget* btn, void*) {
	btn->deactivate();
	printf("start Simulation\n");
	start = std::clock();
	run = true;
}

void getControlSignalHandler(Fl_Widget*, void*) {
	printf("Getting controlSignal ...\n");
	controlSignalInput.setValue(std::to_string(pid.getCotrolSignal()));
}

void onDesiredStateChange(Fl_Widget*, void*) {
	printf("desired state change\n");
	dgSetPoint = SetPointInput.getValue();
}

void onPGainChange(Fl_Widget*, void*) {
	dgPGain = pGainInput.getValue();
	printf("P Gain set to %f\n", dgPGain);
	pid.setPGain(dgPGain);
}

void onIGainChange(Fl_Widget*, void*) {
	dgIGain = iGainInput.getValue();
	printf("I Gain set to %f\n", dgIGain);
	pid.setIGain(dgIGain);
}

void onDGainChange(Fl_Widget*, void*) {
	dgDGain = dGainInput.getValue();
	printf("D Gain set to %f\n", dgDGain);
	pid.setDGain(dgDGain);	
}

void onSampleFreqChange(Fl_Widget*, void*) {
	igSampleFreq = (int)sampleFreqInput.getValue();
	printf("PID sample freq set to %d \n", igSampleFreq);
	pid.setFS(igSampleFreq);
}

void onFilterCutoffChange(Fl_Widget*, void*) {
	igFilterCutOff = (int)cutOffFreqInput.getValue();
	printf("D signal filter cutoff freq set to %d \n", igFilterCutOff);
}

void onFilterOrderChange(Fl_Widget*, void*){
	igFilterOrder = (int)orderFilterInput.getValue();
	printf("D signal filter order set to %d \n", igFilterOrder);
}
