//*********************************************************************************
// Headers
//*********************************************************************************
#include "GUI.h"
#include "PIDLib.h"
#include "FIRLowFilter.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <list>
#include <fstream>

//*********************************************************************************
// Defines
//*********************************************************************************

#ifndef WINDOWWIDTH
#define WINDOWWIDTH 1025
#endif

#ifndef WINDOWHEIGHT
#define WINDOWHEIGHT 768
#endif


//*********************************************************************************
// namespaces
//*********************************************************************************

using namespace std;

//*********************************************************************************
// Global Variables
//*********************************************************************************

std::mutex syncMutex;			// a global instance of std::mutex to protect global variable
list<double> calculatedValues;	// a global instance of std::mutex to protect global variable
list<int> timeSample;        // a global instance of std::mutex to protect global variable
bool   killThraeds = false; 		// a global instance of std::mutex to protect global variable
double setPoint = 255.0;
double dgPGain = 0.6;
double dgIGain = 0.5;
double dgDGain = 0.158;
double PIDOutput = 0;
int    igSampleFreq = 250;
int    igFilterCutOff = 100;
int    igFilterOrder = 40;
bool   run = false;
std::clock_t start;
double duration;
PIDController pid(dgPGain,dgIGain,dgDGain, igSampleFreq, igFilterCutOff);

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
InputStepper pGainInput(Coord(20,30),70,30,"P Gain",0.1, onPGainChange);
InputStepper iGainInput(Coord(20,70),70,30,"I Gain",0.1, onIGainChange);
InputStepper dGainInput(Coord(20,110),70,30,"D Gain",0.001, onDGainChange);
InputStepper sampleFreqInput(Coord(20,150),70,30,"Sample Frequency",10, onSampleFreqChange);
InputStepper cutOffFreqInput(Coord(200,30),50,30,"Cut-Off Frequency",10, onFilterCutoffChange);
InputStepper orderFilterInput(Coord(200,70),50,30,"Order",1, onFilterOrderChange);
InputBox controlSignalInput(Coord(430,30),50,30,"Control Signal Value");
InputBox feedbackSignalInput(Coord(430,70),50,30,"Feedback Signal Value");
InputStepper SetPointInput(Coord(430,110),67,30,"Desired State",50, onDesiredStateChange);

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
Graph PIDSignalGraph(Coord(20,(WINDOWHEIGHT/3) + 20), WINDOWWIDTH - 40, 2*(WINDOWHEIGHT/3) - 60, "PID Signal Out");
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

	window.attach(orderFilterInput);
	orderFilterInput.setValue(igFilterOrder);

	window.attach(controlSignalInput);
	controlSignalInput.setValue("N/A");

	window.attach(feedbackSignalInput);
	feedbackSignalInput.setValue("N/A");

	window.attach(PIDText);
	window.attach(FilterText);
	window.attach(OutputsText);

	window.attach(SetPointInput);
	SetPointInput.setValue(setPoint);
	// Attach buttons
	window.attach(btnStartSimulation);
	window.attach(btnStopSimulation);
	window.attach(btnExportSimulation);
	window.attach(btnGetControllSignal);
	window.attach(btnGetFeedbackSignal);

	window.attach(PIDSignalGraph);
	window.attach(ErrorSignalGraph);
	ErrorSignalGraph.setNoBoxType();
	window.attach(SetPointSignalGraph);
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
		    PIDSignalGraph.addValue(PIDOutput, SIGNALCOLOR(PID));
		    //ErrorSignalGraph.addValue(val1, SIGNALCOLOR(ERROR));
		   	SetPointSignalGraph.addValue(setPoint, SIGNALCOLOR(FEEDBACK));
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
			
			//printf("PIDThread\n");
		 	if( pid.updatePID( setPoint - PIDOutput, setPoint, &PIDOutput) ) {
		 		timeSample.push_back((int)( std::clock() - start ) / (CLOCKS_PER_SEC/1000));
		 		calculatedValues.push_back(PIDOutput);
		 	}
		}	
  	}

  	for(std::list<double>::iterator it=calculatedValues.begin(); it != calculatedValues.end(); ++it)
  		  	printf("list: %f\n", *it);

	  for(std::list<int>::iterator it=timeSample.begin(); it != timeSample.end(); ++it)
	  	printf("list time: %d\n", *it);

  	printf("ready to kill PIDThread\n");

}



//*********************************************************************************
// Event Handlers
//*********************************************************************************

void exportDataHandler(Fl_Widget*, void*) {
	printf("export simulation data\n");
}

void stopSimulationHandler(Fl_Widget*, void*) {
	btnStartSimulation.enable();
	printf("stop Simulation\n");
	run = false;
}

void onCutOffFreqChangeHandle(Fl_Widget*, void*) {
	printf("onCutOffFreqChangeHandle\n");
}

void getFeedbackSignalHandler(Fl_Widget*, void*) {
	printf("get feesback signal\n");

	//float a = -1.23456;
	double b = -12;
	printf("test negatives \n\n float: %f -- double: %f \n", b, sin(2*PI*300*b));

}

void startSimulationHandler(Fl_Widget* btn, void*) {
	btn->deactivate();
	printf("start Simulation\n");
	start = std::clock();
	run = true;
}

void getControlSignalHandler(Fl_Widget*, void*) {
	printf("get control signal\n");

	printf("test FIR Low filter\n");
	double fs = 44000; // 44KHz
	double fc = 100; // 300Hz cutOff
	int N = 40;	 // 1024 # taps
	printf("low filter: \n N = %d fs = 100 Hz, \n fcutoff = 300 Hz\n\n", N);
	double *h = new double[N];

	//test arrays
	double array1[10];
	double array2[] = {1,2,3,4,5,6};

	printf(" length %ld, \n", (sizeof(array1)/sizeof(*array1)));
	printf(" length %ld, \n", (sizeof(array2)/sizeof(*array2)));

	FIRLowFilter lf(fc, fs, N);
	lf.calculate(h);

	for (int i = 0; i < N; i++) {
	        printf(" %f, \n", h[i]);
	       	PIDSignalGraph.addValue( h[i], SIGNALCOLOR(PID));
	}
}

void onDesiredStateChange(Fl_Widget*, void*) {
	printf("desired state change\n");
	setPoint = SetPointInput.getValue();
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
