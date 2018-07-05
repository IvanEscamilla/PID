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

//*********************************************************************************
// Defines
//*********************************************************************************

#ifndef WINDOWWIDTH
#define WINDOWWIDTH 800
#endif

#ifndef WINDOWHEIGHT
#define WINDOWHEIGHT 600
#endif

//*********************************************************************************
// namespaces
//*********************************************************************************

using namespace std;

//*********************************************************************************
// Global Variables
//*********************************************************************************

list<double> calculatedValues;	// a global instance of std::mutex to protect global variable
bool killThraeds = false; 		// a global instance of std::mutex to protect global variable
std::mutex syncMutex;			// a global instance of std::mutex to protect global variable
double setPoint = 400.0;
double dgPGain = 0.1;
double dgIGain = 0.01;
double dgDGain = 0.001;
double PIDOutput = 0;
bool run = false;
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
InputBox pGainInput(Coord(20,30),50,30,"P Gain");
InputBox iGainInput(Coord(20,70),50,30,"I Gain");
InputBox dGainInput(Coord(20,110),50,30,"D Gain");
InputBox sampleFreqInput(Coord(20,150),50,30,"Sample Frequency");
InputBox cutOffFreqInput(Coord(200,30),50,30,"Cut-Off Frequency", onCutOffFreqChangeHandle);
InputBox orderFilterInput(Coord(200,70),50,30,"Order");
InputBox csVariable(Coord(430,30),50,30,"Control Signal Value");
InputBox fbVariable(Coord(430,70),50,30,"Feedback Signal Value");
InputStepper SetPointInput(Coord(430,110),67,30,"Desired State", onDesiredStateChange);

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
	pGainInput.setValue("10");

	window.attach(iGainInput);
	iGainInput.setValue("10");
	
	window.attach(dGainInput);
	dGainInput.setValue("10");

	window.attach(sampleFreqInput);
	sampleFreqInput.setValue("10");

	//Filter inputs
	window.attach(cutOffFreqInput);
	cutOffFreqInput.setValue("10");

	window.attach(orderFilterInput);
	orderFilterInput.setValue("1024");

	window.attach(csVariable);
	csVariable.setValue("N/A");

	window.attach(fbVariable);
	fbVariable.setValue("N/A");

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

	PIDController a(dgPGain,dgIGain,dgDGain, 44000, 100);

	while (!killThraeds) {
		// the access to this function is mutually exclusive
		std::lock_guard<std::mutex> guard(syncMutex);


		if(run){
			//printf("PIDThread\n");
			PIDOutput = a.updatePID( setPoint - PIDOutput, setPoint);	
		}	
  	}

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
