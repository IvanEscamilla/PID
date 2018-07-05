#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Chart.H>
#include <FL/Fl_Slider.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Spinner.H>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>
#include <stdio.h>

using namespace std;

enum SIGNALCOLOR
{
	PID = FL_GREEN,
	ERROR = FL_RED,
	FEEDBACK = FL_YELLOW,
	SETPOINT = FL_WHITE,
	AXIS = FL_WHITE
};

enum GRIDSCALING
{
	AUTO_SCALE,
	AUTO_SCALE_EQUAL,
	FIXED_SCALE
};

//using StartSimulationCallback = void(*)(Fl_Widget*, void*);

int _mainW{Fl::w()/2}, 
	_mainH{Fl::h()};

using Callback = void(*)(Fl_Widget*, void*);

void defaultHandle(Fl_Widget*, void*) {
	printf("there is no event handler attached to the widget!!\n");
}

struct Windows;

struct Coord {
	Coord(int x, int y) : 
		_x(x), 
		_y(y) {}
	int getX(void) { return _x; }
	int getY(void) { return _y; }

	void setX(int newX) { _x = newX; }
	void setY(int newY) { _y = newY; }
	private:
		int _x, 
			_y;
};

struct Shape {
	int _strokeWidth, 
		_strokeColor, 
		_strokeFillColor;
	
	vector<Coord> pts;

	Shape(int sW, int sC, int sFC) :
		_strokeWidth(sW), 
		_strokeColor(sC), 
		_strokeFillColor(sFC) {}

	void add(Coord p) { pts.push_back(p); }
	
	void move(int dx, int dy) {
		for(unsigned int i = 0; i < pts.size(); i++) {
			pts[i].setX(pts[i].getX() + dx);
			pts[i].setY(pts[i].getY() + dy);
		}
	}
	virtual void draw() {}
};

struct Line : Shape {
	Line(Coord x, Coord y, int strokeWidth, int strokeColor)
	: Shape(strokeWidth, strokeColor, 0) {
		add(x);
		add(y);
	}
	void draw() {
		fl_line_style(FL_SOLID, _strokeWidth);
		fl_color(_strokeColor);
		fl_line(pts[0].getX(),pts[0].getY(),pts[1].getX(),pts[1].getY());
	}
};

struct Text : Shape {

	int _font;
	string _content;
	
	Text(Coord p, int f, int sw, int sc, string str) : 
		_font(f), 
		_content(str), 
		Shape(sw, sc, 0) {
			add(p);
		}
	void draw() {
		fl_font( _font, _strokeWidth );
		fl_color( _strokeColor );
		fl_draw( _content.c_str(),pts[0].getX(), pts[0].getY());
	}
};

struct Widget {
	int _w, _h;
	string 	_label;
	Coord 	_p;
	Callback _handler;
	Windows *owner;

	Widget(Coord point, int width, int height, string str, Callback cb) :
	 	_w(width), 
	 	_h(height),
	 	_label(str),
	 	_p(point), 
	 	_handler(cb) {}

	virtual void attach(Windows&) {}
};

struct InputBox : Widget {

	Fl_Input* _inputWidget;
	
	InputBox( Coord p, int w, int h, string s, Callback cb = defaultHandle)
	: Widget( p, w, h, s, cb) {}

	void setValue(string s) { _inputWidget->value( s.c_str() ); }
	string getValue() { return string( _inputWidget->value() ); }
	
	void attach(Windows &window) {
		_inputWidget = new Fl_Input( _p.getX(), _p.getY(), _w, _h, _label.c_str());
		_inputWidget->align(FL_ALIGN_RIGHT);
		_inputWidget->color(FL_WHITE);
		_inputWidget->textfont(FL_COURIER);
		_inputWidget->textcolor(FL_BLACK);
		_inputWidget->textsize(12);
		_inputWidget->labelfont(FL_COURIER_ITALIC);
		_inputWidget->labelcolor(FL_WHITE);
		_inputWidget->labelsize(14);
		_inputWidget->callback(_handler);
    	_inputWidget->when(FL_WHEN_ENTER_KEY_ALWAYS);
		owner = &window;
	}
};

struct OutputBox : Widget {

	Fl_Text_Display* _outputBox;
	Fl_Text_Buffer* buff;
	
	OutputBox(Coord p, int w, int h, string s) 
	: Widget(p, w, h, s, 0) {}
	
	void setValue(string s) { buff->text(s.c_str()); }
	
	void attach(Windows &window) {
		_outputBox = new Fl_Text_Display( _p.getX(), _p.getY(), _w, _h, _label.c_str());
		buff = new Fl_Text_Buffer();
		_outputBox->buffer(buff);
		_outputBox->align(FL_ALIGN_RIGHT);
		_outputBox->color(FL_BLACK);
		_outputBox->textfont(FL_COURIER);
		_outputBox->textcolor(FL_WHITE);
		_outputBox->textsize(12);
		_outputBox->labelfont(FL_COURIER_ITALIC);
		_outputBox->labelcolor(FL_WHITE);
		_outputBox->labelsize(14);
		owner = &window;
	}
};

struct InputStepper : Widget {

	Fl_Spinner* _inputWidget;
	
	InputStepper( Coord p, int w, int h, string s, Callback cb = defaultHandle)
	: Widget( p, w, h, s, cb) {}

	void setValue(double s) { _inputWidget->value(s); }
	double getValue() { return double( _inputWidget->value() ); }
	
	void attach(Windows &window) {
		_inputWidget = new Fl_Spinner ( _p.getX(), _p.getY(), _w, _h, _label.c_str());
		_inputWidget->align(FL_ALIGN_RIGHT);
		_inputWidget->color(FL_WHITE);
		_inputWidget->textfont(FL_COURIER);
		_inputWidget->textcolor(FL_BLACK);
		_inputWidget->textsize(12);
		_inputWidget->labelfont(FL_COURIER_ITALIC);
		_inputWidget->labelcolor(FL_WHITE);
		_inputWidget->labelsize(14);
		_inputWidget->callback(_handler);
    	_inputWidget->when(FL_WHEN_ENTER_KEY_ALWAYS);
    	_inputWidget->step(50);
    	_inputWidget->range(0, 1000);
		owner = &window;
	}
};

struct Graph: Widget {

	Fl_Chart* _chart;
	Graph(Coord p, int w, int h, string s, Callback cb = defaultHandle)
	: Widget(p, w, h, s, cb) {}

	void addValue(double val, int color) {
		//static char val_str[20];
        //sprintf(val_str, "%.0lf", val);
		_chart->add(val, "", color);
	}

	void setNoBoxType() {
		_chart->box(FL_NO_BOX);
	}

	void attach(Windows &window) {
		_chart = new Fl_Chart( _p.getX(), _p.getY(), _w, _h, _label.c_str());
		_chart->type(FL_LINE_CHART);
   		_chart->bounds(-50, 300);
	    _chart->autosize(1);
	    _chart->maxsize(100);
	    _chart->color(FL_BLACK);
	    _chart->textcolor(FL_WHITE);
		owner = &window;
	}
    // void draw() {
    //     printf("drawing line\n");
    //     // COMPUTE NEW COORDS OF LINE
    //     static long start = time(NULL);
    //     long tick = time(NULL) - start;
    //     char secs[80]; sprintf(secs, "%02ld:%02ld", tick/60, tick%60);
    //     double pi = 3.14 - (((double)(tick % 60) / 60.0) * 6.28);
    //     int radius = h() / 2;
    //     int x1 = (int)(x() + w()/2),
    //         y1 = (int)(y() + h()/2),
    //         x2 = (int)(x1 + (sin(pi) * radius)),
    //         y2 = (int)(y1 + (cos(pi) * radius));

    //     // TELL BASE WIDGET TO DRAW ITS BACKGROUND
    //     Fl_Box::draw();

    //     // DRAW 'SECOND HAND' OVER WIDGET'S BACKGROUND
    //     fl_color(TICK_COLOR);
    //     fl_line(x1, y1, x2, y2);
    //     fl_color(CIRC_COLOR);
    //     fl_pie(x1-10, y1-10, 20, 20, 0.0, 360.0);

    //     // DRAW TIMER TEXT STRING
    //     fl_color(TICK_COLOR);
    //     fl_font(FL_HELVETICA,16);
    //     fl_draw(secs, x()+4, y()+h()-4);
    // }

    // void Timer_CB(void *userdata) {
    //     MyTimer *o = (MyTimer*)userdata;
    //     o->redraw();
    //     Fl::repeat_timeout(0.05, Timer_CB, userdata);
    // }


    // MyTimer(int X,int Y,int W,int H,const char*L=0) : Fl_Box(X,Y,W,H,L) {
    //     box(FL_FLAT_BOX);
    //     color(BG_COLOR);
    //     Fl::add_timeout(0.15, Timer_CB, (void*)this);
    // }
};

struct Button : Widget {
	Fl_Button* _buttonWidget;
	Button(Coord p, int w, int h, string s, Callback cb)
	: Widget(p, w, h, s, cb) {}
	void attach(Windows &window) {
		_buttonWidget = new Fl_Button( _p.getX(), _p.getY(), _w, _h, _label.c_str());
		_buttonWidget->callback(_handler);
		_buttonWidget->labelsize(14);
		owner = &window;
	}
	void disable() {
		_buttonWidget->deactivate();
	}
	void enable() {
		_buttonWidget->activate();
	}
};

struct Windows : Fl_Double_Window {
	
	Windows(Coord origin, int w, int h, string title):	Fl_Double_Window(origin.getX(), origin.getY(), w, h, title.c_str()) {
		color(FL_BACKGROUND_COLOR);
		show();
	}

	vector<Shape*> shapes;

	void attach(Shape& s) { shapes.push_back(&s); }
	void attach(Widget& w) {
		begin();
		w.attach(*this);
		end();
	}

	void draw() {
		Fl_Double_Window::draw();
		for(unsigned int i = 0; i < shapes.size(); i++) shapes[i]->draw();
	}
};