/*
 * main_17_03.cpp
 * 
 * Copyright 2016 amrc <amrc@400a>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <tgmath.h>
#include <complex> 
#include <vector>
//#include "filter_rtc.hpp"
//#include "bissection_exclusion4.hpp"
//#include "filter_num_dif2.hpp"
#include "ClarkeParkTransforms.cpp"
#include "Rot_Flux_Ang.hpp"
#include "tpid_class.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
#include <gtkmm.h>	
#include <gtkmm/application.h>
#include <string>
#include "control_motor.cpp"
#include <thread>
#include <mutex>
bool flag=false;
bool run=false;
char o;
extern long nn;
extern double TR;
extern double torq_g;
using namespace std;
double velocidade=0;
tThreePhase v(0,0,0);
bool quit=false;
float w_ref=0;
float load=0;
float imrreff=0.0;
control_loops control;
Gtk::Entry* pSetMagCurrent_global = nullptr;
Gtk::Entry* pSetLoad_global = nullptr;
Gtk::Entry* pSpeed_global = nullptr;
Gtk::Statusbar* pSpeedOut_global=nullptr;
Gtk::Statusbar* pTimeOut_global=nullptr;
Gtk::Statusbar* pGeneratedTorque_global=nullptr;
Gtk::Statusbar* pMagCurrent_global=nullptr;
Gtk::Statusbar* pTr_global=nullptr;
Gtk::Statusbar* pSpeed_reference_global=nullptr;
Gtk::Statusbar* pLoad_global=nullptr;	
Gtk::Statusbar* pia_global=nullptr;
//mutex mu;
		
int kbhit(void) {
    static bool initflag = false;
    static const int STDIN = 0;

    if (!initflag) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initflag = true;
    }

    int nbbytes;
    ioctl(STDIN, FIONREAD, &nbbytes);  // 0 is STDIN
    return nbbytes;
}

class menu_run{
public:
menu_run()/*lthread(0)*//*,stop(false)*/{//pWin=NULL;
}

/*~menu_run(){
		Glib::Mutex::Lock lock (mutex);
		//stop=true;
		if(lthread)
		lthread->join();
	}*/
	//pWindow->sig_done.connect(sigc::ptr_fun(start));


//	void setpointerWindow(Gtk::ApplicationWindow *p){pWin=p;};

//Gtk::ApplicationWindow* pWin;
//pWin->sig_done.connect(sig::ptr_fun(mr));
	
Glib::Dispatcher signal;

//protected:
	
/*mutable*/ //std::mutex lmutex;
//std::mutex mmutex;
//bool stop;
	
void mr(){
	//std::lock_guard<std::mutex>	lock (mmutex);
	while(quit==false){//or other thread da window
   	//colocar timer 0.0001?? isto seria com motor real, nao simulacao
		//if (stop) break;
	if (kbhit()/*!=0*/||flag==true){
		if(flag ==false)o=getchar();//TODO
		switch(o){	
			case 'r' :run=true;flag=false;break;
			case 'q' :quit=true;flag=false;break;
			case 'm' :cout<<"enter magnetizing current reference:";if(flag==true){flag=false;break;};cin>>imrreff;fflush(stdin);break;
			case 's' :cout<<"enter speed:";if(flag==true){flag=false;break;};cin>>w_ref;fflush(stdin);break;
			case 'l' :cout<<"enter load:";if(flag==true){flag=false;break;};cin>>load;fflush(stdin);break; 
			case '+' :w_ref++;flag=false;break;
			case '-' :w_ref--;flag=false;break;
			case 'k' :load--;flag=false;break;
			case 'p' :load++;flag=false;break;
//case 'return': break;//pseudocode
			default:cout<<"invalid operator, s(commanded speed),l(load);+(increment load),- (decrement load), k(decrement load),p(increment load),r(run),q(quit)"<<endl;
			
		}
		}
		else if (run==true){
		cout<<"www";
		control.set_w_ref(w_ref);
		control.set_imrref(imrreff);
		control.set_torq_L(load);
		control.set_Theta_r();
		
		v=InvClarke(control.get_V());
		float s=v.a;cout<<"fase a:"<<v.a<<"fase b:"<<v.b<<"Fase c"<<v.c<<endl;
		control.get_wr(v.a,v.b,v.c);velocidade=control.get_wr();cout<<"vel:"<<velocidade<<"get_w_ref:"<<control.get_w_ref()<<endl;

		signal();
		}
};//while
};
std::thread mThread() {
          return std::thread([=] { menu_run::mr(); });
      }
};

menu_run menu_control;
std::thread* lthread;

void start(){
		if (lthread){cout<<"já há thread"<<endl;}else
		{cout <<"varias threads"<<endl;lthread=new std::thread (/*[this]*//*{*/(menu_control.mThread())/*}*/);////atencao (()) //TODO(trocar por return da funcao)
		lthread->detach();}
		//else {lthread->detach();};
};

//Lm-definido noutro include, filter_num....hpp.., como M

class RunButton
{
	public:
	RunButton(){};
    void on_button_clicked(){flag=true;o='r';start();
		//runn();//sem sentido devido ao ciclo while
		
		};
};
/*class Quit 
{
	public:
	Quit(){};
    void on_button_clicked(){flag=true;o='q';/*start();*/
	//};
//};*/

class WRefmm{
	public:
	WRefmm(){};
    void on_button_clicked(){flag=true;o='+';start();
	
	};
};

class WRefmimi
{
	public:
	WRefmimi(){};
    void on_button_clicked(){flag=true;o='-';start();
		
	};
};

class Loadmm
{
	public:
	Loadmm(){};
    void on_button_clicked(){flag=true;o='p';start();
		
	};
};

class Loadmimi
{
	public:
	Loadmimi(){};
    void on_button_clicked(){flag=true;o='k';start();
		
	};
};
class setMagCurrentt{
		public:
		setMagCurrentt(){};
		void on_entry_changed(/*Gtk::Entry* pSetMagCurrenttt*/){imrreff=atof((pSetMagCurrent_global->get_text()).c_str());ofstream temp ("temp.txt");temp <<"imrreff:"<<(pSetMagCurrent_global->get_text())<<endl;
		start();};
};

class setLoad{
		public:
		setLoad(){};
		void on_entry_changed(){load=atof((pSetLoad_global->get_text()).c_str());};//stof(string2float

};
class Speed{
		public:
		Speed(){};
		void on_entry_changed(){w_ref=atof((pSpeed_global->get_text()).c_str());};//stof(string2float
};
class Stat_bar{
		public:
		Stat_bar(){};
		void Stat(){
			pSpeedOut_global->push(to_string(control.get_wr()));
			pGeneratedTorque_global->push(to_string(control.get_torq_g()));
			pMagCurrent_global->push(to_string(control.angle.get_imr()));
			pTr_global->push(to_string(control.get_TR()));
			pTimeOut_global->push(to_string(control.get_n()));
			//cout <<"w:::"<<w<<endl;
			pSpeed_reference_global->push(to_string(control.get_w_ref()));
			pLoad_global->push(to_string(control.get_torq_L()));
			pia_global->push(to_string(control.get_ias()));
			//start();
		};
		
};
int main(int argc, char *argv[]){
	//std::lock_guard<std::mutex>	lock (lmutex);
 
	RunButton Run_;
	//Quit Quit_;
	WRefmm	WRefmm_;
	WRefmimi WRefmimi_;
	Loadmm Loadmm_;
	Loadmimi Loadmimi_;
	setMagCurrentt setMagCurrent_o;
	setLoad setLoad_o;
	Speed Speed_o;
	Stat_bar Stat_bar_o;
	       //auto app=Gtk::Aplication::create(argc,argv,'org.gtkmm.examples.base');
	Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv, "org.gtkmm.examples.base");

//double t=0;
//TODO ver construtor

//control.set_w_ref(170);//colocar valor (rad/s)

	
	auto builder=Gtk::Builder::create();
	try{
		builder->add_from_file("final.glade");
		}
	catch(const Glib::FileError& ex)
	{
		std::cerr<<"FileError:"<<ex.what()<<std::endl;
		return 1;
	}
	catch(const Glib::MarkupError& ex)
	{
		std::cerr<<"MarkupError:"<<ex.what()<<std::endl;
		return 1;
	}
	catch (Gtk::BuilderError& ex)
	{
	std:cerr<<"BuilderError:"<<ex.what()<<std::endl;	
	}


Gtk::ApplicationWindow* pWindow=nullptr;  
	builder->get_widget("Window", pWindow);
	
Gtk::Button* pButtonRun = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("Run",pButtonRun);
	
/*Gtk::Button* pQuit = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("Quit",pQuit);
	*/
Gtk::Button* pWRefmm = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("WRefmm",pWRefmm);
	
Gtk::Button* pWRefmimi = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("WRefmimi",pWRefmimi);
	
Gtk::Button* pLoadmm = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("Loadmm",pLoadmm);
	
Gtk::Button* pLoadmimi = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("Loadmimi",pLoadmimi);
	
Gtk::Entry* pSetMagCurrent = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("SetMagCurrent",pSetMagCurrent);

Gtk::Entry* pSetLoad = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("SetLoad",pSetLoad);

Gtk::Entry* pSpeed = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("Speed",pSpeed);

Gtk::Statusbar* pTimeOut=nullptr;
	builder->get_widget("TimeOut",pTimeOut);
	
Gtk::Statusbar* pTr=nullptr;
	builder->get_widget("TR",pTr);

Gtk::Statusbar* pMagCurrent=nullptr;
	builder->get_widget("MagCurrent",pMagCurrent);

Gtk::Statusbar* pGeneratedTorque=nullptr;
	builder->get_widget("GeneratedTorq",pGeneratedTorque);
	
Gtk::Statusbar* pSpeedOut=nullptr;
	builder->get_widget("SpeedOut",pSpeedOut);

Gtk::Statusbar* pVa=nullptr;
	builder->get_widget("Va",pVa);
Gtk::Statusbar* pVb=nullptr;
	builder->get_widget("Vb",pVb);
Gtk::Statusbar* pVc=nullptr;
	builder->get_widget("Vc",pVc);	

Gtk::Statusbar* pSpeed_reference=nullptr;
	builder->get_widget("Speed_reference",pSpeed_reference);

Gtk::Statusbar* pLoad_=nullptr;
	builder->get_widget("Load_",pLoad_);
Gtk::Statusbar* pia=nullptr;
	builder->get_widget("ia",pia);

if(pWindow)

{	//lock_guard<mutex> guard (mu);
	menu_control.signal.connect(sigc::mem_fun(Stat_bar_o,&Stat_bar::Stat));

	/*if(pQuit)
    { 
      pButtonRun->signal_clicked().connect( sigc::mem_fun(Quit_, &Quit::on_button_clicked));
    }*/			
	 if(pButtonRun)
    { 
      pButtonRun->signal_clicked().connect( sigc::mem_fun(Run_, &RunButton::on_button_clicked) );
    }
     if(pWRefmm)
    { 
      pWRefmm->signal_clicked().connect( sigc::mem_fun(WRefmm_, &WRefmm::on_button_clicked) );
    }
     if(pWRefmimi)
    { 
      pWRefmimi->signal_clicked().connect( sigc::mem_fun(WRefmimi_, &WRefmimi::on_button_clicked) );
    }
     if(pLoadmm)
    { 
      pLoadmm->signal_clicked().connect( sigc::mem_fun(Loadmm_, &Loadmm::on_button_clicked) );
    }
     if(pLoadmimi)
    { 
      pLoadmimi->signal_clicked().connect( sigc::mem_fun(Loadmimi_, &Loadmimi::on_button_clicked) );
    }
   	if(pSetMagCurrent)
	{	pSetMagCurrent_global=pSetMagCurrent;
		pSetMagCurrent->signal_activate().connect(sigc::mem_fun(setMagCurrent_o,&setMagCurrentt::on_entry_changed/*(pSetMagCurrent)*/));
		//stof(string2float
	}
	if(pSetLoad){
		pSetLoad_global=pSetLoad;
		pSetLoad->signal_activate().connect(sigc::mem_fun(setLoad_o,&setLoad::on_entry_changed));
		start();
		//sig_done();
	}
	if(pSpeed){
		pSpeed_global=pSpeed;
		pSpeed->signal_activate().connect(sigc::mem_fun(Speed_o,&Speed::on_entry_changed));
		start();
		//sig_done();//stof(string2float
	}
	if(pTimeOut)
	{	pTimeOut_global=pTimeOut;
		
		start();
		//sig_done();	
	}
	if(pTr)
	{	pTr_global=pTr;
		
		start();
		//sig_done();
	}
	if(pMagCurrent)
	{	pMagCurrent_global=pMagCurrent;
		
		start();
		//sig_done();
	}
	if(pGeneratedTorque){
		pGeneratedTorque_global=pGeneratedTorque;
		
		start();
		//sig_done();
	}
	if(pSpeedOut){
		pSpeedOut_global=pSpeedOut;
		//pSpeedOut_global->push(to_string(control.get_wr()));
		start();
		//sig_done();
	};
	
	if(pSpeed_reference)
	{	pSpeed_reference_global=pSpeed_reference;
		
		//start();
		//sig_done();
	}
	
	if(pLoad_)
	{	pLoad_global=pLoad_;
		
		//start();
		//sig_done();
	}
	if(pia)
	{	pia_global=pia;
		
		//start();
		//sig_done();
	}
	app->run(*pWindow);
	
};
	//Get the GtkBuilder-instantiated Button, and connect a signal handler:
//Rs_Tr constt;
//filter ft;
   //////  
delete pWindow;
return 0;
}
