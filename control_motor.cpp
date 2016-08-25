/*#include <stdio.h>
#include <iostream>
#include <math.h>
#include <tgmath.h>
#include <complex> 
#include <vector>
//#include "filter_rtc.hpp"
#include "bissection_exclusion4.hpp"
#include "filter_num_dif2.hpp"
#include "ClarkeParkTransforms.cpp"
#include "SlipAngle.hpp"
#include "tpid_class.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>

#include <string>
#include <fstream>*/
#include "control_motor.hpp"
#define T  0.0001
ofstream Tr ("Tr.txt");
extern double M;//0.050382//0.0117//0.069312//mutual inductance
extern double Ls;//0.051195//0.014//0.07132


extern double Lr;//0.052571//0.014//0.07132
extern double J;//TODO:??

//!!!!!!!!!!!!!!double dy_nt(double &y1, double &y_1);//y1 means y[n+1] 
	//return dynt=(y1-y_1)/(2*per);

//!!!!!!!-tirar isto final-double d2y_nt(double &y1, double &y,double &y_1);//derived can only be obtained  when n>=? 
extern double ro;//0.999863//1-*M/(Ls*Lr)//alterar nome
extern double B;//M/(ro*Ls*Lr)
extern double roLs;//0.013998//ro*Ls
extern double MB;//0.69851//B*M
extern double one_p_MB;//1.69851//1+MrB
double Lsro=(ro*Ls);//0.00081261
double Lrro=(ro*Lr);//0.0021891

//#include <gtkmm/button.h>
using namespace std;
using namespace Maths;
/*#define fhz 60//?? 50 ??
#define Ls 0.051195
#define np 2//number pole pairs
#define sqrt3_2 0.8660254038//to do
#define Rs	0.10941
///#define p_2w0lm //w0 velocidade rotor e nao da freq da linha
//#define Rr	 
#define p_2 3//p/2
//#define K 0.5318138651// 2/3*2/p*Lr/M
#define Lr 0.052571
#define J 0.089//tudo:??
#define P 4//poles number
*/
/*
#define T  0.0001
#define Lsro (ro*Ls)//0.00081261
#define Lrro (ro*Lr)//0.0021891

#define vel_p (4.51)//?
#define vel_i 0//0.0129//?
#define vel_d 0//?
#define vel_Min_pid_res (-170)//?
#define vel_Max_pid_res 170//360//44//?
#define vel_cel 8//?

#define torque_control_p 1//0.6///1?
#define torque_control_i 0.000625
#define torque_control_d 0//?
#define torque_control_Min_pid_res (-150)//?
#define torque_control_Max_pid_res 150//?
#define torque_control_cel 3000//?

#define current_control_y_p 1//0.7//10//?
#define current_control_y_i 0.000625//0.0625//?
#define current_control_y_d 0//?
#define current_control_y_Min_pid_res (-200)//100?
#define current_control_y_Max_pid_res 200//100?
#define current_control_y_cel 3000//?

#define flux_control_p 1//0.6///1?
#define flux_control_i 0.00625
#define flux_control_d 0//?
#define flux_control_Min_pid_res (-100)//?
#define flux_control_Max_pid_res 100//?
#define flux_control_cel 3000//?

#define current_control_x_p 1//0.7//10//?
#define current_control_x_i 0.000625//0.0625//?
#define current_control_x_d 0//?
#define current_control_x_Min_pid_res (-200)//100?
#define current_control_x_Max_pid_res 200//100?
#define current_control_x_cel 3000//?
ofstream fTorque ("torque.txt");
ofstream fVel ("speed.txt");
ofstream fImr ("Imr.txt");
ofstream fIDQ ("IDQ.txt");
ofstream fwref ("wref.txt");
ofstream fload ("load.txt");
*/
//class motor{
//alteradoOOOOOOOOOOO//TODO a mais (existn n ciclos d control>>>>> nn
/*public:
long n;
protected:
	
private:
	double ids,iqs,idr,iqr;
	double B_[3][3],C[3][3];
	double D,Bs,Br,Bm;
double fds,fqs,fdr,fqr;
	double torq_L;
double theta_r;
double alfa;

*/

	double motor::torque(double Vds,double Vqs){//torque generated
		//	cout<<"vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
		ids=ids+(Vds/roLs-alfa*ids+B/TR*fdr+np*wr*B*fqr+np*wr*iqs)*T;//ids=Lr/(Lr*Ls-M*M)*fds-M/(Ls*Lr-M*M)*fdr;
		iqs=iqs+(Vqs/roLs-alfa*iqs+B/TR*fqr-np*wr*B*fdr-np*wr*ids)*T;//idr=-M/(Ls*Lr-M*M)*fds+Ls/(Ls*Lr-M*M)*fdr;
		//iqs=Lr/(Lr*Ls-M*M)*fqs-M/(Ls*Lr-M*M)*fqr;//iqr=-M/(Ls*Lr-M*M)*fqs+Ls/(Ls*Lr-M*M)*fqr;//double torq_g=(-3/2*P/2*M/(Lr*Ls-M*M)*(fds*fqr-fqs*fdr));//fds=fds+(Vds+2*PI*fhz*fqs-Rs*ids)*T;cout<<"fds:"<<fds<<endl;
		//fqs=fqs+(Vqs-2*PI*fhz*fds-Rs*iqs)*T;cout<<"fqs:"<<fqs<<endl;
		fdr=fdr+(M/TR*ids-fdr/TR)*T;//fdr=fdr+((2*PI*fhz-P/2*wr)*fqr-Rr*idr)*T;cout<<"fdr:"<<fdr<<endl;
		fqr=fqr+(M*iqs/TR-fqr/TR)*T;//fqr=fqr+(-(2*PI*fhz-P/2*wr)*fdr-Rr*iqr)*T;cout<<"fqr:"<<fqr<<endl;
		torq_g=M*np/Lr*(iqs*fdr-ids*fqr);/*cout<<"ids"<<ids<<endl<<"iqs"<<iqs<<endl;*/fTorque<<n<<" "<<torq_g<<"tttttttttttttttttttttttttttttorq_g:"/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl;//<<"torq_l:"<<torq_L<<endl;	
		return torq_g;
	};
//!!!!!!!!!!!!!errado? pois conversao em relacao estator e em relacao angulo rotorico, to do in another version the introduction of pwm: qd->abc->pwm, pwm->dq
//!!(NAO) vou introduzir voltage compensation pois isso seria se nao houvesse correccao do Tr(isto em relacao ti),mas depois introduzi o q aparece no livro
//public:
long motor::get_n(){return n;};
//double torq_g;
double motor::get_torq_g(){return torq_g;};
//double TR;
double motor::get_TR(){return TR;};

void motor::set_theta_r(double t){theta_r=t;};
//void incr_n(){nn++;};
	//int n(){return nn;};///alteradOOOOOOOOOOO
	motor::motor(){TR=/*1.12252*/0.17703/*Lr/Rr*/;Rs=/*0.258*/0.10941/*0.435*/;alfa=(Rs/roLs+M*M/(roLs*Lr*TR));n=0;ids=0;iqs=0;idr=0;iqr=0;wr=0;/*Rr=3.9;*/fds=0;fqs=0;fdr=0;fqr=0;};
	motor::~motor(){};
	double motor::torque(double vas,double vbs,double vcs){
		return torque(ClarkePark(/*2*PI*fhz*/np*theta_r,tThreePhase(vas,vbs,vcs)).d,ClarkePark(/*2*PI*fhz*/np*theta_r,tThreePhase(vas,vbs,vcs)).q);
	};
	double motor::get_wr(double va,double vb,double vc/*double torq_g,double torq_L*/){
		//??if torq_g>torq_L
		wr=wr+((torque(va,vb,vc)-torq_L)*T/J);
	return wr;
	};
	double motor::get_ias(){
		return (InvClarkePark(np*theta_r/*2*PI*fhz*n()*T*/,tTwoPhaseDQ (ids,iqs)).a);
	};
	double motor::get_ibs(){
		return (InvClarkePark(np*theta_r/*2*PI*fhz*n()*T*/,tTwoPhaseDQ (ids,iqs)).b);
	};
	double motor::get_ics(){
		return (InvClarkePark(np*theta_r/*2*PI*fhz*n()*T*/,tTwoPhaseDQ (ids,iqs)).c);
	};
	void motor::set_torq_L(double torque){torq_L=torque;};
void motor::set_TR(double r){TR=r;};
void motor::set_Rs(double rr){Rs=rr;};
	double motor::get_torq_L(){return torq_L;};
	double motor::get_wr(){return wr;};
//tTwoPhaseDQ get_Vdq(){return tTwoPhaseDQ(Vds,Vqs);};
/*	tThreePhase get_abc_voltage(){//tudo:get or find abc_voltage;when torqL>torq_g
		return tThreePhase (bi2tri_as(Vds),bi2tri_bs(Vds,Vqs),bi2tri_cs(Vds,Vqs)) ;
};*/

//double Rs;
double motor::get_Rs(){return Rs;};
//protected:
//double wr;


//};

//class control_loops: public motor{
//public:
	double control_loops::get_rot_fl_an(){return rot_fl_an;};
	//double imrref;
	//double rot_fl_an;

	//filter f;
	//Rs_Tr ConstTr;
//	long nn;//(nn*T)
	/*SlipAngle angle;
	tThreePhase abc_current;
	tThreePhase abc_voltage;
	tTwoPhaseDQ IDQ;
	tTwoPhaseDQ VDQ;
	tTwoPhase V;
*/

	//float Theta_r;
	//double w_ref;

	control_loops::control_loops():c(0),imrref(0.0),angle(0.0),abc_current(0.0,0.0,0.0),abc_voltage(0.0,0.0,0.0),IDQ(0.0001,0.0001),VDQ(0.0,0.0),rot_fl_an(0.0),Theta_r(0.0),w_ref(0.0),V(0.0,0.0),IDQ_rotor(0.0,0.0),VDQ_rotor(0.0,0.0){
			vel.tune_pid(vel_p,vel_i,vel_d);/*vector <Rs_Tr> ConstTr(100,Rs_Tr())*/;ConstTr=nullptr;
			torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);
			current_control_y.tune_pid(current_control_y_p,current_control_y_i,current_control_y_d);
			current_control_x.tune_pid(current_control_x_p,current_control_x_i,current_control_x_d);
			flux_control.tune_pid(flux_control_p,flux_control_i,flux_control_d);
			
	};
	control_loops::~control_loops(){/*delete ConstTr;*/};
	void control_loops::set_w_ref(double wreff) {w_ref=wreff;};
	void control_loops::set_imrref(double imrreff){imrref=imrreff;};
	double control_loops::get_w_ref(){return w_ref;};

	void control_loops::set_Theta_r(){
		Theta_r=Theta_r+motor::wr*T;
		while (Theta_r>=(2*PI)||Theta_r<0)//TODO 0 e 2*PI || -2*PI e 2PI {
		       {if(Theta_r>=(2*PI)){Theta_r=Theta_r-(2*PI);}
			else{Theta_r=Theta_r+(2*PI);};};
		motor::set_theta_r(Theta_r);//float Theta_r_np=Theta_r*np;while(Theta_r_np>=2*PI){Theta_r_np=Theta_r_np-2*PI;};
		//cout<<"Theta_r*np"<<(Theta_r_np)<<endl;//adapt after 2PI; electrical?
	}
	
	/*void control_loops::*/class ConstantTr/*(Rs_Tr * ConstTr_)*/{
		////-------estimacao de TR:	
				public:
					void operator()(Rs_Tr * ConstTr_,control_loops * thiss){
				//vector<Rs_Tr >::iterator it = ConstTr.end();it--;
		//	/*	if ((1.05*velo)>(motor::wr)&&(velo*0.95)<(motor::wr)){*/
				ConstTr_->Rs_Tr_do_it();
				/*se for maior ou menor que 1+-7%*/
				thiss->set_TR(ConstTr_->get_Tr());/*motor::Rs=ConstTr->get_Rs();*/cout<<"TR::::"<<ConstTr_->get_Tr()<<endl/*<<"Rs:"<<motor::Rs<<endl*/;double R=ConstTr_->get_Rs();double TT=ConstTr_->get_Tr();Tr<<"TR::::"<<TT<<"RS"<<R<<endl;
				//};
				delete ConstTr_;Tr<<"before nullptr"<<ConstTr_<<endl;ConstTr_=nullptr;Tr<<ConstTr_<<endl;
		//TODO
	};
	};
//std::thread control_loops::thread_Tr() {//TODO-retirar?
//          return std::thread([=] {control_loops::ConstantTr(); });
//};
	//get_VDQ +++++++++++++++++++++++++++
	tTwoPhase control_loops::get_V(/*const*/ ){
			if(n==0){
//PIDs----------------------
			 vel.init_pid(motor::wr,w_ref,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
				vel.calc_pid();

			 torque_control.init_pid(IDQ.q*3/2*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
				torque_control.calc_pid();

			current_control_y.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
				current_control_y.calc_pid();
			
			flux_control.init_pid(angle.get_imr(),imrref,flux_control_Min_pid_res,flux_control_Max_pid_res ,flux_control_cel);
				flux_control.calc_pid();
			current_control_x.init_pid(IDQ.d,flux_control.get_pid_result(),current_control_x_Min_pid_res,current_control_x_Max_pid_res ,current_control_x_cel);
				current_control_x.calc_pid();
cout<<"imrref_"<<imrref<<endl;
					VDQ.q=current_control_y.get_pid_result();cout<<"VDQ.q:"<<VDQ.q<<endl;//se necessario reduzir codigo podesse eliminar vdq?

// isd.init_pid(IDQ.d,(90/*-5*w_ref*w_ref*w_ref+56*w_ref*w_ref-209*w_ref+300*/)/*TODO:(30%nominal)*/,current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
//	current_control_y.calc_pid();
					VDQ.d=current_control_x.get_pid_result();cout<<"VDQ.d:"<<VDQ.d<<endl;
				//reset?integral

//+Vdecouple-------------tudo:srea necessario com a Tr corregida			
	VDQ.d=VDQ.d-angle.get_wm()*Lsro*IDQ.q;				
	VDQ.q=VDQ.q+angle.get_wm()*Lsro*IDQ.d+(Ls-Lsro)*angle.get_wm()*angle.get_imr();

			}
		else{
//----------------------------------------
			abc_current.a=/*motor::*/this->get_ias();cout<<"ia"<<abc_current.a<<endl;abc_current.b=this->get_ibs();abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor
			IDQ=ClarkePark(rot_fl_an,abc_current);//TODO???rfa==0?

			/*tTwoPhaseDQ*/IDQ_rotor=(ClarkePark((np*Theta_r),abc_current));
			/*tTwoPhaseDQ*/VDQ_rotor=(ClarkePark((np*Theta_r),abc_voltage));//TODO: voltage,done?, sensor?				
			//signal_Tr;//start_thread_Tr();//TODO??
			if(n>5000){
				if(c==0){
					velo=motor::wr; ConstTr=new Rs_Tr();//Rs_Tr f;ConstTr.push_back(f);
					Tr<<"new"<<ConstTr<<endl;
				};
				c++;
				//vector<Rs_Tr >::iterator it = ConstTr.end();it--;
				//h=ConstTr.size();h--;
				ConstTr->enter_medition(IDQ_rotor.d/*f.filtred_isx()*/,IDQ_rotor.q/*f.filtred_isy()*/,VDQ_rotor.d/*f.filtred_usx()*/,VDQ_rotor.q/*f.filtred_usy()*/,motor::wr/*f.filtred_()*/);
				//};
				if(c==100/*&&*/){	
				c=0;
				//signal_Tr;//TODO
				std::thread t1((ConstantTr()),std::move(ConstTr),std::move(this));
				t1.join();
				};
			}		;
////---------------------------	
float IDQq=IDQ.q;float IDQd=IDQ.d;fIDQ<<n<<" "<<IDQd<<" "<<IDQq<<" IDQd "<<"IDQq"<<endl/*<<"motor::TR"<<motor::TR<<endl*/<<"motor::wr"<<motor::wr;
			rot_fl_an=angle.RotFluxAng(IDQq,IDQd,T,motor::TR,(motor::wr*np));cout<<"rfa:"<<rot_fl_an<<endl;//TODO nofinal retirar T chamando-o d define

//PIDs-------------------------------------------
			vel.set_process_point(motor::wr);			
			vel.set_setpoint(w_ref);
			vel.calc_pid();

			torque_control.set_process_point(IDQ.q*3/2*np*M*M/Lr*angle.get_imr());
			torque_control.set_setpoint(vel.get_pid_result());    cout<<"wel.get_pid_result:"<<vel.get_pid_result()<<endl;
			torque_control.calc_pid();

	current_control_y.set_process_point(IDQ.q);
	current_control_y.set_setpoint(torque_control.get_pid_result());
	current_control_y.calc_pid();
	
	flux_control.set_process_point(angle.get_imr());
	flux_control.set_setpoint(imrref);
	flux_control.calc_pid();

	current_control_x.set_process_point(IDQ.d);
	current_control_x.set_setpoint(flux_control.get_pid_result());
	current_control_x.calc_pid();

VDQ.d=current_control_x.get_pid_result();   cout<<"VDQ.d:"<<VDQ.d<<endl;
VDQ.q=current_control_y.get_pid_result();   cout<<"VDQ.q:"<<VDQ.q<<endl;

//+Vdecouple-------------	
	VDQ.d=VDQ.d-angle.get_wm()*Lsro*IDQ.q;cout<<"VDQD+"<<VDQ.d<<endl;//Rs*IDQ.d-np*2/3*motor::wr*(Lsro+Lrro)*IDQ.q;				
	VDQ.q=VDQ.q+angle.get_wm()*Lsro*IDQ.d+(Ls-Lsro)*angle.get_wm()*angle.get_imr();
cout <<"VDQ.q+com"<<VDQ.q<<endl;
fImr<<n<<" "<<angle.get_imr()<<" imr:"/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;	
		//TODO:ver codigo de test PID em q referem-se ao erro e pid_set_integral
		};
		cout<<"n:"<<n<<endl;++n;
		V=InvPark((rot_fl_an),VDQ);
		abc_voltage=InvClarke(V);//TODO remove in real
		return V;
	};/*
private:
	_pid vel; 
	_pid torque_control;
	_pid current_control_y; 
	_pid flux_control;
	_pid current_control_x; 
	//double Tr;
	//sPWM
};
*/
/*

class RunButton
{
	public:
	RunButton(){};
    void on_button_clicked(){flag=true;o='r';};
};
class Quit 
{
	public:
	Quit(){};
    void on_button_clicked(){flag=true;o='q';};
};

class WRefmm
{
	public:
	WRefmm(){};
    void on_button_clicked(){flag=true;o='+';};
};

class WRefmimi
{
	public:
	WRefmimi(){};
    void on_button_clicked(){flag=true;o='-';};
};

class Loadmm
{
	public:
	Loadmm(){};
    void on_button_clicked(){flag=true;o='p';};
};

class Loadmimi
{
	public:
	Loadmimi(){};
    void on_button_clicked(){flag=true;o='k';};
};

int main(int argc, char *argv[]){
	//auto app=Gtk::Aplication::create(argc,argv,'org.gtkmm.examples.base');
	Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv, "org.gtkmm.examples.base");
//double t=0;
control_loops control;//TODO ver construtor
//control.set_w_ref(170);//colocar valor (rad/s)
double velocidade=0;
tThreePhase v(0,0,0);
bool quit=false;
float w_ref=0;
float load=0;
float imrreff=0.0;

RunButton Run_;
Quit Quit_;
WRefmm	WRefmm_;
WRefmimi WRefmimi_;
Loadmm Loadmm_;
Loadmimi Loadmimi_;

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
	
Gtk::Button* pQuit = nullptr;//new Gtk::Button("Run", true);
	builder->get_widget("Quit",pQuit);
	
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
	builder->get_widget("Tr",pTr);

Gtk::Statusbar* pMagCurrent=nullptr;
	builder->get_widget("MagCurrenr",pMagCurrent);

Gtk::Statusbar* pGeneratedTorque=nullptr;
	builder->get_widget("GeneratedTorque",pGeneratedTorque);
	
Gtk::Statusbar* pSpeedOut=nullptr;
	builder->get_widget("SpeedOut",pSpeedOut);

Gtk::Statusbar* pVa=nullptr;
	builder->get_widget("Va",pVa);
Gtk::Statusbar* pVb=nullptr;
	builder->get_widget("Vb",pVb);
Gtk::Statusbar* pVc=nullptr;
	builder->get_widget("Vc",pVc);
	
	if(pWindow)
{ 
	    //Get the GtkBuilder-instantiated Button, and connect a signal handler:
	if(pQuit)
    { 
      pButtonRun->signal_clicked().connect( sigc::mem_fun(Quit_, &Quit::on_button_clicked) );
    }			
   
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
    //code for others button
	if(pSetMagCurrent)
	{
		imrreff=atof((pSetMagCurrent->get_text()).c_str());//stof(string2float
		//o=m;flag=true;
	}
	if(pSetLoad)
	{
		load=atof((pSetLoad->get_text()).c_str());//stof(string2float
		
	}
	if(pSpeed)
	{
		w_ref=atof((pSpeed->get_text()).c_str());//stof(string2float
		
	}
 app->run(*pWindow);    
}	*/

/*
while(quit==false)//or other thread da window
{//do while
//pWindow terminava aqui
   	//colocar timer 0.0001?? isto seria com motor real, nao simulacao
		if (kbhit()*//*!=0*//*||flag==true){
		o=getchar();
		switch(o){	
			case 'r' :run=true;flag=false;break;
			case 'q' :quit=true;flag=false;break;
			case 'm' :cout<<"enter magnetizing current reference:";if(flag==true)break;cin>>imrreff;fflush(stdin);break;
			case 's' :cout<<"enter speed:";cin>>w_ref;fflush(stdin);break;
			case 'l' :cout<<"enter load:";cin>>load;fflush(stdin);break; 
			case '+' :w_ref++;flag=false;break;
			case '-' :w_ref--;flag=false;break;
			case 'k' :load--;flag=false;break;
			case 'p' :load++;flag=false;break;
//case 'return': break;//pseudocode
			default:cout<<"invalid operator, s(commanded speed),l(load);+(increment load),- (decrement load), k(decrement load),p(increment load),r(run),q(quit)"<<endl;
			//run
			//stop
			//commanded speed
			//commanded load
			//quit
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
		control.get_wr(v.a,v.b,v.c);velocidade=control.get_wr();fV
		* el<<"vel:"<<velocidade<<endl;

		}
}


//double vaf,vbf,vcf;
//motor m;
double is;
//Rs_Tr constt;
filter ft;
tTwoPhaseDQ ii(0,0),vv(0,0);
float theta_r;
//double  velocidade;
//while((m.n())<93340){
//if(m.n()==1000){m.set_TR(0.00299);m.set_Rs(2.04);};
//atencao se tetha_rede estoura
//is=2/3*sqrt(pow((m.get_ids())*1.4,2)+pow((m.get_iqs())*1.4,2));cout<<"is:::"<<is<<endl;  
//cout<<"ias"<<m.get_ias()<<endl;
//cout<<"ibs"<<m.get_ibs()<<endl;
//cout<<"ics"<<m.get_ics()<<endl;
//vaf=220*1.4/*-0.2*abs(is))*//*cos(2*PI*fhz*(m.n()*T));
//vbf=220*1.4/*-0.2*abs(is))*//*cos(2*PI*fhz*(m.n()*T)-(2*PI/3));
//vcf=220*1.4/*-0.2*abs(is))*//*cos(2*PI*fhz*(m.n()*T)+(2*PI/3));
//velocidade=m.get_wr(vaf,vbf,vcf);
//theta_r=theta_r+m.get_wr()*T;if (theta_r>=(2*PI))theta_r=theta_r-2*PI;//adapt after 2PI; electrical?
//m.set_theta_r(theta_r);
//ii=Park(np*theta_r,InvPark(/*2*PI*fhz*m.n()*T*///np*theta_r,tTwoPhaseDQ(m.get_isdq())));
//vv=ClarkePark(np*theta_r,tThreePhase(vaf,vbf,vcf));
//float IDQrD=ii.d;float IDQrQ=ii.q;float VDQrD=vv.d;float VDQrQ=vv.q;  // ft.fil_sampl(IDQrD,IDQrQ,VDQrD,VDQrQ,theta_r);
////constt.enter_medition(ft.filtred_isx(),ft.filtred_isy(),ft.filtred_usx(),ft.filtred_usy(),ft.filtred_/*wr*/());
////cout <<"f_isx:"<<ft.filtred_isx()<<endl;
//constt.Y_W_::enter_medition(IDQrD,IDQrQ,VDQrD,VDQrQ,m.get_wr());
//		if(m.n()>9000){constt.Rs_Tr_do_it();double rs=constt.get_Rs();double tr=constt.get_Tr();cout<<"Trrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr:"<<tr<<endl<<"Rssssssssssss:"<<rs<<endl;};


//cout<<"isd:"<<ii.d<<"isq:"<<ii.q<<endl<<"vsd:"<<vv.d<<"vsq"<<vv.q<<endl;
//cout<<"vellllllllllllllllllllllll:"<<velocidade<<endl<<"n"<<m.n()<<endl;
//m.incr_n();
//};



//while(velocidade<6000){
//tThreePhase tensoes(InvClarkePark(control.get_ThetaSlip(),control.get_VDQ()));
//double a,b,c;a=tensoes.a;b=tensoes.b;c=tensoes.c;
//velocidade =(control.motor::get_wr(a,b,c));
//cout <<velocidade;
//};

//while(cin>>c){
//switch(c)
//{case r:
//	while(t<stop&&(cin>>c)==false){//tudo :ver esta logica

	//wr_(torque (a,b,c),L);
//	};
//case s:cout<<"set speeed ref";cin>>w_ref;

//};

//delete pWindow;
//return 1;
//};


