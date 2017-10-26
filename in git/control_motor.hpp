#ifndef CONTROL_MOTOR_	
#define CONTROL_MOTOR_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <tgmath.h>
#include <complex> 
#include <vector>
//#include "filter_rtc.hpp"
#include "bissection_exclusion4.hpp"
#include "Rotor_Time_Const.hpp"
#include "ClarkeParkTransforms.cpp"
#include "Rot_Flux_Ang.hpp"
#include "tpid_class.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
#include <thread>
#include <string>
#include <fstream>
//#include <gtkmm.h>
//#include <gtkmm/application.h>
#include <chrono>
#include <mutex>
#include <future>
#include <atomic>
//#include <gtkmm/button.h>
using namespace std;
using namespace Maths;
/*#define fhz 60//?? 50 ??
#define Ls 0.051195
#define M 0.050382//mutual inductance
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
#define Idn 15//TODO
#define Imin 10//TODO
#define Imax 43//TODO
#define Wn (Vmax/Ls/pow((Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn)),1/2))
#define Wc (Vmax/(Imax*Ls)*pow(((ro*ro+1)/(2*ro*ro)),1/2))
#define Kt 3/2*np*M*M/Lr//TODO atencao pois M aparece como nome de 2 variaveis,repetido
#define Tm1 (Kt*Idn*pow((Imax*Imax-Idn*Idn),1/2))
#define Rm 350 //TODO
#define Llr 0.0021891 //TODO verificar
#define Vmax 300//TODO

//#define Tm2 tem q ser calculado pois e variavel. Iden para Tm3 

#define vel_p (6/* 3 *//*4.5*/)//TODO
#define vel_i 0//0.00129//?
#define vel_d 0//?
#define vel_Min_pid_res (-170)//?
#define vel_Max_pid_res 170//360//44//?
#define vel_cel 8//?

#define torque_control_p (1)//0.6///1?
#define torque_control_i (0.000625/*1.2*/)
#define torque_control_d 0//?
//#define torque_control_Min_pid_res (-150)//?
//#define torque_control_Max_pid_res 150//?
#define torque_control_cel 3000//?

#define current_control_y_p 1//TODO//0.7//10//?
#define current_control_y_i 0.000625//0.0625//?
#define current_control_y_d 0//?
#define current_control_y_Min_pid_res (-200)//100?
#define current_control_y_Max_pid_res 200//100?
#define current_control_y_cel 3000//?

//#define flux_control_p 1//0.6///1?
//#define flux_control_i 0.00625
//#define flux_control_d 0//?
//#define flux_control_Min_pid_res (-100)//?
//#define flux_control_Max_pid_res 100//?
//#define flux_control_cel 3000//?

#define current_control_x_p 1//0.7//10//?
#define current_control_x_i 0.000625//0.0625//?
#define current_control_x_d 0//?
#define current_control_x_Min_pid_res (-200)//100?
#define current_control_x_Max_pid_res 200//100?
#define current_control_x_cel 3000//?
ofstream fTorque ("torque.txt");
ofstream fVel ("speed.txt");
ofstream fImr ("Imr.txt");
ofstream fIDQ ("IDQ_q.txt");
ofstream fwref ("wref.txt");
ofstream fload ("load.txt");
ofstream IDQ_d__lma ("IDQ_d__lma.txt");
//std::thread* Tr_thread;

//bool fim_calc_tr;
//bool fim_ciclo;



class motor{
//alteradoOOOOOOOOOOO//TODO a mais (existn n ciclos d control>>>>> nn
public:
	long n;motor();~motor();
protected:
	
private:
	double ids,iqs,idr,iqr;
	double B_[3][3],C[3][3];
	double D,Bs,Br,Bm;
	double fds,fqs,fdr,fqr;
	double torq_L;
	double theta_r;
	double alfa;


	double torque(double Vds,double Vqs);
//!!!!!!!!!!!!!errado? pois conversao em relacao estator e em relacao angulo rotorico, to do in another version the introduction of pwm: qd->abc->pwm, pwm->dq
//!!(NAO) vou introduzir voltage compensation pois isso seria se nao houvesse correccao do Tr(isto em relacao ti),mas depois introduzi o q aparece no livro
public:
	long get_n();
	double torq_g;
	double get_torq_g();
	double TR;
//double Tr_calc;
	double get_m_Tr_calc_gl();

	void set_theta_r(double t);
//void incr_n(){nn++;};
	//int n(){return nn;};///alteradOOOOOOOOOOO
	double torque(double vas,double vbs,double vcs);
	double get_wr(double va,double vb,double vc/*double torq_g,double torq_L*/)
		//??if torq_g>torq_L
	;
	double get_ias();
	double get_ibs();
	double get_ics();
	void set_torq_L(double torque);

	void set_Rs(double rr);
	double get_torq_L();
	double get_wr();
//tTwoPhaseDQ get_Vdq(){return tTwoPhaseDQ(Vds,Vqs);};
/*	tThreePhase get_abc_voltage(){//tudo:get or find abc_voltage;when torqL>torq_g
		return tThreePhase (bi2tri_as(Vds),bi2tri_bs(Vds,Vqs),bi2tri_cs(Vds,Vqs)) ;
};*/
	double Rs;
	double get_Rs();
protected:
	double wr;


};

class control_loops: public motor{
public:
	
	double IDQ_rotor_d();
	double IDQ_rotor_q();
	double VDQ_rotor_d();
	double VDQ_rotor_q();
	
	double get_rot_fl_an();
	double imrref;
	double rot_fl_an;
	long velo;

	//filter f;
	//Rs_Tr ConstTr;
//	long nn;//(nn*T)
	SlipAngle angle;
	tThreePhase abc_current;
	tThreePhase abc_voltage;
	tTwoPhaseDQ IDQ;
	tTwoPhaseDQ VDQ;
	tTwoPhase V;
	tTwoPhaseDQ IDQ_rotor;
	tTwoPhaseDQ VDQ_rotor;
	///std::thread thread_Tr();
	//Glib::Dispatcher signal_Tr;	//TODO -retirar?
	float Theta_r;
	double w_ref;
	
	double get_w_ref();
	control_loops()/*:imrref(0.0),angle(0.0),abc_current(0.0,0.0,0.0),abc_voltage(0.0,0.0,0.0),IDQ(0.0001,0.0001),VDQ(0.0,0.0),rot_fl_an(0.0),Theta_r(0.0),w_ref(0.0),V(0.0,0.0)*/;
	~control_loops();
	void set_w_ref(double wreff) ;
	void set_imrref(double imrreff);
	

	void set_Theta_r();
	//get_VDQ +++++++++++++++++++++++++++
	tTwoPhase get_V(/*const*/ );
//void ConstantTr(Rs_Tr * ConstTr_);
/*static */thread * t1;
private:
//int h;
//vector <Rs_Tr> ConstTr;


//std::atomic<bool> fim_ciclo;

	_pid vel; 
	_pid torque_control;
	_pid current_control_y; 
	_pid flux_control;
	_pid current_control_x; 
	//double Tr;
	//sPWM
	double IDQ_d_1,IDQ_d_,IDQ_d1,IDQ_d_p,IDQ_d_pp;
	double IDQ_q_1,IDQ_q_,IDQ_q1,IDQ_q_p,IDQ_q_pp;
	double wmr_,wmr_1,wmr_p;
};
#endif
