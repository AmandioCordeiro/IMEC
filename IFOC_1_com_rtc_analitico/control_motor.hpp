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
//#include <thread>
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
extern tTwoPhase v_;
extern double velocidade;
extern double VDQ_alpha_esc,VDQ_beta_esc;
extern double T;
extern double Tr_calc_gl;
extern double m_Tr_calc_gl;
#define Idn 129.0//155.0//160.0//150.0//22kw:17.0//15.0//30//TODO
#define Idmin (73.0*1.1)//80//67.0//80.0//66.0//22kw:12.0//TODO
#define Imax 400.0//meus igbt so sao de 300, 400 é o valor de parametros do relatorio 1pv5135//22kw:200//95//43//TODO
/*#define*/double Wn = (Vmax/(Ls*pow((Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn)),1/2)));
/*#define*/double Wc = (Vmax/(Imax*Ls)*pow(((ro*ro+1)/(2*ro*ro)),1/2));
/*#define*/double Kt=(3.0/2.0*np*M*M/Lr);//TODO atencao pois M aparece como nome de 2 variaveis,repetido
#define Tm1 (Kt*Idn*sqrt(Imax*Imax-Idn*Idn))
/*#define*/double Rm=650.0; //TODO, nao funcionou c define nao sei pk, talvez dpois d inserir .0 ja deia, como esta agora
/*#define*/double Llr=(Lr-M);//0.0021891 //TODO verificar Llr=Lr-M;
//#define Vmax (300/pow(3,1/2))//TODO
//extern float VDC;
//#define Tm2 tem q ser calculado pois e variavel. Iden para Tm3 
extern double IDC;
extern double vaa,vbb,vcc;
/*#define*/extern double vel_p;//experimentado com 6//1000//1//9//1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO// 13marco
/*#define*/extern double vel_i;//     ''       ''  0.0006// (0.00006)//(0.00006)//0
#define vel_d 0//?
//#define vel_Min_pid_res -900.0//600 //-300.0//-170.0				//Min value PI out
//#define vel_Max_pid_res 900.0//600 //300//170.0//360//44//?		//Max value PI out
#define vel_cel 0.04//0.004//8//24//50//(3*4)//(4)//?8					//max aceleration at setpoint

/*#define*/extern double torque_control_p;// /*0.000001*/10//3.6//3.6//not added (*2) after introduced the currrent controller/*(4.0*0.7)*///6.0//0.2//1.0//0.6//6//3//(1.0*0.4)//(40*0.19)//0.6///1?
/*#define*/extern double torque_control_i;// /*0.05*/0.008//0.004//(0.0006*1.2)//0.0006//6.0//0.0006//7.0//0.0000625//(0.000625*0.7/*1.2*/)
#define torque_control_d 0.0//?
#define torque_control_Min_pid_res (-400.0)//? max current
#define torque_control_Max_pid_res 400.0//?
#define torque_control_cel 400.0//(300)//?3000

#define current_control_y_p 3//TODO//0.7//10//?//TODO!!!- i tried implement this controller without sucess 
#define current_control_y_i 0.5//(0.00625)//0.0625//?
#define current_control_y_d 0.0//?
//float current_control_y_Min_pid_res = -2/3*VDC;//100?
//float current_control_y_Max_pid_res= 2/3*VDC;// 400//100?
#define current_control_y_cel	300.0// 3000//?

//#define flux_control_p 1//0.6///1?
//#define flux_control_i 0.00625
//#define flux_control_d 0//?
//#define flux_control_Min_pid_res (-100)//?
//#define flux_control_Max_pid_res 100//?
//#define flux_control_cel 3000//?

#define current_control_x_p /*0.000001*/1.9//2//(0.4)//1//11.0//(6.23*1.11)//0.4//1//0.3//100//5.0//110//5//10//20.0//10//(7)//0.7//10//? 13marco
#define current_control_x_i /*0.2*/0.0006//0.00006//(0.0006*0.2)//(0.098)//4.5//0.006//7.0//(0.00006)//0.000625//?
#define current_control_x_d 0.0//?
//float current_control_x_Min_pid_res = -2/3*VDC;// (-400)//100?
//float current_control_x_Max_pid_res = 2/3*VDC;//400//100?
#define current_control_x_cel	0.01//0.002//.001//?3000

ofstream fTorque ("torque.txt");
ofstream fVel ("speed.txt");
ofstream fImr ("Imr.txt");
ofstream fIDQ ("IDQ_q__e_Vs.txt");
ofstream fwref ("wref.txt");
ofstream fload ("load.txt");
ofstream IDQ_d__lma ("IDQ_d__lma.txt");


class motor{
public:
	long n;motor();~motor();
protected:
	tThreePhase abc_current;
	RotFluxAng angle;//TODO subst SlipAngle por RotFluxAngle
	double RotorFluxAngle;	
private:
	double ids,iqs,idr,iqr;
	double B_[3][3],C[3][3];
	double D,Bs,Br,Bm;
	double fds,fqs,fdr,fqr,idf,iqf;
	double torq_L;
	double theta_r;
	double alfa;
	double Idm,Iqm,d_Idm,d_Iqm;
	
	double torque_z(double Vds,double Vqs,double angle_get_wm);
	double torque_g(double Vds,double Vqs,double angle_get_wm);

public:
double Rr;
	long get_n();
	double torq_g;
	double get_torq_g();
	double TR;
	double get_m_Tr_calc_gl();

	void set_theta_r(double t);
	double torque(double vas,double vbs,double vcs);
	double get_wr(double va,double vb,double vc);
	double get_ias();
	double get_ibs();
	double get_ics();
	void set_torq_L(double torque);

	void set_Rs(double rr);
	double get_torq_L();
	double get_wr();
//tTwoPhaseDQ get_Vdq(){return tTwoPhaseDQ(Vds,Vqs);};

	double Rs;
	double get_Rs();
protected:
	double wr;


};

class control_loops: public motor{
public:
double vec_MediaTr_[9];
	
	
	void get_VDC(float vdc);
	double IDQ_rotor_d();
	double IDQ_rotor_q();
	double VDQ_rotor_d();
	double VDQ_rotor_q();
	
	double get_rot_fl_an();
	double imrref;
	//double rot_fl_an;
	long velo;

	//filter f;
	//Rs_Tr ConstTr;
	
	
	tThreePhase abc_voltage;
	tThreePhase abc_voltage_svpwm;
	tTwoPhaseDQ IDQ;
	tTwoPhaseDQ VDQ;
	tTwoPhaseDQ VDQ_rfa;
	tTwoPhase V/*,v_*/;
	tTwoPhaseDQ IDQ_rotor;
	tTwoPhaseDQ VDQ_rotor;
	float Theta_r;
	double w_ref;
	
	double get_w_ref();
	control_loops()/*:imrref(0.0),angle(0.0),abc_current(0.0,0.0,0.0),abc_voltage(0.0,0.0,0.0),IDQ(0.0001,0.0001),VDQ(0.0,0.0),rot_fl_an(0.0),Theta_r(0.0),w_ref(0.0),V(0.0,0.0)*/;
	~control_loops();
	void set_w_ref(double wreff) ;
	void set_imrref(double imrreff);
	

	void set_Theta_r();
	void vel_tune_pid();//TODO remove at end
	void torque_control_tune_pid();//TODO remove at end
	//get_VDQ +++++++++++++++++++++++++++
	tTwoPhase get_V(/*const*/ );
	
//void ConstantTr(Rs_Tr * ConstTr_);
/*static *///thread * t1;
private:
//std::atomic<bool> fim_ciclo;

	_pid vel; 
	_pid torque_control;
	_pid current_control_y; 
	_pid flux_control;
	_pid current_control_x;
	//_pid rtc;
	//float torque_control_Min_pid_res;//?
	//float torque_control_Max_pid_res;
	float vel_Min_pid_res;//600 //-300.0//-170.0				//Min value PI out
	float vel_Max_pid_res;
	float current_control_y_Min_pid_res;//(-(2/3)*300);//100?
	float current_control_y_Max_pid_res;//((2/3)*300);// 400//100?
	float current_control_x_Min_pid_res;//(-(2/3)*300);// (-400)//100?
	float current_control_x_Max_pid_res;//((2/3)*300);//400//100?
	 
	double IDQ_d_1,IDQ_d_,IDQ_d1,IDQ_d_p,IDQ_d_pp;
	double IDQ_q_1,IDQ_q_,IDQ_q1,IDQ_q_p,IDQ_q_pp;
	double wmr_,wmr1,wmr_p,wmr__1;
	double VDQd_p;
	double VDQq_p;
	double sobre_Tr_comp,s;
	double IDQq_p,stc_p;
	
};
#endif
