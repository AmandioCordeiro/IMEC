#ifndef FOC_H
#define FOC_H

#include <stdio.h>//TODO remove at end
#include <iostream>//TODO remove at end
#include <math.h>
#include <tgmath.h>
//#include "filter_rtc.hpp"
#include "ClarkeParkTransforms.cpp"
#include "Rot_Flux_Ang.hpp"
#include "tpid_class.h"
#include <sys/ioctl.h>//TODO remove at end
#include <termios.h>//TODO remove at end
#include <stdbool.h>//TODO remove at end
#include <unistd.h>//TODO remove at end
//#include <thread>
#include <string>//TODO remove at end
#include <fstream>//TODO remove at end
#include <iomanip>//TODO remove at end

using namespace Maths;
using namespace std;

#define FIXED_FLOAT(x) std::fixed <<std::setprecision(4)<<(x)//TODO remove at end
#define FIXED_FLOAT_L(x) std::fixed <<std::setprecision(9)<<(x)//TODO remove at end

#define T 0.000125//8khz? TODO sample period
#define T_lag (0.0000000000000000000001)//? TODO_: time since read values until apply PWM 
//TODO_ insert values and descoment in end #define vel_p 20.0 	//?
//TODO_ insert values and descoment in end #define vel_i 0.0006 //?
#define vel_d 0
//#define vel_Min_pid_res -300.0//?		//Min value PI out
//#define vel_Max_pid_res 300.0//?		//Max value PI out
//#define ACCEL 7.7 //7.7km/h per sec
#define vel_cel 0.03//?					//max aceleration at setpoint

//TODO_ insert values and descoment in end #define torque_control_p 128//without current control_y 1.4//0.33//3.6// ?
//TODO_ insert values and descoment in end #define torque_control_i 0.000008//0.01// ?
#define torque_control_d 0.0
#define torque_control_Min_pid_res -391.0//?
#define torque_control_Max_pid_res 391.0//?
#define torque_control_cel 0.4//400.0//?

#define current_control_y_p 1.0//TODO//0.7//10//? 
#define current_control_y_i 0.0005//(0.00625)//0.0625//?
#define current_control_y_d 0.0//?
#define current_control_y_Min_pid_res (-6.45)
#define current_control_y_Max_pid_res 6.45
#define current_control_y_cel 1.0// 3000//?


//TODO_ insert values and descoment in end #define current_control_x_p 0.54//1.9//? 
//TODO_ insert values and descoment in end #define current_control_x_i 0.00001//0.001//?
#define current_control_x_d 0.0
#define current_control_x_Min_pid_res -100.0// ? 
#define current_control_x_Max_pid_res 100.0//?
#define current_control_x_cel 0.05//0.04//?

#define np 2.0//? number pair poles
#define M 0.001500//? mutual inductance
#define Ls (0.000140+M)//? Lls+M
#define Llr 0.000140//?
#define Lr (Llr+M)
#define Idn 103//99.0//101.0//?
#define Idmin (71.0)
#define Tr 0.252308//? (Lr/Rr)
#define Rs 0.012//?
#define Rm 650.0//? represent eddy currents. TODO_ don't know this value
#define ro (1.0-(M*M)/(Ls*Lr))
#define Imax 400.0//? max igbt current

#define KT (3.0/2.0*np*M*M/Lr)
#define TM1 (KT*Idn*sqrt(Imax*Imax-Idn*Idn))
#define LOAD_1_sec 13//? to limit initial current setpoint torque to this
//speed up code
#define T_LOAD_1_sec 0.135/T
#define KT_t_cag 0.9*KT

#define VDC_BAT 300.0
extern float vel_p, vel_i, torque_control_p, torque_control_i, current_control_x_p, current_control_x_i;

extern long n;
float dy_nt_(float /*&*/y1, float /*&*/y_1);
float d2y_nt_(float /*&*/y1, float /*&*/y,float /*&*/y_1);

extern float max_mod_v_ref;
extern float ang;
extern float ang_u;

void calc_max_mod_v_ref(tTwoPhase v_bi);

extern float Wm;	
extern float Rd_we;
extern float Rq_we;
extern float IDQ_d_lma;// rotor magnetization current
			
//extern float Wn;
//extern float Wc;	
extern float Tm;
extern float Tm2;
extern float Tm3;
extern float iqmax;
extern float error;
extern int sinal_;

extern float IDC;//TODO: remove at end? if needed. to calc medium value of IDC 
extern float vaa,vbb,vcc;
//extern RotFluxAng angle;
extern float ids,iqs,VDC;
extern float Vmax;
extern float vel_p,vel_i,torque_control_p,torque_control_i,current_control_x_p,current_control_x_i;

extern float Lsro;//TODO make #define
//#define TR (Lr/Rr)//TODO remove at end
extern float RotorFluxAngle;
extern tThreePhase abc_current;

extern ofstream Log ;//TODO remove at end
extern ofstream fTorque ;//TODO remove at end
extern ofstream fVel ;//TODO remove at end
extern ofstream fload ;//TODO remove at end
extern ofstream T1T2 ;//TODO remove at end
extern ofstream fwref;//TODO remove at end
extern ofstream fIDQ;//TODO remove at end
extern ofstream fImr;//TODO remove at end
extern ofstream IDQ_d__lma;//TODO remove at end
class FOC
{
   public:
      /*static void ParkClarke(s32fp il1, s32fp il2, uint16_t angle);
      static void InvParkClarke(s32fp id, s32fp iq, uint16_t angle);
      static void SetDirection(int dir);
      static s32fp id;
      static s32fp iq;
      
      static uint32_t DutyCycles[3];
	  */
	  FOC();
	  ~FOC();
	  float get_VDC();
	  void vel_tune_pid();//TODO_ can be removed at end after find good values 
		void torque_control_tune_pid();//TODO_ can be removed at end after find good values
	
	  void calc_max_mod_v_ref(tTwoPhase v_bi);
	  void GetDutyCycles(float il1, float il2, /*float VDC,*/ float w_ref/*commanded rotor speed*/, float wr_/*rotor speed*/);
	  float il3/*, IDC*/;
	  int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2
	  float T0,T1,T2;
	  float pwm_a,pwm_b,pwm_c;// PWM
	  float Wn,Wc;
	  _pid vel; 
	  _pid torque_control;
	  _pid current_control_x;
	  _pid current_control_y;
	  float vel_Min_pid_res;
	  float vel_Max_pid_res;
  	  tTwoPhaseDQ IDQ, VDQ;
  	  float IDQ_d1,IDQ_d_1,IDQ_d_,IDQ_d_p,IDQ_d_pp;
	  float IDQ_q1,IDQ_q_1,IDQ_q_,IDQ_q_p,IDQ_q_pp;
	  float wmr_,wmr1,wmr_p,wmr__1;
	  tTwoPhase v;
	  
	  //float vaa,vbb,vcc;
	  float Tr_calc;
	  tThreePhase abc_voltage_svpwm;
	  
	  tTwoPhaseDQ VDQ_rtc,VDQ_ant;
	  float const_VDQ_d, const_VDQ_q;
	  int n_rtc;
	  static RotFluxAng angle;
   protected:
   private:
	
};

#endif // FOC_H
