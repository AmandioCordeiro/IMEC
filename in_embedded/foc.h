#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "my_fp.h"
#include <math.h>  //dont know what math is needed
#include <tgmath.h>
#include "ClarkeParkTransforms.cpp"
#include "Rot_Flux_Ang.hpp"
#include "tpid_class.h"

using namespace Maths;

class FOC
{
   public:
      static void ParkClarke(s32fp il1, s32fp il2, uint16_t angle);
      static void InvParkClarke(s32fp id, s32fp iq, uint16_t angle);
      static void SetDirection(int dir);
      static s32fp id;
      static s32fp iq;
      
      static uint32_t DutyCycles[3];
	  FOC();
	  ~FOC();
	  void GetDutyCycles(float il1, float il2, float VDC, float w_ref/*commanded rotor speed*/, float wr/*rotor speed*/);
	  float Vmax, il3, IDC;
	  int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2
	  float T0,T1,T2;
	  float pwm_a,pwm_b,pwm_c;// PWM
	  
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
	  tThreePhase abc_current;
	  float vaa,vbb,vcc;
	  float Tr_calc;
	  tThreePhase abc_voltage_svpwm;
	  float RotorFluxAngle;
	  float VDQ_rtc;
	  float const_VDQ_d, const_VDQ_q,VDQ_ant;
	  int n_rtc=0;
   protected:
   private:
	RotFluxAng angle;
};

#endif // FOC_H
