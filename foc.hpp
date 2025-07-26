#ifndef FOC_H
#define FOC_H

#include "battery_simulation.hpp"
#include "my_math.h"
#include "variables.hpp"
#include "ClarkeParkTransforms.cpp"

#define FIXED_FLOAT(x) std::fixed <<std::setprecision(4)<<(x)//TODO remove at end
#define FIXED_FLOAT_L(x) std::fixed <<std::setprecision(9)<<(x)//TODO remove at end

//#ifndef T
//#define T 0.000125//8khz? TODO sample period
//#endif

#define T_lag (0.0000001)//? TODO_: time since read values until apply PWM 



extern float  Lls, Ls , ro , Ls_ro , TM1 , ro_ro_Ls_Ls_Ls_Ls__KT__KT , Imax_Imax , Imax_Imax_Imax_Imax , Imax_Imax__2 , KT , four__KT_KT;
extern s32fp FP_Ls , FP_Idn , FP_RS , FP_Imax , FP_Imax_Imax , FP_Imax_Imax_ro_ro , FP_KT , FP_CONST_Tm2;


//#define FP_M FP_FROMFLT ( M )
//#define Ls ( 0.000140 + M )//? Lls+M
//#define FP_Ls	FP_FROMFLT ( Ls )
#define Idn 112//102//103//138// 127//138// 103/*141*///137//129//128//103//99.0//101.0//?// Id nominal, rotor current magnetization nominal
//#define FP_Idn	FP_FROMFLT( Idn )
#define Tr 0.252308//? (Lr/Rr)
#define Rs 0.012//?
//#define FP_Rs FP_FROMFLT( Rs )
//#define FP_Rm FP_FROMFLT( Rm )
//#define ro ( 1.0 - ( M * M ) / ( Ls * Lr ))
//#define FP_ro FP_FROMFLT ( ro )
//#define Ls_ro ( Ls * ro )
//#define FP_INV__ro_ro FP_FROMFLT ( 1 / ( ro * ro ))
//#define FP_ro_ro FP_FROMFLT ( ro * ro )
//#define FP_KT__2__ro FP_FROMFLT ( KT / 2.0 / ro)
//#define ro_ro_Ls_Ls_Ls_Ls__KT__KT ( ro * ro * Ls * Ls * Ls * Ls / KT / KT )
#define Imax 370//395//333//400//333//400////348//*325*///320//340//344//400.0//422//? max igbt current
//#define FP_Imax	FP_FROMFLT ( Imax )
//#define FP_Imax_Imax FP_FROMFLT ( Imax * Imax )//TODO remove
//#define Imax_Imax_ro_ro FP_FROMFLT ( Imax * Imax * ro * ro )
//#define Imax_Imax ( Imax * Imax )
//#define Imax_Imax_Imax_Imax ( Imax * Imax * Imax * Imax )
//#define Imax_Imax__2 ( Imax_Imax / 2.0 )
//#define KT ( 3.0 / 2.0 * np * M * M / Lr )
//#define FP_KT FP_FROMFLT ( KT )
//#define four__KT_KT ( 4.0 / KT / KT )
//#define FP_CONST_Tm2 FP_FROMFLT ( KT / ( 1.0 - ro * ro ))

//#define TM1 ( KT * Idn * sqrt ( Imax * Imax - Idn * Idn ))// Maximum torque to apply in first zone

#define LOAD_1_sec 13//? to limit initial current setpoint torque to this
//speed up code
//#define T_LOAD_1_sec 0.135 / T 
//#define KT_t_cag 0.9*KT

#ifdef NODECOUPLING
#define SIZE_AVR_FILT_LMA 60//TODO was 13
#define SIZE_AVR_FILT_LMA__1 59 //(SIZE_AVR_FILT_LMA -1 )
#else
#define SIZE_AVR_FILT_LMA 20//TODO was 13
#define SIZE_AVR_FILT_LMA__1 19 //(SIZE_AVR_FILT_LMA -1 )
#endif
//#define SIZE_AVR_FILT_TR 100


extern float vel_p, vel_i, torque_control_p, torque_control_i, current_control_d_p, current_control_d_i;
float dy_nt_(float /*&*/y1, float /*&*/y_1);
float d2y_nt_(float /*&*/y1, float /*&*/y,float /*&*/y_1);


//void calc_max_mod_volt(tTwoPhase v_bi);

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

//extern float IDC;//TODO: remove at end? if needed. to calc medium value of IDC 
extern float vaa,vbb,vcc;
//extern RotFluxAng angle;
extern float ids,iqs;


extern float vel_p,vel_i,torque_control_p,torque_control_i,current_control_d_p,current_control_d_i,current_control_q_p;

extern float Lsro;//TODO make #define
//#define TR (Lr/Rr)//TODO remove at end
#ifdef FLOATP

extern float RotorFluxAngle;
extern float Vmax;
extern float VDC;
extern tThreePhase abc_current;
extern float ang_u;
extern float max_mod_volt;
extern float ang;

#endif
#ifdef FP

extern s32fp FP_RotorFluxAngle;
extern s32fp FP_Vmax;
extern s32fp FP_VDC;
extern FP_tThreePhase FP_abc_current;
extern s32fp FP_ang_u;
extern s32fp FP_max_mod_volt;
extern s32fp FP_ang;
#endif
#ifdef MIX_FLOATP_FP

extern tThreePhase abc_current;
extern float ang_u;
extern float max_mod_volt;
extern float ang;
extern float Wm;
extern float Vmax;


extern float RotorFluxAngle;
extern s32fp FP_RotorFluxAngle;
extern FP_tThreePhase FP_abc_current;
extern s32fp FP_ang_u;
//TODO JANextern s32fp FP_max_mod_volt;
extern s32fp FP_ang;

#endif
/*TODO FP???*/extern float Wm;

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
#ifdef FLOATP
	void calc_max_mod_volt(tTwoPhase v_bi);
	//void FP_calc_max_mod_volt(FP_tTwoPhase v_bi);//TODO
	tTwoPhaseDQ IDQ, VDQ, VDQ_contr;
	tTwoPhase v;

	tThreePhase abc_voltage_svpwm;
	static RotFluxAng angle;
	
#endif
#ifdef FP
  	  
  	  FP_tTwoPhaseDQ FP_IDQ, FP_VDQ, FP_VDQ_contr, FP_VDQ_rtc;
  	  FP_tTwoPhase FP_v;
  	  tTwoPhaseDQ VDQ_contr;//TODO FP
  	  FP_tThreePhase FP_abc_voltage_svpwm;
	 //static RotFluxAng angle; //TODO FP
	 void FP_calc_max_mod_volt(FP_tTwoPhase v_bi);
		static FP_RotFluxAng FP_angle;
 
#endif	 
#ifdef MIX_FLOATP_FP
	  void calc_max_mod_volt(tTwoPhase v_bi);
	  void FP_calc_max_mod_volt(FP_tTwoPhase v_bi);
  	  tTwoPhase v;
	  FP_tTwoPhase FP_v;
      tTwoPhaseDQ IDQ, VDQ, VDQ_contr;
   	  FP_tTwoPhaseDQ FP_IDQ, FP_VDQ, FP_VDQ_contr, FP_VDQ_rtc;

      tThreePhase abc_voltage_svpwm;
   	  FP_tThreePhase FP_abc_voltage_svpwm;

	  static RotFluxAng angle;
	  static FP_RotFluxAng FP_angle;
 
#endif
tTwoPhaseDQ VDQ_rtc,VDQ_ant;/*TODO FP*/

//	  float get_VDC();
	  void vel_tune_pid();//TODO_ can be removed at end after find good values 
	  void torque_control_tune_pid();//TODO_ can be removed at end after find good values
	

	  void GetDutyCycles(float il1, float il2, float VDC, float w_ref/*commanded rotor speed*/, float wr_/*rotor speed*/);
	  bool exc_v_PI();
	  float il3/*, IDC*/;
	  //int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2
	  //float T0,T1,T2;
	  float pwm_a,pwm_b,pwm_c;// PWM
	 // float Wn,Wc;//speed of limit regions, max. torque region, max. power region, max. power-speed region
	  
	  //float TR_v[SIZE_AVR_FILT_TR];
					float Tr_calc_this;//, 
					//TR_avr;
	  
	  //PI controllers
	  _pid vel;
	  _pid torque_control;
	  _pid current_control_d;
	  _pid current_control_q;
	
		s32fp get_torq_setpoint();
	
	  float vel_Min_pid_res;
	  float vel_Max_pid_res;
	  

  	  float IDQ_d1,IDQ_d_1,IDQ_d_,IDQ_d_p,IDQ_d_pp;
	  float IDQ_q1,IDQ_q_1,IDQ_q_,IDQ_q_p,IDQ_q_pp;
	  float wmr_,wmr1,wmr_p,wmr__1;
	  
	  //float vaa,vbb,vcc;
	  float Tr_calc;
float IDQ_d_lma_v[ SIZE_AVR_FILT_LMA  ];
float IDQ_d_lma_avr;
float IDQ_d_lma_avr_;


	  float const_VDQ_d, const_VDQ_q;
	  int n_rtc;
	  static battery_simulation bat;//TODO inplement in real, need know battery voltage for each cycle

   protected:
   private:
	
};

#endif // FOC_H
