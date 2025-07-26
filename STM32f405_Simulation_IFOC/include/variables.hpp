#ifndef VARIABLES_H
#define VARIABLES_H

#include "../../src/ClarkeParkTransforms.cpp"
#include "my_fp_2.h"
using namespace Maths;


#define VehicleSimulation 1
#define VehicleSimulationFull 1

#define IFOC_ALG 1
//#define IFOC_ALG_TORQUE 0

extern float pwm_a,pwm_b,pwm_c;// PWM

#define FLOATP 1//FP 1 //FLOATP 1

extern float T;
extern float IDC;
extern long long ni;

#ifdef FLOATP

extern float Vmax ;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module)
extern float VDC ;//battery voltage
extern s32fp FP_VDC;
extern s32fp FP_Vmax;
#else
#ifdef FP

extern s32fp FP_VDC;
extern s32fp FP_Vmax;
//TODO FP
extern float VDC;
extern float Vmax;


#else

extern float Vmax;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module)//TODO JAN
extern float VDC;//battery voltage


#endif

#endif
extern s32fp FP_1, FP_Rm, FP_M , FP_Llr , FP_Rr_est , FP__1 , FP_0;//TODO REMOVE?
#define FP_Lr FP_FROMFLT(Lr)//TODO calc const to efic

/*TODO FP */extern tTwoPhaseDQ VDQ_rtc;//used to estimate Rotor time constant

extern float w_ref, wr;

//extern float _fds,_fqs, ids_ant, iqs_ant;
//extern float fds_,fqs_;

extern float T0,T1,T2;
extern int a1,b1,c1,a2,b2,c2;

extern bool SIGNAL_bat_full;

extern float Rr;//TODO needed??
//extern float Rr_initial;//TODO remove at end

//extern float IDQ_d_min;// 50//(71.0*0.7)
//extern float IDCmin;
//extern float IDCmax;
//extern bool IDC_min_flag;
////extern bool IDC_less_than_min;
extern s32fp set_point_previous;
extern s32fp set_point_previous_v;
extern bool Flag_reset_controller_idq_q;
extern uint8_t it_exc_v;//number of iterations in while cycle when voltage exceed the limit
//extern  float IDQ_D_MIN;//TODO //ID_MIN 50//Min. rotor magnetization current(Idq.d) .This value is changed with velocity

extern bool primeiro;
//new
//extern int time_bin_max;
//extern int time_betw_bin_max;
//extern float temperatur_motor;
//#define FLOATP 1

extern const float  TIME_BIN_MAX ;//= 4.2/*6.38*/*1.0/T;
//new
extern const float TIME_IDCMIN_MAX ;//= 9.2/*6.38*/*1.0/T;//10 s but i do cag.
//extern int time_IDCmin;
//extern int time_betw_IDCmin_max ;
//extern float temperatur_battery ;

extern const float IDC_MIN_MAX ;//= (-80*1.5*0.9);//-200
extern const float IDC_MIN ;//= (-80*1.0*0.9);//ah * 0.5c * cag.
extern bool flag_decrease_idq_q;

//new
#define IDQ_D_MIN 81 //50 60 86 92.14(91.14), 90 .23, 94 .33 ;98. 92.42(91.36 com 4 velocidadde de caixa, 91.60 com mudança de valores aquando das reducoes

#define M 0.001500 //? mutual inductance
extern const float M_M;// = (M*M);
extern const float M_M__Rm;// = (M*M/Rm);
extern const float M_M_Llr_Llr__Rm__Lr__Lr;// = (M*M*Llr*Llr/Rm/Lr/Lr);
extern s32fp FP_M_M ;//= FP_FROMFLT(M*M);

#define PI	3.14159265358979

#define np 2.0//? number pair poles
#define Rm 200//650.0//? represents eddy currents and histeresis?(core loss?). TODO_ don't know this value
#define Llr 0.000140//?


extern const float Lr ;//= (Llr+M);
#ifdef FP
#endif

extern float Rr_est;

extern float Wn, Wc;

extern bool flag_no_torque_motor;

//IGBT values
#define TURN_ON_DELAY_TIME 0.00000012 //TODO put the values of igts, 0.12us
#define RISE_TIME 0.00000006
#define TURN_OFF_DELAY_TIME 0.00000052
#define FALL_TIME 0.00000007
extern const float T_R_T_F_TIMES ;// = (TURN_ON_DELAY_TIME + RISE_TIME + TURN_OFF_DELAY_TIME + FALL_TIME);

#define MAX_RAD_S 1000
#endif
