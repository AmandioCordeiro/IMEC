#include "ClarkeParkTransforms.cpp"
using namespace Maths;

#define FP 1//FP 1 //FLOATP 1

extern float T; 
extern float IDC;
extern long n;

#ifdef FLOATP

extern float Vmax ;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module) 
extern float VDC ;//battery voltage

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

extern s32fp FP_VDC;
extern s32fp FP_Vmax;

#endif
#endif
/*TODO FP */extern tTwoPhaseDQ VDQ_rtc;//used to estimate Rotor time constant

extern float w_ref, wr;

extern float _fds,_fqs, ids_ant, iqs_ant;
extern float fds_,fqs_;

extern float T0,T1,T2;
extern int a1,b1,c1,a2,b2,c2;

extern bool SIGNAL_bat_full;

extern float Rr;//TODO needed??
extern float Rr_initial;//TODO remove at end

extern float IDQ_d_min;// 50//(71.0*0.7)
extern float IDCmin;
extern float IDCmax;
extern bool IDC_min_flag;
//extern bool IDC_less_than_min;
extern float set_point_previous;
extern float set_point_previous_v;
extern bool Flag_reset_controller_idq_q;
extern u_int8_t it_exc_v;//number of iterations in while cycle when voltage exceed the limit
//extern  float IDQ_D_MIN;//TODO //ID_MIN 50//Min. rotor magnetization current(Idq.d) .This value is changed with velocity

extern bool primeiro;
//new
extern int time_bin_max;
extern int time_betw_bin_max;
extern float temperatur_motor;
//#define FLOATP 1

#define  TIME_BIN_MAX 4.2/*6.38*/*1.0/T
//new
#define TIME_IDCMIN_MAX 9.2/*6.38*/*1.0/T//10 s but i do cag.
extern int time_IDCmin;
extern int time_betw_IDCmin_max ;
extern float temperatur_battery ;

#define IDC_MIN_MAX (-80*1.5*0.9)//-200
#define IDC_MIN (-80*1.0*0.9)//ah * 0.5c * cag.
extern bool flag_decrease_idq_q;
 
//new
#define IDQ_D_MIN 60

#define M 0.001500//? mutual inductance
#define M_M (M*M)
#define M_M__Rm (M*M/Rm)
#define M_M_Llr_Llr__Rm__Lr__Lr (M*M*Llr*Llr/Rm/Lr/Lr)
#define FP_M_M FP_FROMFLT(M*M)

#define PI	3.14159265358979

#define np 2.0//? number pair poles
#define Rm 300//650.0//? represents eddy currents. TODO_ don't know this value
#define Llr 0.000140//?
#define Lr (Llr+M)
extern float Rr_est;

extern float Wn, Wc;

extern bool flag_no_torque_motor;

//IGBT values
#define TURN_ON_DELAY_TIME 0.00000012 //TODO put the values of igts, 0.12us
#define RISE_TIME 0.00000006
#define TURN_OFF_DELAY_TIME 0.00000052
#define FALL_TIME 0.00000007
#define T_R_T_F_TIMES  (TURN_ON_DELAY_TIME + RISE_TIME + TURN_OFF_DELAY_TIME + FALL_TIME) 
