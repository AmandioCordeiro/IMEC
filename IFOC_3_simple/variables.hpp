#include "ClarkeParkTransforms.cpp"
using namespace Maths;

extern float T; 
extern float IDC;
extern long n;
extern float VDC, Vmax;

extern float w_ref, wr;

extern tTwoPhaseDQ VDQ_rtc;
extern float _fds,_fqs, ids_ant, iqs_ant;
extern float fds_,fqs_;

extern float T0,T1,T2;
extern int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2

extern bool SIGNAL_bat_full;

extern float Rr;//TODO needed??
extern float Rr_initial;//TODO remove at end

extern float IDQ_d_min;// 50//(71.0*0.7)
extern float IDCmin;
extern bool IDC_min_flag;
//extern bool IDC_less_than_min;
extern float set_point_previous;
extern float set_point_previous_v;
extern bool Flag_reset_controller_idq_q;
extern u_int8_t it_exc_v;//number of iterations in while cycle when voltage exceed the limit
extern  float IDQ_D_MIN;//TODO //ID_MIN 50//Min. rotor magnetization current(Idq.d) .This value is changed with velocity

extern bool primeiro;

//new
extern int time_bin_max;
extern int time_betw_bin_max;
extern float temperatur_motor;
#define  TIME_BIN_MAX 6.38*1.0/T

//IGBT values
#define TURN_ON_DELAY_TIME 0.00000012 //TODO put the values of igts, 0.12us
#define RISE_TIME 0.00000006
#define TURN_OFF_DELAY_TIME 0.00000052
#define FALL_TIME 0.00000007
#define T_R_T_F_TIMES  (TURN_ON_DELAY_TIME + RISE_TIME + TURN_OFF_DELAY_TIME + FALL_TIME) 
