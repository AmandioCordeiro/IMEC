#include "variables.hpp"
float T = 0.000250;//<- 4khz //0.000125 <- 8khz;//( 1.0 / ( 1 << 13 ));//0.000125( 1.0 / ( 1 << 13 ));//(0.000128/2);//8khz? TODO sample(cycle) period,//TODO in embedded remove  T/2 because simulation of motor be half sample period, in real: T
float IDC = 0.0;//estimated DC current
long long ni = 0;//number of iterations of control cycle 

float pwm_a,pwm_b,pwm_c;// PWM


#ifdef FLOATP
float Vmax ;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module)//TODO JAN
float VDC ;//battery voltage
s32fp FP_VDC = 1 ;
s32fp FP_Vmax = 1 ;
//TODO FP???tTwoPhaseDQ VDQ_rtc( 0.0 , 0.0 );//used to estimate Rotor time constant

#else
#ifdef FP
float VDC;//TODO REMOVE AT FP?
float Vmax;//TODO REMOVE AT FP?
s32fp FP_VDC = 1;
s32fp FP_Vmax = 1;

//TODO FP???FP_tTwoPhaseDQ FP_VDQ_rtc( 1 , 1 );

#else

float VDC;//battery voltage
s32fp FP_VDC = 1;
float Vmax;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module)//TODO JAN
s32fp FP_Vmax = 1;

//TODO FP???tTwoPhaseDQ VDQ_rtc( 0.0 , 0.0 );//used to estimate Rotor time constant

#endif

#endif

s32fp FP__1 = FP_FROMFLT(-1.0), FP_1 = FP_FROMFLT(1.0),FP_0 = FP_FROMFLT(0.0), FP_Rm = FP_FROMFLT( Rm ) ,FP_M = FP_FROMFLT( M ) , FP_Llr = FP_FROMFLT( Llr ), FP_Rr_est = FP_FROMFLT( Rr );//TODO REMOVE?


float wr = 0.0000001;/*rotor speed*/
float w_ref = 3;/*set speed on PI controller*/
bool flag_no_torque_motor = false;

//float _fds,_fqs, ids_ant, iqs_ant;
//float fds_,fqs_;
float T0,T1,T2;
int a1 , b1 , c1 , a2 , b2 , c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2. Vector of tension composed by 2 vectores??TODO??right??

bool SIGNAL_bat_full = false ;// flag to inform battery full

 float Rr = 0.0065 ;//rotor resistance, TODO needed??edit. julgo q usado somente na simulaçao de motor, logo nao estou a ver mt bem a ujtilidade, ? se no caso da resistencia variar com a temp., mas como já esta compensado no RTC
 float  Rr_est = Rr ;//TODO needed??

//Rrest float Rr_initial = Rr ;//TODO remove at end

//static float IDQ_d_min = 60 ;//(71.0*0.7)

//float IDCmin = -66 ;//-55;
//float IDCmax = 240;//new

//bool IDC_min_flag = false ;
////bool IDC_less_than_min = false;

bool flag_decrease_idq_q = false ;

s32fp set_point_previous = FP_FROMFLT( 0.0 ) ;
bool Flag_reset_controller_idq_q = false ;
s32fp set_point_previous_v = FP_FROMFLT( 0.0 );
uint8_t it_exc_v = 0 ;
//float IDQ_D_MIN = 60;

bool primeiro = false ;
//new
//int time_bin_max = 0 ;
//int time_betw_bin_max = ( int ) ( 610 * 2 * 1 / T ) ;
//float temperatur_motor = 40 ;

//new
//int time_IDCmin = 0 ;
//int time_betw_IDCmin_max =  ( int ) ( 610 * 2 * 1 / T ) ;
//float temperatur_battery = 28 ;
//new
float Wn = 0.0 /*speed that limit max torque speed region from max current(power) limit region */, Wc = 0.0/*speed that limit max current(power) limit region from max Power speed(voltage) limit region*/;

const float T_R_T_F_TIMES  = (TURN_ON_DELAY_TIME + RISE_TIME + TURN_OFF_DELAY_TIME + FALL_TIME);
const float M_M = (M*M);
const float M_M__Rm = (M*M/Rm);
const float M_M_Llr_Llr__Rm__Lr__Lr = (M*M*Llr*Llr/Rm/Lr/Lr);
s32fp FP_M_M = FP_FROMFLT( M * M );
const float IDC_MIN_MAX = (-80*1.5*0.9);//-200
const float IDC_MIN = (-80*1.0*0.9);//ah * 0.5c * cag.

const float  TIME_BIN_MAX = 4.2/*6.38*/*1.0/T;
//new
const float TIME_IDCMIN_MAX = 9.2/*6.38*/*1.0/T;
const float Lr = (Llr+M);
