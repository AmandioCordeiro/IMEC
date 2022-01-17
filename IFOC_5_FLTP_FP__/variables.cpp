#include "variables.hpp"

float T = ( 1.0 / ( 1 << 14 ));//(0.000128/2);//8khz? TODO sample(cycle) period,//TODO in embedded remove  T/2 because simulation of motor be half sample period, in real: T
float IDC = 0.0;//estimated DC current
long n = 0;//number of iterations of cycle control

#ifdef FLOATP
float Vmax;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module)//TODO JAN
float VDC;//battery voltage

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

float wr = 0.00000000001;/*rotor speed*/
float w_ref = 0.0;/*set speed on PI controller*/
bool flag_no_torque_motor = false;

float _fds,_fqs, ids_ant, iqs_ant;
float fds_,fqs_;
float T0,T1,T2;
int a1 , b1 , c1 , a2 , b2 , c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2. Value of tension composed by 2 vectores??TODO??right??

bool SIGNAL_bat_full = false ;

 float Rr = 0.0065 ;//TODO needed??edit. julgo q usado somente na simulaçao de motor, logo nao estou a ver mt bem a ujtilidade, ? se no caso da resistencia variar com a temp., mas como já esta compensado no RTC
 float  Rr_est = Rr ;//TODO needed??
//Rrest float Rr_initial = Rr ;//TODO remove at end

float IDQ_d_min = 60 ;//(71.0*0.7)

float IDCmin = -66 ;//-55;
float IDCmax = 240;//new

bool IDC_min_flag = false ;
//bool IDC_less_than_min = false;
bool flag_decrease_idq_q = false ;

float set_point_previous = 0.0 ;
bool Flag_reset_controller_idq_q = false ;
float set_point_previous_v = 0.0 ;
u_int8_t it_exc_v = 0 ;
//float IDQ_D_MIN = 60;

bool primeiro = false ;
//new
int time_bin_max = 0 ;
int time_betw_bin_max = ( int ) ( 610 * 2 * 1 / T ) ;
float temperatur_motor = 40 ;

//new
int time_IDCmin = 0 ;
int time_betw_IDCmin_max =  ( int ) ( 610 * 2 * 1 / T ) ;
float temperatur_battery = 28 ;
//new			
float Wn = 0.0 /*speed that limit max torque speed region from max current(power) limit region */, Wc = 0.0/*speed that limit max current(power) limit region from max Power speed(voltage) limit region*/;
