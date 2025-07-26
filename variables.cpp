#include "variables.hpp"

float T = 0.000250;//4khz ?? 1.0 / ( 1 << 12 );// 0.000125;//1.0 / 8000.0;//8000;//( 1 << 13 ));//16khz//(0.000128/2);//8khz? TODO sample(cycle) period,//TODO in embedded remove  T/2 because simulation of motor be half sample period, in real: T

float T_16khz = 1.0 / ( 1 << 12 );// 0.000125;//1.0 / 8000;//( 1 << 14 ));//14

float T_8khz = 1.0 / ( 1 << 12 ); //doc.: << 12 is 4khz

float IDC = 0.0;//estimated DC current
long n = 0;//number of iterations of cycle control


float Lr = (Llr+M) ;
float M_M = (M*M);
float M_M__Rm = (M*M/Rm);
float M_M_Llr_Llr__Rm__Lr__Lr = (M*M*Llr*Llr/Rm/Lr/Lr);
s32fp FP_M_M = FP_FROMFLT(M*M) ;

//float Imax_Imax_ro_ro = Imax*Imax*ro*ro;
//float ro_ro = ro * ro ;
//float ro_ro_Ls_Ls_Ls_Ls = ro * ro * Ls * Ls * Ls * Ls ;

//COntrollers: _p gain value applied at error
//				_i "	"       "     " error integral
//				_d not used
//				_Min_pid_res Minimum PI end value
//				_Max_pid_res Maximum PI end value
//				_cel acceleration at setpoint in the controller until the setpoint set by "user"
//
//TODO_ insert values and descoment in end #define vel_p 20.0 	//?
//TODO_ insert values and descoment in end #define vel_i 0.0006 //?

//Attention we want control current but in VSI (voltage source inverter) we change voltage to this. Needed also decoupling, component .d of current depends also from component .q of volatage; idem for .q
//#define current_control_q_p 1.0//TODO//0.7//10//? 
#ifdef NODECOUPLING
float current_control_q_Max_pid_res = 170;//8.8 ;//* 1.28;//1.9//*3 better ef, but bigger over tension//*1.5;//*1.29;//new TODO
float current_control_q_Min_pid_res = -170;//8.8 ;//* 3;//* 3 better """"";//*1.29;//new TODO doing
float current_control_q_p = 0.66 * 7 ;//0.09 here1.56 ;//2.0*0.99/*new 1.56*//*1.4*//*0.64/*0.082 1.0*/;
float current_control_d_p = 1;//2.14 1.6 2.2 ;//mudei0.54;//0.5 a experiencia tava 0.63
float current_control_d_i = 0.00088 ;//mudei0.0004 0.00001;<---testes otem 0.03
#else
float current_control_q_p = 0.66 / 4 /*60*/ ;//0.09 here1.56 ;//2.0*0.99/*new 1.56*//*1.4*//*0.64/*0.082 1.0*/;
float current_control_q_Max_pid_res = 7.7;//* 1.28;//1.9//*3 better ef, but bigger over tension//*1.5;//*1.29;//new TOD
float current_control_q_Min_pid_res = -7.7 ;//* 3;//* 3 better """"";//*1.29;//new TODO doing
float current_control_d_p = 1.51 / 4/*50*/;//2.14 1.6 2.2 ;//mudei0.54;//0.5 a experiencia tava 0.63
float current_control_d_i = 0.00088 ;//mudei0.0004 0.00001;<---testes otem 0.03
#endif

//TODO remove at end the folloving block, replace for defines
//PI controllers values
//float vel_p = 6.6 ;////20.0;//6;//1000;//1;//1//9/ /1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO: speed controller gain 
//float vel_i = 0.0009;//006 ;//0.00006;//TODO: speed controller integral gain 
//float torque_control_p = /*to use current controller */ 4.6 /* 16 estava este val em torq_control:0.33,mas experimentei c 1.4;*//* e deu bons result.*//*3.6*0.124*/;//alterei*0.5 TODO: torque controller gain
//float torque_control_i = 0.00025;//8 /*alterei0.0009(0.001) 0.008*///TODO: torque controller integral gain


//TODO remove at end the folloving block, replace for defines
//PI controllers values
float vel_p = 20 ;//6.6//20.0;//6;//1000;//1;//1//9/ /1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO: speed controller gain 
float vel_i = 0.0009;//006 ;//0.00006;//TODO: speed controller integral gain 
float torque_control_p = /*to use current controller */ 4.3 /* 16 estava este val em torq_control:0.33,mas experimentei c 1.4;*//* e deu bons result.*//*3.6*0.124*/;//alterei*0.5 TODO: torque controller gain
float torque_control_i = 0.00025;//8 /*alterei0.0009(0.001) 0.008*///TODO: torque controller integral gain






#ifdef FLOATP
float Vmax;	//Vmax = VDC/CONST_SQRT3_ (voltage circle module)//TODO JAN
float VDC;//battery voltage
s32fp FP_VDC = 1;
s32fp FP_Vmax = 1;
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

s32fp set_point_previous = FP_FROMFLT( 0.0 ) ;
bool Flag_reset_controller_idq_q = false ;
s32fp set_point_previous_v = FP_FROMFLT( 0.0 );
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
