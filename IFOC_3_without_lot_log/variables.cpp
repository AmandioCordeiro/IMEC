#include "variables.hpp"
float T =(1.0/(1<<14));//(0.000128/2);//8khz? TODO sample period,//TODO in embedded remove  T/2 because simulation of motor be half sample period, in real: T
float IDC=0.0;
long n=0;
float VDC;
float Vmax;

float wr=0.00000000001;
float w_ref=0.0;
tTwoPhaseDQ VDQ_rtc(0.0,0.0);

float _fds,_fqs, ids_ant, iqs_ant;
float fds_,fqs_;
float T0,T1,T2;
int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2

bool SIGNAL_bat_full=false;

float Rr=0.0065;//TODO needed??
float Rr_initial=Rr;//TODO remove at end

float IDQ_d_min = 50;//(71.0*0.7)

float IDCmin = -55;
bool IDC_min_flag=false;
//bool IDC_less_than_min = false;

float set_point_previous = 0.0;
bool Flag_reset_controller_idq_q = false;
float set_point_previous_v = 0.0;
int it_exc_v = 0;
float IDQ_D_MIN = 50;

bool primeiro=false;

//new
int time_bin_max = 0;
int time_betw_bin_max = (int) (610*2*1/T) ;
float temperatur_motor = 40;
