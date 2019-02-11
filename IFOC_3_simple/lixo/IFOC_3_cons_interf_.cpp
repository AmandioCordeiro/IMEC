/*
 * IFOC_3_.cpp
 * 
 * Copyright 2019 amrc <amandio.miguel@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <tgmath.h>
#include <vector>
#include "ClarkeParkTransforms.cpp"
#include "tpid_class.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
#include "Rot_Flux_Ang.hpp"
#include "foc.hpp"
#include <string>
#include <string.h>
#include <fstream>
//#include <complex> 
#include <iomanip>

using namespace std;
using namespace Maths;

float Lrro=(ro*Lr);//0.0021891
#define R 0.34 //whell radius
#define MASSA (3500.0)//massa veiculo
/*#define*/float T_G_R = (4.313/*2.33*//*1.436 1 0.789*/*4.1/*reduction:*1.9*/); //total gear racio. (gear_box*final_gear*"reduction_low_ratio")
float T_G_R_ant=T_G_R;	
float gradiente=0;//-0.25;//0.15;
#define G 9.81
#define C_D 0.5
#define RO_AIR 1.25
#define C_R 0.01
#define A_F 2


float sum_power_out=0.001;
float sum_power_in=0.000001;
float Torq_net=0.0;
float sum_power_trac =0.0;
float sum_power_battery = 0.0;
bool e=true;
int n_rtc=0;
float T_G_R_b_clutch=T_G_R_ant;//thing its not needed
//float Vf=0.0;
float Te=0.0;//torque motor
float J2=0.0;//inercia vehicle??
float w_ref_ant=0.0;
float distance_=0.0;
float velocidade=0.0;//vel. clutch
float load=0.0, load_s=0.0;
float b, end_f;
float w_ref=0.0;

float pl=0.0;
bool p=true;

float torq_L=0.0;
float J=(0.071);//TODO:?? 1pv5135-4ws14: 0.071

float wr=0.0;

bool quit=false;



//clutch:
#define N_P 2.0 
#define us 0.15
#define ud 0.24
#define fi 1.5
#define R1 0.074
#define R2 0.104
#define XTO_CNT 0.00402
#define XTO_CLS 0.00778
#define XTO_MAX 0.012
#define F_MAX 5000

#define CLUTCH_TIME 0.5//0.13//0.5 sec.
int clutch_time=-1;




FOC foc_;//TODO_ create it in embedded
RotFluxAng	FOC::angle(0.0);//TODO_ create it in embedded


//float Vmax=VDC/sqrt(3.0);	//need actualization in pratice in function get_VDC( 

float t=0.0;

float roLs=(ro*Ls);//0.013998//ro*Ls
float fdr=0.0,fqr=0.0, Iqm=0.0, Idm=0.0, fqs=0.0, fds=0.0;
float Rr=0.0065;

extern float ids, iqs, IDC;

//////////
float get_w_r();//TODO implement in REAL
//////////
	void set_w_ref(float wreff) {w_ref=wreff;};
	float get_w_ref(){return w_ref;};
	void set_torq_L(float torque){torq_L=torque;};

	float get_torq_L(){return torq_L;};//TODO remove in real

//----------------	the most simple of doing the following, but i dont know if is the more accurate, is measure only IDC, the current flowing from batteries
	float get_ias(){//TODO in real read from sensors
		abc_current.a= (InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).a);
		return abc_current.a;
	};
	float get_ibs(){//TODO in real read from sensors
		abc_current.b= (InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).b);
		return abc_current.b;
	};
	float get_ics(){//TODO in real read from sensors or if system simetric and equilibrated: 
		//abc_current.c=-abc_current.a-abc_current.b; but is ??better get abc_current.c from measure
		abc_current.c=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).c);
		return abc_current.c;
	};
//-----------------	

float torque_g(float Vds,float Vqs,float angle_get_wm){//torque generated, frame generic . used #######"backward euler method"######
			cout<<"Vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
	float Lls=Ls-M;
		if (angle_get_wm==0.0)angle_get_wm=0.00000000001;
		float	s_2=(angle_get_wm-wr*np)/angle_get_wm*(angle_get_wm-wr*np)/angle_get_wm;
		float Rfe=Rm/(s_2+1);		
		//usado (fds_n+1-fds_n)/T=fds_n+1 +(...), e nao: (fds_n+1/T-fds_n)/T=fds_n+(...)
		float fds_=1/(1/T+Rs/Lls)*(Vds+fds/T+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		float fqs_=1/(1/T+Rs/Lls)*(Vqs+fqs/T-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		float fdr_=1/(1/T+Rr/Llr)*(fdr/T+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm);
		float fqr_=1/(1/T+Rr/Llr)*(fqr/T-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm);
		float Idm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm/*-np*wr*/)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		float Iqm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+Iqm/T-(angle_get_wm/*-np*wr*/)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;

		
		fds=fds_;
		fqs=fqs_;
		fdr=fdr_;
		fqr=fqr_;
		Idm=Idm_;
		Iqm=Iqm_;
		
		ids=(fds-M*Idm)/Lls;
		iqs=(fqs-M*Iqm)/Lls;
		
		
	
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_out += abs(IDC*VDC) ; else sum_power_out += (t*wr);
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_in += abs(t*wr); else sum_power_in += (IDC*VDC);
	sum_power_trac += t*wr;
	sum_power_battery += IDC*VDC/10000/3600;fTorque<<"battery energy : "<<sum_power_battery<<" wh"<<endl;
	if (n*T==195)fTorque<<"efficiency end cycle "<<sum_power_out/sum_power_in<<endl;
	fTorque<<FIXED_FLOAT(n*T)<<" sec., Torque_generated (previous): "<<t/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl<<"power (Torque_generated*rotor_velocity)(previous):"<<(t/*np*/*wr)<<"; power (Vdc*Idc)(previous):"<<(IDC*VDC)<<endl;if (IDC<0) fTorque<<"efficiency (instant. generat.)(previous) :"<<(IDC*VDC)/(t/*np*/*wr);else fTorque<<" efficiency (instant. motor)(previous):"<<(t/*np*/*wr)/(IDC*VDC);fTorque<<" machine medium efficiency: "<<sum_power_out/sum_power_in/*<<" ef: "<<sum_power_trac/sum_power_battery*/<<endl<<"We- sincronous speed in ang. electric: "<<angle_get_wm<<endl<<"(angle_get_wm-wr*np) \"slip angle:\" "<<(angle_get_wm-wr*np)<<endl;//<<"torq_l:"<<torq_L<<endl;	
	t=(3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds)));
	return t;
};

float torque(float vas,float vbs,float vcs){
		return torque_g(ClarkePark(RotorFluxAngle/*np*theta_r*/,tThreePhase(vas,vbs,vcs)).d/*alpha*/,ClarkePark(RotorFluxAngle/*np*theta_r*/,tThreePhase(vas,vbs,vcs)).q/*beta*/,FOC::angle.get_wm());
};


void clutch(){//
	float J1=J;
	//float J2=MASSA*R*R/T_G_R/T_G_R;
	//float Te=torque(va,vb,vc);
	float T_l=get_torq_L();
	float n1=get_w_r();
	//Torq_net=Te-T_l;
	float n2=velocidade;//??
	float Wfc=n2-n1;
	
	float Ru=1.0/(R2-R1)*(us/2.0*R2*R2+(ud-us)*(R2+1.0)/(fi*Wfc)*log(cosh(fi*R2*Wfc))-(us/2.0*R1*R1+(ud-us)*(R1+1.0)/(fi*Wfc)*log(cosh(fi*R1*Wfc))));
	float Xto=XTO_MAX/(CLUTCH_TIME/T)*clutch_time;
	float Ffc=F_MAX;
	if (Xto>0.0 && Xto<=XTO_CNT)
		{Ffc=0.0;}
	else if(Xto>XTO_CNT && Xto<=XTO_CLS)
			Ffc=F_MAX*(1.0-sqrt(1.0-pow((Xto-XTO_CNT)/(XTO_CLS-XTO_CNT),2)));		 		
	float Tfc=-N_P*Ru*Ffc/T_G_R;
	
	n1=n1+(Te-Tfc)/J1*T;
	n2=n2+(Tfc-T_l)/J2*T;
	
	velocidade=n2;
	wr=n1;
		fVel<<"Wr(rotor velocity): "<<n1<</*" a: "<<a<<*/" torque_clutch: "<<Tfc<<endl;
};
float get_wr(float va,float vb,float vc/*float torq_g,float torq_L*/){
		//float a=atan(gradiente);//TODO estava comentado este e alinha a seguir --load in ifoc_2_sem_gui
		//torq_L=(C_D*1/2*RO_AIR*A_F*wr/T_G_R*R*wr/T_G_R*R+C_R*MASSA*G*cos(a)+MASSA*G*sin(a))*R/T_G_R;
		Te=torque(va,vb,vc);
		if (e==true && velocidade*R/T_G_R*3.6 < 26){
				T_G_R = (4.313 /*2.33 1.436 1 0.789*/*4.1/*1.9*/);
			if (T_G_R_ant!=T_G_R){
				T_G_R_b_clutch=T_G_R_ant;
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//jan 2019 ;w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
				
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
				//FOC::angle.reset();
				}

			}
		if (e==true && velocidade*R/T_G_R*3.6 > 30 && velocidade*R/T_G_R*3.6 < 66 )
			{T_G_R = (/*4.313*/ 2.33/*1.436 1 0.789*/*4.1/*1.9*/);
			if (T_G_R_ant!=T_G_R){
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//jan 2019 ;w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
				
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
				//FOC::angle.reset();
				}
			}
		if (e==true && velocidade*R/T_G_R*3.6 > 70 )
			{T_G_R = (/*4.313 2.33*/1.436/* 1 0.789*/*4.1/*1.9*/);
			if (T_G_R_ant!=T_G_R){
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//jan 2019 ;w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
			
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
				//FOC::angle.reset();
				}
			}
				
		J2=MASSA*R*R/T_G_R/T_G_R;
		if(e==true){//if driving cycle:
			if (clutch_time>0){
				clutch();
				//torq_L-=torque_clutch;
				if (clutch_time==1)w_ref=w_ref_ant;	
				fwref<<FIXED_FLOAT(n*T)<<"clutch_time: "<<clutch_time<<endl;
						
				clutch_time--;
				}
			//if (clutch_time==0){
				//torq_L-=torque_clutch;
				//clutch_time=clutch_time-3;
			//	}
			else if (velocidade>0.0 && (velocidade>wr*1.05 || velocidade<wr*0.95))
					clutch();
					else if (velocidade<0.0 && (velocidade<wr*1.05 || velocidade>wr*0.95))
					clutch();
				else{
					Torq_net=Te-torq_L;
					wr=wr+(Torq_net)*T/(J+J2/*R*R*MASSA/(T_G_R*T_G_R)*/);
					velocidade=wr;
					}
		}
		else{//if vheicle mode or by load??not sure//T_G_R=1.436*4.1;//TODONEw, test load 3rth
			if (T_G_R_ant!=T_G_R){
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
			////	clutch_time=CLUTCH_TIME*10000;
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
			////	//FOC::angle.reset();
			}
			if (clutch_time>0){
				clutch();
				//torq_L-=torque_clutch;
				clutch_time--;	
				if (clutch_time==1)w_ref=w_ref_ant;	
				fwref<<FIXED_FLOAT(n*T)<<"clutch_time: "<<clutch_time<<endl;
					
				}
			//if (clutch_time==0){
				//torq_L-=torque_clutch;
				//clutch_time-=3;
			//	}
			else {
				if (T_G_R==0){
					wr=wr+(Te/*torque(va,vb,vc)-torq_L*/)*T/J;
					velocidade+=torq_L/J2*T;
					}
				else if (velocidade>0.0 && (velocidade>wr*1.05 || velocidade<wr*0.95))
					clutch();
					else if (velocidade<0.0 && (velocidade<wr*1.05 || velocidade>wr*0.95))
					clutch();
					else{
						Torq_net=Te/*torque(va,vb,vc)*/-torq_L;
						wr=wr+(Torq_net)*T/(J+J2/*R*R*MASSA/(T_G_R*T_G_R)*/);
						velocidade=wr;
					}
			}	
			}
		return wr;
	};	
float get_w_r(){return wr;};//TODO implement in REAL





/////


void enable_raw_mode()
{
    termios term;
    tcgetattr(0, &term);
    term.c_lflag &= ~(ICANON | ECHO); // Disable echo as well
    tcsetattr(0, TCSANOW, &term);
}

void disable_raw_mode()
{
    termios term;
    tcgetattr(0, &term);
    term.c_lflag |= ICANON | ECHO;
    tcsetattr(0, TCSANOW, &term);
}

bool kbhit()
{
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    return byteswaiting > 0;
}

int main(int argc, char **argv)
{
	
	bool run=false;

//	long c=0;

	char o;

	ifstream driving_cycle("ececol.txt");
	string line;
	int i=0;//here
	bool primeiro=true;
	int gear=2;
	enable_raw_mode();
	
	while (quit==false){
	
		if (/*o=getch()*/kbhit()/*||flag==true*/)
		{
			o=getchar();
			switch(o){
				
				case '\n':break;	
				case 'r' :run=true;/*flag=false*/;break;
				case 'q' :quit=true;/*flag=false*/;break;
				////case 'm' :cout<<"enter magnetizing current reference:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	////tcflush(0, TCIFLUSH);cin>>imrreff;enable_raw_mode();/*fflush(stdin);*/break;
				case 's' :cout<<"enter speed:";e=false;/*e-> to set by driving cycle or by speed*//*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>w_ref;enable_raw_mode();/*fflush(stdin);*/break;
				case 'l' :cout<<"enter torque load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>load_s;enable_raw_mode();p=false;e=false;/*fflush(stdin);*/break; 
				case 'g' :cout<<"enter gear:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>gear;if(gear==0)T_G_R =0; if(gear==1)T_G_R = (4.313/* 2.331.436/* 1 0.789*/*4.1/*1.9*/);if(gear==2)T_G_R = (/*4.313*/ 2.33/*1.436 1 0.789*/*4.1/*1.9*/);if(gear==3)T_G_R = (/*4.313 2.33*/1.436/* 1 0.789*/*4.1/*1.9*/);if(gear==4)T_G_R = (/*4.313 2.33 1.436*/ 1 /*0.789*/*4.1/*1.9*/);enable_raw_mode();/*fflush(stdin);*/break; 
				case 'a' :cout<<"enter prop. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'b' :cout<<"enter int. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 'c' :cout<<"enter prop. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'e' :cout<<"enter prop. gain flux:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>current_control_x_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'f' :cout<<"enter int. gain flux:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>current_control_x_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 'd' :cout<<"enter int. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_i;enable_raw_mode();/*fflush(stdin);*/break;
				case ' ' :cout<<"pause, press r to return";run=false;break;//disable_raw_mode();
	//tcflush(0, TCIFLUSH);char f='p';while(f != ' '){cin>>f;cout<<f;};enable_raw_mode();/*fflush(stdin);*/break;
				case '+' :w_ref++;/*flag=false;*/break;
				case '-' :w_ref--;/*flag=false;*/break;
				//case 'k' :load--;/*flag=false;*/break;
				//case 'p' :load++;/*flag=false;*/break;
				//case 't' :cout<<"set by torque load ";p=false;break;
				case 'v' :cout<<"set by vehicle load,enter road gradiente(ex. 0.0 <-> 0.2): ";p=true;e=false;/*break;*///inicial p=true
				/*case 'm' :cout<<"enter road gradiente";*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>gradiente;enable_raw_mode();/*fflush(stdin);*/break;
				//case 'h' :cout<<"enter power load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	//tcflush(0, TCIFLUSH);cin>>pl;enable_raw_mode();/*fflush(stdin);*/break;
				case 'j' :pl--;/*flag=false;*/break;
				case 'i' :pl++;/*flag=false;*/break;
				//case 'e' : e=true;break;//set by driving_cycle
				
				default:cout<<"invalid operator, \ns(commanded speed)"<<" \na (enter prop. gain speed)"<<" \nb (enter int. gain speed)"<<"\nc (enter prop. gain torq_controll)"<<"\nd (enter int. gain torq_controll)"<<" \nl(load), "<</*\nt(set by torque load), */"\nv(set by vehicle load), \n+(increment speed), \n-(decrement speed) \ng (switch gear)"<</*, k(decrement load),p(increment load),*/", \nr(run), \nq(quit)"<<endl;	
			}
		
		}
		
		else if (run==true){			
			//DEfinition of torque load in function of power specified
		//TODO remove following at end
			if (n*T<0.135)load=LOAD_1_sec*0.9;
			else if (p && get_w_r()!=0) {
				float a=atan(gradiente);
				float load_C_R;if (velocidade>0)load_C_R=C_R*MASSA*G*cos(a);else if(velocidade<0)load_C_R = -C_R*MASSA*G*cos(a);else if (velocidade==0)load_C_R=0;
					load=(C_D*1/2*RO_AIR*A_F*/*wr*/velocidade/T_G_R*R*/*wr*/velocidade/T_G_R*R + load_C_R + MASSA*G*sin(a))*R/T_G_R;//torque resistant vehicle
					}
			else load=load_s;
		//TODO remove above at end
			/*control.*///foc_.vel_tune_pid();//TODO remove at END, in embedded
			/*control.*///foc_.torque_control_tune_pid();
			
			T1T2<<FIXED_FLOAT(n*T)<<" sec. "<<endl;
			fwref<<FIXED_FLOAT(n*T)<<"sec.; motor rotor reference velocity: "<<w_ref<<endl;
			fload<<FIXED_FLOAT(n*T)<<"sec.; Torque load: "<<get_torq_L()<<endl;
			
						
			//four urban drive cycle, ece 15*4
			if (i<4 && e==true && driving_cycle.is_open())
			{				
						if (primeiro){getline(driving_cycle,line);primeiro=false;}
						fVel<<"  line "<<line<<"reference velocity (km/h)"<<endl;
						while(line.length() > 13 )getline(driving_cycle,line);	
						
						vector<char> v(line.size()+1);
						memcpy( &v.front(), line.c_str(), line.size() + 1 );
						
						char* s_line=v.data();
						char* pEnd;
						
						b = strtod (s_line/*szOrbits*/, &pEnd);
						end_f = strtod (pEnd, NULL);
						fVel<<" (b+196*i): "<<(b+196*i)/*<<" b e end :"<<b<<end*/<<endl;
						
						if (b==195 && (n*T) == (b+196*i)){
							driving_cycle.close();
							driving_cycle.open("ececol.txt",std::ifstream::in);
							i++;
							primeiro =true;
							w_ref=end_f*1000/3600*T_G_R/R;
							}
						else if ((n*T) == (b+196*i)) 	
						{	w_ref=end_f*1000/3600*T_G_R/R;
							getline(driving_cycle,line);
						
						}
						//if (b==25)quit=true;//test 
			}
			//extra urban drive cycle
			else if (e==true)
				{
				if (primeiro){
					
					driving_cycle.close();
					driving_cycle.open("eudccol.txt",std::ifstream::in);
					getline(driving_cycle,line);
					primeiro=false;
					}
				fVel<<"  line   "<<line<<"reference velocity (km/h)"<<endl;
				while(line.length() > 13 )getline(driving_cycle,line);	
				
				vector<char> v(line.size()+1);
				memcpy( &v.front(), line.c_str(), line.size() + 1 );
				
				char* s_line=v.data();
				char* pEnd;
				b = strtod (s_line/*szOrbits*/, &pEnd);
				end_f = strtod (pEnd, NULL);
				if ((n*T) == /*b*/(b+196*i)) 	//here
				{	w_ref=end_f*1000/3600*T_G_R/R;
					getline(driving_cycle,line);
				
				}
				if (b==/*175*/400)quit=true;//here
				}	
				
			set_w_ref(w_ref);
			
			set_torq_L(load);//TODO this is only in simulation, not needed in real
			//set_Theta_r();
			foc_.get_VDC();//TODO_ adapt in real(embedded).function must read DC voltage			
			//TODO in real the following reads from e Hall sensors			
			//TODO julgo n ser necessario seguinte, trat em GetDutyCycles
			get_ias();/*cout<<"ia"<<abc_current.a<<endl;*/get_ibs();get_ics();//abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor, is better get abc_current.c from measure				

			//**float ia=get_ias();//TODO to measure cos phi
			//**função a seguir com tempo variavel, necessario resolver para tempo const
			//**svpwm(control);//news gates and times
			/*here*/
			//**iaa_p=ia;
			//**vaa_p=vaa;
			
			
			
/////____
			
			//**get_wr(vaa,vbb,vcc); 
			
			foc_.GetDutyCycles((get_ias()), (get_ibs()), /*get_VDC(),*/ (get_w_ref())/*commanded rotor speed*/, (get_w_r())/*rotor speed*/);
T1T2<<" vaa_n:"<<vaa<<" vbb_n:"<<vbb<<" vcc_n:"<<vcc<<endl<<"IDC_previous: "<<IDC<<endl;
			
			get_wr(vaa,vbb,vcc);
			
			distance_+=velocidade/T_G_R*R*T; 
			fVel<<FIXED_FLOAT(n*T)<<"sec.;"<< " velocity clutch (rad/s): "<<velocidade<<"; T_G_R(total gear ratio): "<<T_G_R;
			 fVel<<";vehicle speed (km/h): "<<velocidade*R/T_G_R*3.6<<"; distance: "<<distance_<<" meters"<<endl;
			
//_____________

//_____________
		
	}	
	//fim_ciclo=true;
		
	
	}disable_raw_mode();
	tcflush(0, TCIFLUSH);
	
	return 0;

};

