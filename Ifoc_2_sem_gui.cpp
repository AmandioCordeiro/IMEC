/*
 * Ifoc.cpp
 * 
 * Copyright 2016 amrc <amandio.miguel@gmail.com>
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


//#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
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
#include "control_motor.cpp"
//#include "filter_rtc.hpp"
#include <string>
#include <string.h>
#include <fstream>
#include <complex> 


using namespace std;
using namespace Maths;


#define LOAD_MIN -154
#define LOAD_MAX 154

extern ofstream PEI_txt;



extern double torq_g;

bool quit=false;
float w_ref=0.0;
float load=0.0, load_s=0.0;
float pl=0.0;
bool p=true;
float imrreff=0.0;
extern double VDC, Vmax;
//extern long timer_Tr;
extern double IDC;
//void svpwm(tTwoPhase v);
double distance_=0.0;


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
	
	control_loops control;
	
	bool run=false;
	
	//bool fim_calc_tr=true;
	//Rs_Tr *ConstTr=nullptr;
	//double vec_MediaTr_calc_gl[6];
	//for (int i=0;i<6;i++)vec_MediaTr_calc_gl[i]=Tr_calc_gl;
	
	//filter filtro;
	
	long c=0;
	
	char o;
	
	//double v_med_Tr_calc[16]={0.0};
	//double v_med_T[16]={0.0};
		
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
	tcflush(0, TCIFLUSH);cin>>load_s;enable_raw_mode();p=false;/*fflush(stdin);*/break; 
				case 'g' :cout<<"enter gear:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>gear;if(gear==0)T_G_R =0; if(gear==1)T_G_R = (4.313/* 2.331.436/* 1 0.789*/*4.1/*1.9*/);if(gear==2)T_G_R = (/*4.313*/ 2.33/*1.436 1 0.789*/*4.1/*1.9*/);if(gear==3)T_G_R = (/*4.313 2.33*/1.436/* 1 0.789*/*4.1/*1.9*/);if(gear==4)T_G_R = (/*4.313 2.33 1.436*/ 1 /*0.789*/*4.1/*1.9*/);enable_raw_mode();/*fflush(stdin);*/break; 
				case 'a' :cout<<"enter prop. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'b' :cout<<"enter int. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 'c' :cout<<"enter prop. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'd' :cout<<"enter int. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_i;enable_raw_mode();/*fflush(stdin);*/break;
				case ' ' :cout<<"pause, press r to return";run=false;break;//disable_raw_mode();
	//tcflush(0, TCIFLUSH);char f='p';while(f != ' '){cin>>f;cout<<f;};enable_raw_mode();/*fflush(stdin);*/break;
				case '+' :w_ref++;/*flag=false;*/break;
				case '-' :w_ref--;/*flag=false;*/break;
				//case 'k' :load--;/*flag=false;*/break;
				//case 'p' :load++;/*flag=false;*/break;
				//case 't' :cout<<"set by torque load ";p=false;break;
				case 'v' :cout<<"set by vehicle load ";p=true;break;//inicial p=true
				//case 'h' :cout<<"enter power load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	//tcflush(0, TCIFLUSH);cin>>pl;enable_raw_mode();/*fflush(stdin);*/break;
				case 'j' :pl--;/*flag=false;*/break;
				case 'i' :pl++;/*flag=false;*/break;
				//case 'e' : e=true;break;//set by driving_cycle
				
				default:cout<<"invalid operator, \ns(commanded speed)"<<" \na (enter prop. gain speed)"<<" \nb (enter int. gain speed)"<<"\nc (enter prop. gain torq_controll)"<<"\nd (enter int. gain torq_controll)"<<" \nl(load), "<</*\nt(set by torque load), */"\nv(set by vehicle load), \n+(increment speed), \n-(decrement speed)"<</*, k(decrement load),p(increment load),*/", \nr(run), \nq(quit)"<<endl;	
			}
		
		}
		
		else if (run==true){
			//DEfinition of torque load in function of power specified
		//TODO remove following at end
			if (control.get_n()*T<0.135)load=LOAD_1_sec*0.9;
			else if (p && control.get_wr()!=0) {
				double a=atan(gradiente);	double load_C_R;if (velocidade>0)load_C_R=C_R*MASSA*G*cos(a);else if(velocidade<0)load_C_R = -C_R*MASSA*G*cos(a);else if (velocidade==0)load_C_R=0;
				load=(C_D*1/2*RO_AIR*A_F*/*wr*/velocidade/T_G_R*R*/*wr*/velocidade/T_G_R*R+load_C_R+MASSA*G*sin(a))*R/T_G_R;//torque resistant vehicle
			//  load=pl/abs(control.get_wr());
			 // if (load<LOAD_MIN)load=LOAD_MIN;
			 // if (load>LOAD_MAX) load=LOAD_MAX;}
			//else if (p){
				//	if (load<0)load=LOAD_MIN;
				//	if (load>0) load=LOAD_MAX;
					}
			else load=load_s;
		//TODO remove above at end
			control.vel_tune_pid();//TODO remove at END, in embedded
			control.torque_control_tune_pid();
			
			T1T2<<control.get_n()*T<<" sec. "<<endl;
			fwref<<control.get_n()*T<<"sec. ref. velocity: "<<w_ref<<endl;
			fload<<control.get_n()*T<<"sec. Torque load(ref.): "<<control.get_torq_L()<<endl;
			//fim_ciclo=false;
			
						
			//four urban drive cycle, ece 15*4
			if (i<4 && e==true && driving_cycle.is_open())
			{				
						if (primeiro){getline(driving_cycle,line);primeiro=false;}
						fVel<<"  line   "<<line<<endl;
						while(line.length() > 13 )getline(driving_cycle,line);	
						
						// char szOrbits[] = "365.24 29.53";
						
						/*vector<char> toVector( const std::string& s ) {
							string s = "apple";  
							vector<char> v(s.size()+1);
							memcpy( &v.front(), s.c_str(), s.size() + 1 );
							return v;
						}
						vector<char> v = toVector(std::string("apple"));
*/
						//// what you were looking for (mutable)
						//char* c = v.data();
						vector<char> v(line.size()+1);
						memcpy( &v.front(), line.c_str(), line.size() + 1 );
						
						char* s_line=v.data();
						char* pEnd;
						//double d1, d2;
						double b, end;
						/*d1*/b = strtod (s_line/*szOrbits*/, &pEnd);
						/*d2*/end = strtod (pEnd, NULL);
						fVel<<" (b+196*i): "<<(b+196*i)/*<<" b e end :"<<b<<end*/<<endl;
						
						if (b==195 && (control.get_n()*T) == (b+196*i)){
							driving_cycle.close();
							driving_cycle.open("ececol.txt",std::ifstream::in);
							i++;
							primeiro =true;
							w_ref=end*1000/3600*T_G_R/R;
							}
						else if ((control.get_n()*T) == (b+196*i)) 	
						{	w_ref=end*1000/3600*T_G_R/R;
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
				fVel<<"  line   "<<line<<endl;
				while(line.length() > 13 )getline(driving_cycle,line);	
				
				// char szOrbits[] = "365.24 29.53";
				
				/*vector<char> toVector( const std::string& s ) {
					string s = "apple";  
					vector<char> v(s.size()+1);
					memcpy( &v.front(), s.c_str(), s.size() + 1 );
					return v;
				}
				vector<char> v = toVector(std::string("apple"));
*/
				//// what you were looking for (mutable)
				//char* c = v.data();
				vector<char> v(line.size()+1);
				memcpy( &v.front(), line.c_str(), line.size() + 1 );
				
				char* s_line=v.data();
				char* pEnd;
				//double d1, d2;
				double b, end;
				/*d1*/b = strtod (s_line/*szOrbits*/, &pEnd);
				/*d2*/end = strtod (pEnd, NULL);
				if ((control.get_n()*T) == /*b*/(b+196*i)) 	//here
				{	w_ref=end*1000/3600*T_G_R/R;
					getline(driving_cycle,line);
				
				}
				if (b==400)quit=true;
				}	
				
			control.set_w_ref(w_ref);
			
			//control.set_imrref(imrreff);//TODO remove this, LMA!...
			control.set_torq_L(load);//TODO this is only in simulation, not needed in real
			control.set_Theta_r();
			control.get_VDC(300.0/*343.0*/);//TODO adapt in real.function must read DC voltage			
			//TODO in real the following reads from e Hall sensors			
			control.get_ias();/*cout<<"ia"<<abc_current.a<<endl;*/control.get_ibs();control.get_ics();//abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor, is better get abc_current.c from measure
			
/////____	
			//to measure previous current DC:
			
			if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
			{	//v_.alpha=2/3*VDC*T1/T+1/3*VDC*T2/T;
				//v_.beta=1/CONST_SQRT3_*VDC*T2;
				//vaa=T1/T*2*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3+T2/T*VDC/3;vcc=-T1/T*VDC/3-T2/T*2*VDC/3;
				//vaa=2/3*VDC*pwm_a;
				//vbb
				IDC = control.get_ias()/*or abc_current.a*/*T1/T - control.get_ics()*T2/T;
				////IDC2_3=control.get_ias()*2/3 - control.get_ics()*2/3;
				}
			else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
			{   //v_.alpha=1/3*VDC*T1/T-1/3*VDC*T2/T;
				//v_.beta=VDC/CONST_SQRT3_*T1+1/CONST_SQRT3_*VDC*T2;
				//vaa=T1/T*VDC/3-T2/T*VDC/3;vbb=T1/T*VDC/3+T2/T*2*VDC/3;vcc=-2*T1/T*VDC/3-T2/T*VDC/3;
				IDC = - control.get_ics()*T1/T+control.get_ibs()*T2/T;
				////IDC2_3 = - control.get_ics()*2/3+control.get_ibs()*2/3;
				}
			else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
			{	//v_.alpha=-1/3*VDC*T1/T-2/3*VDC*T2/T;
				//v_.beta=VDC/CONST_SQRT3_*T1/T+0.0*T2/T;
				//vaa=-T1/T*VDC/3-2*T2/T*VDC/3;vbb=T1/T*VDC*2/3+T2/T*VDC/3;vcc=-T1/T*VDC/3+T2/T*VDC/3;
				IDC = control.get_ibs()*T1/T - control.get_ias()*T2/T;
				////IDC2_3 = control.get_ibs()*2/3 - control.get_ias()*2/3;
				}
			else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
			{	//v_.alpha=-2/3*VDC*T1/T-1/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T2/T+0.0*T1/T;
				//vaa=-T1/T*VDC*2/3-T2/T*VDC/3;vbb=T1/T*VDC/3-T2/T*VDC/3;vcc=T1/T*VDC/3+T2/T*VDC*2/3;
				IDC = - control.get_ias()*T1/T+control.get_ics()*T2/T;
				////IDC2_3 = - control.get_ias()*2/3+control.get_ics()*2/3;
				}
			else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
			{	//v_.alpha=-1/3*VDC*T1/T+1/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T1/T-VDC/CONST_SQRT3_*T2/T;
				//vaa=-T1/T*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3-T2/T*VDC*2/3;vcc=T1/T*VDC*2/3+T2/T*VDC/3;
				IDC = control.get_ics()*T1/T - control.get_ibs()*T2/T;
				////IDC2_3 = control.get_ics()*2/3 - control.get_ibs()*2/3;
				}
			else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
			{	//v_.alpha=1/3*VDC*T1/T+2/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T1/T-0.0*T2/T;			
				//vaa=T1/T*VDC/3+2*T2/T*VDC/3;vbb=-T1/T*VDC*2/3-T2/T*VDC/3;vcc=T1/T*VDC/3-T2/T*VDC/3;
				IDC = - control.get_ibs()*T1/T+control.get_ias()*T2/T;
				////IDC2_3 = - control.get_ibs()*2/3+control.get_ias()*2/3;
				}

			double ia=control.get_ias();//TODO to measure cos phi
			//função a seguir com tempo variavel, necessario resolver para tempo const
			svpwm(control.get_V(),control.get_rot_fl_an(),control);//news gates and times
			/*here*/
			iaa_p=ia;
			vaa_p=vaa;
			
			//tension that will be applied next
			if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
			{	//v_.alpha=2/3*VDC*T1/T+1/3*VDC*T2/T;
				//v_.beta=1/CONST_SQRT3_*VDC*T2;
				vaa=T1/T*2*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3+T2/T*VDC/3;vcc=-T1/T*VDC/3-T2/T*2*VDC/3;
				//IDC = control.get_ias()*T1/T - control.get_ics()*T2/T;
				//IDC2_3=control.get_ias()*2/3 - control.get_ics()*2/3;
				}
			else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
			{   //v_.alpha=1/3*VDC*T1/T-1/3*VDC*T2/T;
				//v_.beta=VDC/CONST_SQRT3_*T1+1/CONST_SQRT3_*VDC*T2;
				vaa=T1/T*VDC/3-T2/T*VDC/3;vbb=T1/T*VDC/3+T2/T*2*VDC/3;vcc=-2*T1/T*VDC/3-T2/T*VDC/3;
				//IDC = - control.get_ics()*T1/T+control.get_ibs()*T2/T;
				//IDC2_3 = - control.get_ics()*2/3+control.get_ibs()*2/3;
				}
			else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
			{	//v_.alpha=-1/3*VDC*T1/T-2/3*VDC*T2/T;
				//v_.beta=VDC/CONST_SQRT3_*T1/T+0.0*T2/T;
				vaa=-T1/T*VDC/3-2*T2/T*VDC/3;vbb=T1/T*VDC*2/3+T2/T*VDC/3;vcc=-T1/T*VDC/3+T2/T*VDC/3;
				//IDC = control.get_ibs()*T1/T - control.get_ias()*T2/T;
				//IDC2_3 = control.get_ibs()*2/3 - control.get_ias()*2/3;
				}
			else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
			{	//v_.alpha=-2/3*VDC*T1/T-1/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T2/T+0.0*T1/T;
				vaa=-T1/T*VDC*2/3-T2/T*VDC/3;vbb=T1/T*VDC/3-T2/T*VDC/3;vcc=T1/T*VDC/3+T2/T*VDC*2/3;
				//IDC = - control.get_ias()*T1/T+control.get_ics()*T2/T;
				//IDC2_3 = - control.get_ias()*2/3+control.get_ics()*2/3;
				}
			else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
			{	//v_.alpha=-1/3*VDC*T1/T+1/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T1/T-VDC/CONST_SQRT3_*T2/T;
				vaa=-T1/T*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3-T2/T*VDC*2/3;vcc=T1/T*VDC*2/3+T2/T*VDC/3;
				//IDC = control.get_ics()*T1/T - control.get_ibs()*T2/T;
				//IDC2_3 = control.get_ics()*2/3 - control.get_ibs()*2/3;
				}
			else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
			{	//v_.alpha=1/3*VDC*T1/T+2/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T1/T-0.0*T2/T;			
				vaa=T1/T*VDC/3+2*T2/T*VDC/3;vbb=-T1/T*VDC*2/3-T2/T*VDC/3;vcc=T1/T*VDC/3-T2/T*VDC/3;
				//IDC = - control.get_ibs()*T1/T+control.get_ias()*T2/T;
				//IDC2_3 = - control.get_ibs()*2/3+control.get_ias()*2/3;
				}

			
/////____
			
	
			//vaa=InvClarke(v_).a;	
			//vbb=InvClarke(v_).b;
			//vcc=InvClarke(v_).c;
			
			//cout<<" vaa:"<<vaa<<" vbb:"<<vbb<<" vcc:"<<vcc<<endl;
			T1T2<<" vaa_n:"<<vaa<<" vbb_n:"<<vbb<<" vcc_n:"<<vcc<<endl<<"IDC_p: "<<IDC<<endl;
			
			//menos efficiente com isto!?
			//if (w_ref==0 && abs(velocidade) < 1)/*T0==T*/{T0=T;vaa=0.0;vbb=0.0;vcc=0.0;}
			
			control.get_wr(vaa,vbb,vcc); velocidade=control.get_wr(); distance_+=velocidade/T_G_R*R*T; fVel<<control.get_n()*T<<"sec. actual velocity, shaft (rad/s): "<<velocidade<<"T_G_R"<<T_G_R<<" km/h: "<<velocidade*R/T_G_R*3.6<<"distance: "<<distance_<<" meters"<<endl;
//_____________

//_____________
		
	}	
	//fim_ciclo=true;
		
	
	}disable_raw_mode();
	tcflush(0, TCIFLUSH);
	
	return 0;

};
