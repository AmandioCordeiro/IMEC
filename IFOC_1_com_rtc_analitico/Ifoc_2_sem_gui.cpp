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
#include "filter_rtc.hpp"
#include "PEI.hpp"
#include <string>
#include <fstream>
#include <complex> 
//#include <mutex>

using namespace std;
using namespace Maths;

//#define per 0.0001  ///////specify sampling period if use filter_num_dif for rotor time constant correction(no use)

//#define np 2//3//2//number pole pairs
#define sqrt3_2 0.8660254038//TODO- julgo que nao foi utilizada
#define LOAD_MIN -193
#define LOAD_MAX 193
//#define T  0.0001 //period

extern ofstream PEI_txt;
ofstream T1T2 ("T1T2.txt");

bool flag=false;//TODO retirar pois julgo q nao foi utilizada

extern double TR;
extern double torq_g;
mutex m;
tThreePhase v(0.0,0.0,0.0);
bool quit=false;
float w_ref=0.0;
float load=0.0;
float pl=0.0;
bool p=true;
float imrreff=0.0;
double T0,T1,T2;//2-errado pois T1+T2 estava a dar > k T. 1-T0 indica o tempo por periodo k esta desligado
int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica
double pwm_a=0.0,pwm_b=0.0,pwm_c=0.0;
extern double VDC, Vmax;
extern long timer_Tr;
extern double IDC;
void svpwm(tTwoPhase v);


extern double vel_p;//1//9//1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO// 13marco
/*#define*/extern double vel_i;

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


template<typename Tf>
bool future_is_ready(std::future<Tf>& t){
    return t.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
bool mythread(Rs_Tr * ConstTr_) {
				    //do some stuff
			ConstTr_->Rs_Tr_do_it();
			double R=ConstTr_->get_Rs();double TT=ConstTr_->get_Tr();//Tr<<"TR::::"<<TT<<"RS"<<R<<endl;
					//};
			ofstream Tr_st ("Tr.txt",ios::app);
			Tr_st<<"ultimo Tr calculado: "<<TT/*Tr_calc_gl*/<<endl<<" Rs: "<<R<<endl;//TODO:Remove in the end
			Tr_st.close();
			
			return true;
}	

void svpwm(tTwoPhase v){//tTwoPhase- bifasico, tendo como referencia o estator
//??std::lock_guard<std::mutex> l(m);
	
	double ang=atan2(v.beta,v.alpha);
	cout<<"angulo"<<ang<<endl;
	T1T2<<"angulo: "<<ang<<endl;
	double ang_u = 0.0;
	//if (ang>=0&&ang<(PI/6))
	//	ang_u = ang - PI/6;//gives negative
	//else if (ang>=PI/6 && ang<(PI/3))
	//		ang_u = ang - PI/6;//gives positive
	if (ang>=0&&ang<(PI/3))
		ang_u = ang - PI/6;//cos positive number == cos negative number
	
	else if (ang>=PI/3 && ang<(2*PI/3))
			ang_u = ang - PI/2 ;
	//else if (ang>=PI/2 && ang<(2*PI/3))
		//	ang_u = ang - PI/2;
	
	else if (ang>=2*PI/3 && ang<(PI))
		//	ang_u = 5*PI/6 - ang;
	//else if (ang>= 5*PI/6 && ang<PI)
			ang_u = ang - 5*PI/6;
	
	
	else if (ang>=-PI && ang<(-2*PI/3))
			//ang_u = ang + 5*PI/6;//gives neg
	//else if (ang>= -5*PI/6 && ang<-2*PI/3)
			ang_u = ang + 5*PI/6;//gives positive	 	
	////////////
	else if (ang>=-2*PI/3 && ang<(-PI/3))
			//ang_u = 3*PI/2 - ang;
	//else if (ang>= 3*PI/2 && ang<5*PI/3)
			ang_u = ang + PI/2;
		
	else //if (ang>=5*PI/3 && ang<(11*PI/6))
			//ang_u = 11*PI/6 - ang;
	//else /*if (ang>= 11*PI/3 && ang<0)*/
			ang_u = ang + PI/6;
					
	float max_mod_v_ref = VDC/(cos(ang_u)*CONST_SQRT3_);
T1T2 << "max_mod_v_ref "<< max_mod_v_ref<<endl;
//max_mod_v_ref=abs(max_mod_v_ref);//abs para double

	if ( (sqrt(v.beta*v.beta+v.alpha*v.alpha)) > max_mod_v_ref ){
		v.beta  = sin(ang)*max_mod_v_ref;T1T2<< "v.beta "<<v.beta;
		v.alpha = cos(ang)*max_mod_v_ref;T1T2<<"v.alpha"<<v.alpha;
	}
	VDQ_alpha_esc=v.alpha;//sqrt(v.beta*v.beta+v.alpha*v.alpha)/2/3*VDC/*max_mod_v_ref*/*sin(ang);
	VDQ_beta_esc=v.beta;//sqrt(v.beta*v.beta+v.alpha*v.alpha)/2/3*VDC/*max_mod_v_ref*/*cos(ang);
	
	if (ang>=0&&ang<(PI/3)){
			T1=3.0/2.0*T/VDC*(v.alpha-v.beta/CONST_SQRT3_);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(2*VDC/3,-VDC/3,-VDC/3)).alpha)*T;
			a1=1;b1=0;c1=0;	//ozz 
			T2=CONST_SQRT3_*T/VDC*v.beta;//T2=3*T/(2*VDC)*(v.alpha-1/CONST_SQRT3_*v.beta);//T2=abs(v.beta)/abs((Clarke(tThreePhase(VDC/3,VDC/3,-2*VDC/3)).beta)*T);
			a2=1;b2=1;c2=0	;/*v_ooz*/
			
			if ((T1+T2)<T) pwm_a=(T1+T2)/T; else pwm_a=1;
			if ((T1+T2)<T) pwm_b=T2/T; else pwm_b=T2/(T1+T2);
			pwm_c=0;
	}
	else if(ang>=(PI/3)&&ang<(2.0*PI/3)){
			T1=3.0*T/2.0/VDC*(v.alpha+1/CONST_SQRT3_*v.beta);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(VDC/3,VDC/3,-2*VDC/3)).alpha)*T;
			a1=1;b1=1;c1=0;//ooz 
			T2=3*T/VDC/2*(v.beta/CONST_SQRT3_-v.alpha);
			a2=0;b2=1;c2=0;/*v_zoz*/
			
			if ((T1+T2)<T) pwm_a=T1/T; else pwm_a=T1/(T1+T2);
			if ((T1+T2)<T) pwm_b=(T1+T2)/T; else pwm_b=1;
			pwm_c=0;
	}
	else if(ang>=(2.0*PI/3)&&ang<(PI)){
			T1=T*CONST_SQRT3_/VDC*v.beta;//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-VDC/3,2*VDC/3,-VDC/3)).alpha)*T;
			a1=0;b1=1;c1=0;//zoz 
			T2=CONST_SQRT3_/2*T/VDC*(-v.beta-CONST_SQRT3_*v.alpha);//T2=abs(v.beta)/abs(Clarke(tThreePhase(-2*VDC/3,VDC/3,VDC/3)).beta)*T/*v_zoo*/;
			a2=0;b2=1;c2=1;
			
			pwm_a=0;
			if ((T1+T2)<T) pwm_b=(T1+T2)/T; else pwm_b=1;
			if ((T1+T2)<T) pwm_c=T2/T; else pwm_c=T2/(T1+T2);
	}
	else if(ang>=(-PI)&&ang<(-2.0*PI/3)){
			T1=3.0*T/2/VDC*(-v.alpha+v.beta/CONST_SQRT3_);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-2*VDC/3,VDC/3,VDC/3)).alpha)*T;
			a1=0;b1=1;c1=1;//zoo 
			T2=-CONST_SQRT3_*T/VDC*v.beta;//T2=abs(v.beta)/abs(Clarke(tThreePhase(-VDC/3,-VDC/3,2*VDC/3)).beta)*T/*v_zzo*/;
			a2=0;b2=0;c2=1;
			
			pwm_a=0;
			if ((T1+T2)<T) pwm_b=T1/T; else pwm_b=T1/(T1+T2);
			if ((T1+T2)<T) pwm_c=(T1+T2)/T; else pwm_c=1;			
	}
	else if(ang>=(-2*PI/3)&&ang<(-PI/3)){
			T1=CONST_SQRT3_*T/2/VDC*(-v.alpha*CONST_SQRT3_-/*2*/v.beta);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-VDC/3,-VDC/3,2*VDC/3)).alpha)*T;
			a1=0;b1=0;c1=1;//zzo 
			T2=CONST_SQRT3_*T/2/VDC*(-v.beta+v.alpha*CONST_SQRT3_);//T2=abs(v.beta)/abs(Clarke(tThreePhase(VDC/3,-2*VDC/3,VDC/3)).beta)*T/*v_ozo*/;
			a2=1;b2=0;c2=1;
			
			if ((T1+T2)<T) pwm_a=T2/T; else pwm_a=T2/(T1+T2);
			pwm_b=0;
			if ((T1+T2)<T) pwm_c=(T1+T2)/T; else pwm_c=1;
	}
	else /*if(ang>=(-PI/3)&&ang<(0))*/{
			T1=-1.0*T/VDC*CONST_SQRT3_*v.beta;//T1=abs(v.alpha)/abs(Clarke(tThreePhase(VDC/3,-2*VDC/3,VDC/3)).alpha)*T;
			a1=1;b1=0;c1=1;//ozo 
			T2=3.0*T/2.0/VDC*(v.beta/CONST_SQRT3_+v.alpha);//T2=abs(v.beta)/abs(Clarke(tThreePhase(2*VDC/3,-VDC/3,-VDC/3)).beta)*T/*v_ozz*/;
			a2=1;b2=0;c2=0;
			
			if ((T1+T2)<T) pwm_a=(T1+T2)/T; else pwm_a=1;
			pwm_b=0;
			if ((T1+T2)<T) pwm_c=T1/T; else pwm_c=T1/(T1+T2);		
	};
	 	
			
	 T0=T-(T1+T2);//T0 time of period that are off 1, 1, 1 or 0, 0, 0
	//doc.: para cada periodo T: T1 de tempo com os respectivos a1, b1, c1; T2 de tempo com os respectivos a2, b2, c2. O interruptor correspondente há outra parte da "perna" será o complementar;
	//TIMERS
	 //a1,T1; a2,T2 	;a0,T0;   a off
	 //b1,T1; b2,T2 	;b0,T0;   and b off
	 //c1,T1 ;c2,T2 	;c0,T0;   and c off
	//ofstream T1T2 ("T1T2.txt",ios::app); 
	cout<<" T1:"<<T1<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<T2<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<T0<<endl;
	cout<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;
	T1T2<<" T1:"<<T1<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<T2<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<T0<<endl;
	T1T2<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;
v_=v;

};

int main(int argc, char **argv)
{
	
	control_loops control;
	
	bool run=false;
	
	bool fim_calc_tr=true;
	Rs_Tr *ConstTr=nullptr;
	double vec_MediaTr_calc_gl[6];
	for (int i=0;i<6;i++)vec_MediaTr_calc_gl[i]=Tr_calc_gl;
	
	filter filtro;
	
	long c=0;
	std::future<bool> foo;
	
	char o;
	
	double v_med_Tr_calc[16]={0.0};
	double v_med_T[16]={0.0};
		
	const auto timeWindow = std::chrono::microseconds(100);
	
	enable_raw_mode();
	
while (quit==false){
		auto start = std::chrono::steady_clock::now();
		
	//std::lock_guard<std::mutex> l(m);
	
		if (/*o=getch()*/kbhit()/*||flag==true*/)
		{
			o=getchar();
			switch(o){
				//std::lock_guard<std::mutex> l(m);
				case '\n':break;	
				case 'r' :run=true;/*flag=false*/;break;
				case 'q' :quit=true;/*flag=false*/;break;
				//case 'm' :cout<<"enter magnetizing current reference:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	//tcflush(0, TCIFLUSH);cin>>imrreff;enable_raw_mode();/*fflush(stdin);*/break;
				case 's' :cout<<"enter speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>w_ref;enable_raw_mode();/*fflush(stdin);*/break;
				case 'l' :cout<<"enter load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>load;enable_raw_mode();/*fflush(stdin);*/break; 
				case 'a' :cout<<"enter prop. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'b' :cout<<"enter int. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 'c' :cout<<"enter prop. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'd' :cout<<"enter int. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_i;enable_raw_mode();/*fflush(stdin);*/break;
				
				case '+' :w_ref++;/*flag=false;*/break;
				case '-' :w_ref--;/*flag=false;*/break;
				case 'k' :load--;/*flag=false;*/break;
				case 'p' :load++;/*flag=false;*/break;
				case 't' :cout<<"seted by torque:";p=false;break;
				case 'h' :cout<<"enter power load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>pl;enable_raw_mode();/*fflush(stdin);*/break;
				case 'j' :pl--;/*flag=false;*/break;
				case 'i' :pl++;/*flag=false;*/break;
	//case 'return': break;//pseudocode
				default:cout<<"invalid operator, s(commanded speed),l(load);+(increment load),- (decrement load), k(decrement load),p(increment load),r(run),q(quit)"<<endl;	
			}
		
		}
		
		else if (run==true){
			//DEfenition of torque load in function of power specified
			if (p && control.get_wr()!=0) {
			  load=pl/abs(control.get_wr());
			  if (load<LOAD_MIN)load=LOAD_MIN;
			  if (load>LOAD_MAX) load=LOAD_MAX;}
			else if (p){
					if (load<0)load=LOAD_MIN;
					if (load>0) load=LOAD_MAX;
					}
			//TODO remove at end
			control.vel_tune_pid();//TODO remove at END
			control.torque_control_tune_pid();
			T1T2<<control.get_n()*T<<" sec. "<<endl;
			fwref<<control.get_n()*T<<"sec. ref. velocity: "<<w_ref<<endl;
			fload<<control.get_n()*T<<"sec. Torque load(ref.): "<<control.get_torq_L()<<endl;
			//fim_ciclo=false;
			
			control.set_w_ref(w_ref);
			//control.set_imrref(imrreff);//TODO remove this, LMA!...
			control.set_torq_L(load);//TODO this is only in simulation, not needed in real
			control.set_Theta_r();
			control.get_VDC(312.0/*343.0*/);//TODO adapt in real.function must read DC voltage			
			//TODO in real necessary read abc_voltage, see control_motor.cpp:170. i guess that the last set value of inverter isnt enough
			//TODO in real the following reads from e Hall sensors
			control.get_ias();/*cout<<"ia"<<abc_current.a<<endl;*/control.get_ibs();control.get_ics();//abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor, is better get abc_current.c from measure
			
			//função a seguir com tempo variavel, necessario resolver para tempo const
			svpwm(control.get_V());//defenidos que gates e o tempo de disparos //2-julgo k ta errado-> 1-TODO necessario centrar os disparos a meio do tpwm
			//erraddo sem fundamento-V*Tempo;
			
/////____			
			
			if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
			{	//v_.alpha=2/3*VDC*T1/T+1/3*VDC*T2/T;
				//v_.beta=1/CONST_SQRT3_*VDC*T2;
				//vaa=T1/T*2*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3+T2/T*VDC/3;vcc=-T1/T*VDC/3-T2/T*2*VDC/3;
				//vaa=2/3*VDC*pwm_a;
				//vbb
				IDC = control.get_ias()*T1/T - control.get_ics()*T2/T;}
			else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
			{   //v_.alpha=1/3*VDC*T1/T-1/3*VDC*T2/T;
				//v_.beta=VDC/CONST_SQRT3_*T1+1/CONST_SQRT3_*VDC*T2;
				//vaa=T1/T*VDC/3-T2/T*VDC/3;vbb=T1/T*VDC/3+T2/T*2*VDC/3;vcc=-2*T1/T*VDC/3-T2/T*VDC/3;
				IDC = - control.get_ics()*T1/T+control.get_ibs()*T2/T;}
			else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
			{	//v_.alpha=-1/3*VDC*T1/T-2/3*VDC*T2/T;
				//v_.beta=VDC/CONST_SQRT3_*T1/T+0.0*T2/T;
				//vaa=-T1/T*VDC/3-2*T2/T*VDC/3;vbb=T1/T*VDC*2/3+T2/T*VDC/3;vcc=-T1/T*VDC/3+T2/T*VDC/3;
				IDC = control.get_ibs()*T1/T - control.get_ias()*T2/T;}
			else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
			{	//v_.alpha=-2/3*VDC*T1/T-1/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T2/T+0.0*T1/T;
				//vaa=-T1/T*VDC*2/3-T2/T*VDC/3;vbb=T1/T*VDC/3-T2/T*VDC/3;vcc=T1/T*VDC/3+T2/T*VDC*2/3;
				IDC = - control.get_ias()*T1/T+control.get_ics()*T2/T;}
			else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
			{	//v_.alpha=-1/3*VDC*T1/T+1/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T1/T-VDC/CONST_SQRT3_*T2/T;
				//vaa=-T1/T*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3-T2/T*VDC*2/3;vcc=T1/T*VDC*2/3+T2/T*VDC/3;
				IDC = control.get_ics()*T1/T - control.get_ibs()*T2/T;}
			else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
			{	//v_.alpha=1/3*VDC*T1/T+2/3*VDC*T2/T;
				//v_.beta=-VDC/CONST_SQRT3_*T1/T-0.0*T2/T;			
				//vaa=T1/T*VDC/3+2*T2/T*VDC/3;vbb=-T1/T*VDC*2/3-T2/T*VDC/3;vcc=T1/T*VDC/3-T2/T*VDC/3;
				IDC = - control.get_ibs()*T1/T+control.get_ias()*T2/T;}
/////____
			
			vaa=InvClarke(v_).a;	
			vbb=InvClarke(v_).b;
			vcc=InvClarke(v_).c;
			
			cout<<" vaa:"<<vaa<<" vbb:"<<vbb<<" vcc:"<<vcc<<endl;
			T1T2<<" vaa:"<<vaa<<" vbb:"<<vbb<<" vcc:"<<vcc<<endl<<"IDC: "<<IDC<<endl;
			
			control.get_wr(vaa,vbb,vcc); velocidade=control.get_wr(); fVel<<control.get_n()*T<<"sec. actual velocity: "<<velocidade<<endl;
//_____________
{
					if(fim_calc_tr){
						if (ConstTr==nullptr){
							ConstTr=new Rs_Tr(/*m_Tr_calc_gl*/);
							c=0;
						};
						c++;
						//TODO: at real need this.  filtro.fil_sampl((control.IDQ_rotor_d()),/*(float)*/ (control.IDQ_rotor_q()),(control.VDQ_rotor_d()),(control.VDQ_rotor_q()),(control.get_wr()));
						ConstTr->enter_medition(/*filtro.filtred_isx()*/(control.IDQ_rotor_d()),/*filtro.filtred_isy()*/(control.IDQ_rotor_q()),/*filtro.filtred_usx()*/ (control.VDQ_rotor_d()),/*filtro.filtred_usy()*/ (control.VDQ_rotor_q()), (control.get_wr())/*filtro.filtred_Theta()*/);				
					}
					else/*if(!fim_calc_tr)*/{	
						if (future_is_ready(foo)){
								fim_calc_tr=true;
								if(ConstTr){delete ConstTr;ConstTr=nullptr;};
								ofstream Tr_s("Tr.txt",ios::app);//TODO remove in embedded								
								
								//para calc valor med de Tr calculado:
								if(Tr_calc_gl!=vec_MediaTr_calc_gl[0]){
									for(int i=4 ; i>=0 ; i--)vec_MediaTr_calc_gl[i+1]=vec_MediaTr_calc_gl[i]; 
									vec_MediaTr_calc_gl[0]=Tr_calc_gl;};
								double mm_Tr_calc_gl=0.0;
								for(int i=0;i<6;i++)mm_Tr_calc_gl+=vec_MediaTr_calc_gl[i];
								double inic_m_Tr_calc_gl=mm_Tr_calc_gl/=6; 
								Tr_s<<control.get_n()*T<<" sec. inic_m_Tr_calc_gl:"<<inic_m_Tr_calc_gl<<endl;//TODO:remove in embedded
								
								//for(unsigned i=15;i>0;--i){v_med_Tr_calc[i-1]=v_med_Tr_calc[i];}
								//for(unsigned i=15;i>0;--i){v_med_T[i-1]=v_med_T[i];}
								//v_med_Tr_calc[15]=Tr_calc_gl*(timer_Tr*T);
								//v_med_T[15]=timer_Tr*T;
								//double val_Tr_calc_gl=0.0;
								//for(unsigned i=0;i<16;++i){val_Tr_calc_gl+=v_med_Tr_calc[i];}
								//double m_T=0.0;
								//for(unsigned i=0;i<16;++i){m_T+=v_med_T[i];}
								//m_Tr_calc_gl=val_Tr_calc_gl/m_T;
								timer_Tr=0;
								
								Tr_s.close();//TODO:remove in embedded
						}
					}
					if(c==3000/*100*/ && ConstTr!=nullptr){		//TODO remove second part		
						//if(fim_calc_tr){
							//TODO:incluir try, throw catch
							foo= std::async(std::launch::async, mythread, /*std::move(*/ConstTr/*)*/);
							fim_calc_tr=false;
							c++;
					}
					
				};
//_____________
		
	}	
	//fim_ciclo=true;
		
	auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    
   auto timeToWait = timeWindow - elapsed;
   if(timeToWait > std::chrono::microseconds::zero())
    {
        std::this_thread::sleep_for(timeToWait);
    }
	}disable_raw_mode();
	tcflush(0, TCIFLUSH);
	
	return 0;

};
