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


#include <iostream>
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
#include <mutex>

//#include <stdlib.h>
//#include <ncurses.h>
using namespace std;
using namespace Maths;

#define per 0.0001  ///////specify sampling period if use filter_num_dif for rotor time constant correction(no use)

#define np 2//3//2//number pole pairs
#define sqrt3_2 0.8660254038//TODO- julgo que nao foi utilizada

#define T  0.0001 //period

//extern ofstream Tr_s;
extern ofstream PEI_txt;

bool flag=false;//TODO retirar pois julgo q nao foi utilizada

extern double TR;
extern double torq_g;
mutex m;
double velocidade=0;
tThreePhase v(0,0,0);
bool quit=false;
float w_ref=0;
float load=0;
float imrreff=0.0;
float /*T0,*/T1,T2;//T0 apenas indica o tempo por periodo k esta desligado
int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica
float VDC=330;//need actualization in pratice
void svpwm(tTwoPhase v, float VDC);

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
			Tr_st<<"ultimo Tr calculado: "<<TT/*Tr_calc_gl*/<<endl;//TODO:Remove in the end
			Tr_st.close();
			
			return true;
}	

void svpwm(tTwoPhase v, float VDC){//tTwoPhase- bifasico, tendo como referencia o estator
//??std::lock_guard<std::mutex> l(m);
	double ang=atan2(v.beta,v.alpha);
	cout<<"angulo"<<ang<<endl;

	if (ang>=0&&ang<(PI/3)){
			T1=3/2*T/VDC*(v.alpha-v.beta/CONST_SQRT3_);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(2*VDC/3,-VDC/3,-VDC/3)).alpha)*T;
			a1=1;b1=0;c1=0;	//ozz 
			T2=CONST_SQRT3_*T/VDC*v.beta;//T2=3*T/(2*VDC)*(v.alpha-1/CONST_SQRT3_*v.beta);//T2=abs(v.beta)/abs((Clarke(tThreePhase(VDC/3,VDC/3,-2*VDC/3)).beta)*T);
			a2=1;b2=1;c2=0	;/*v_ooz*/

	}
	else if(ang>=(PI/3)&&ang<(2*PI/3)){
			T1=3*T/2/VDC*(v.alpha+1/CONST_SQRT3_*v.beta);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(VDC/3,VDC/3,-2*VDC/3)).alpha)*T;
			a1=1;b1=1;c1=0;//ooz 
			T2=3*T/VDC/2*(v.beta/CONST_SQRT3_-v.alpha);
			a2=0;b2=1;c2=0;/*v_zoz*/

	}
	else if(ang>=(2*PI/3)&&ang<(PI)){
			T1=T*CONST_SQRT3_/VDC*v.beta;//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-VDC/3,2*VDC/3,-VDC/3)).alpha)*T;
			a1=0;b1=1;c1=0;//zoz 
			T2=CONST_SQRT3_/2*T/VDC*(-v.beta-CONST_SQRT3_*v.alpha);//T2=abs(v.beta)/abs(Clarke(tThreePhase(-2*VDC/3,VDC/3,VDC/3)).beta)*T/*v_zoo*/;
			a2=0;b2=1;c2=1;

	}
	else if(ang>=(-PI)&&ang<(-2*PI/3)){
			T1=3*T/2/VDC*(-v.alpha+v.beta/CONST_SQRT3_);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-2*VDC/3,VDC/3,VDC/3)).alpha)*T;
			a1=0;b1=1;c1=1;//zoo 
			T2=-CONST_SQRT3_*T/VDC*v.beta;//T2=abs(v.beta)/abs(Clarke(tThreePhase(-VDC/3,-VDC/3,2*VDC/3)).beta)*T/*v_zzo*/;
			a2=0;b2=0;c2=1;
	}
	else if(ang>=(-2*PI/3)&&ang<(-PI/3)){
			T1=CONST_SQRT3_*T/2/VDC*(-v.alpha*CONST_SQRT3_-/*2*/v.beta);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-VDC/3,-VDC/3,2*VDC/3)).alpha)*T;

			a1=0;b1=0;c1=1;//zzo 
			T2=CONST_SQRT3_*T/2/VDC*(-v.beta+v.alpha*CONST_SQRT3_);//T2=abs(v.beta)/abs(Clarke(tThreePhase(VDC/3,-2*VDC/3,VDC/3)).beta)*T/*v_ozo*/;
			a2=1;b2=0;c2=1;
	}
	else if(ang>=(-PI/3)&&ang<=(0)){
			T1=-T/VDC*CONST_SQRT3_*v.beta;//T1=abs(v.alpha)/abs(Clarke(tThreePhase(VDC/3,-2*VDC/3,VDC/3)).alpha)*T;
			a1=1;b1=0;c1=1;//ozo 
			T2=3*T/2/VDC*(v.beta/CONST_SQRT3_+v.alpha);//T2=abs(v.beta)/abs(Clarke(tThreePhase(2*VDC/3,-VDC/3,-VDC/3)).beta)*T/*v_ozz*/;
			a2=1;b2=0;c2=0;
			
	};
	//Julgo nao ser necessario!! T0=T-T1-T2;
	//doc.: para cada periodo T: T1 de tempo com os respectivos a1, b1, c1; T2 de tempo com os respectivos a2, b2, c2. O interruptor correspondente há outra parte da "perna" será o complementar; assim será melhor introduzir tempos neutros para evitar estarem os dois ligados provocando curto circuito
	//TIMERS
	 //a1,T1; a2,T2 sem fundamneto este T0, apenas indica o tempo em que estao todos desligados, por Periodo//a1,T0;
	 //b1,T1; b2,T2 	//b1,T0;
	 //c1,T1 ;c2,T2 	//c1,T0;
	cout/*<<" T0:"<<T0*/<<" T1:"<<T1<<" T2:"<<T2<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<endl;
};

int main(int argc, char **argv)
{
	double vaa,vbb,vcc;
	control_loops control;
	
	bool run=false;
	
	bool fim_calc_tr=true;
	Rs_Tr *ConstTr=nullptr;
	//long num=0;
	filter filtro;
	
	long c=0;
	std::future<bool> foo;
	
	char o;
	
	double v_med_Tr_calc[16]={0};
	double v_med_T[16]={0};
	double T_last=0.0;
	double T_act=0.0;
		
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
	tcflush(0, TCIFLUSH);cin>>imrreff;enable_raw_mode();/*fflush(stdin);*/break;
				case 's' :cout<<"enter speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>w_ref;enable_raw_mode();/*fflush(stdin);*/break;
				case 'l' :cout<<"enter load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>load;enable_raw_mode();/*fflush(stdin);*/break; 
				case '+' :w_ref++;/*flag=false;*/break;
				case '-' :w_ref--;/*flag=false;*/break;
				case 'k' :load--;/*flag=false;*/break;
				case 'p' :load++;/*flag=false;*/break;
	//case 'return': break;//pseudocode
				default:cout<<"invalid operator, s(commanded speed),l(load);+(increment load),- (decrement load), k(decrement load),p(increment load),r(run),q(quit)"<<endl;	
			}
		
		}
		
		else if (run==true){
			fwref<<control.get_n()*T<<"sec. Vel. de referencia: "<<w_ref<<endl;
			fload<<control.get_n()*T<<"sec. Torque de carga(de referencia): "<<control.get_torq_L()<<endl;
			fim_ciclo=false;
			control.set_w_ref(w_ref);
			//control.set_imrref(imrreff);
			control.set_torq_L(load);
			control.set_Theta_r();
			//actualize VDC()
			
			//função a seguir com tempo variavel, necessario resolver para tempo const
			svpwm(control.get_V(),VDC);//defenidos que gates e o tempo de disparos //2-julgo k ta errado-> 1-TODO necessario centrar os disparos a meio do tpwm
			//erraddo sem fundamento-V*Tempo;
			if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
			{	vaa=T1/T*2*VDC/3+T2/T*VDC/3;
				vbb=-T1/T*VDC/3+T2/T*VDC/3;
				vcc=-T1/T*VDC/3-T2/T*2*VDC/3;
			}
			else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
			{vaa=T1/T*VDC/3-T2/T*VDC/3;vbb=T1/T*VDC/3+T2/T*2*VDC/3;vcc=-2*T1/T*VDC/3-T2/T*VDC/3;}
			else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
			{vaa=-T1/T*VDC/3-2*T2/T*VDC/3;vbb=T1/T*VDC*2/3+T2/T*VDC/3;vcc=-T1/T*VDC/3+T2/T*VDC/3;}
			else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
			{vaa=-T1/T*VDC*2/3-T2/T*VDC/3;vbb=T1/T*VDC/3-T2/T*VDC/3;vcc=T1/T*VDC/3+T2/T*VDC*2/3;}
			else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
			{vaa=-T1/T*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3-T2/T*VDC*2/3;vcc=T1/T*VDC*2/3+T2/T*VDC/3;}
			else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
			{vaa=T1/T*VDC/3+2*T2/T*VDC/3;vbb=-T1/T*VDC*2/3-T2/T*VDC/3;vcc=T1/T*VDC/3-T2/T*VDC/3;}
			
			
			control.get_wr(vaa,vbb,vcc);velocidade=control.get_wr();fVel<<control.get_n()*T<<"sec. velocidade actual: "<<velocidade<<endl;
//_____________
{

					if(fim_calc_tr){
						if (ConstTr==nullptr){
							ConstTr=new Rs_Tr(m_Tr_calc_gl);
							c=0;
						};
						c++;//Tr_s<<"c"<<c<<endl;
						////filtro.fil_sampl((control.IDQ_rotor_d()),/*(float)*/ (control.IDQ_rotor_q()),(control.VDQ_rotor_d()),(control.VDQ_rotor_q()),(control.get_wr()));
						ConstTr->enter_medition(/*filtro.filtred_isx()*/(control.IDQ_rotor_d()),/*filtro.filtred_isy()*/(control.IDQ_rotor_q()),/*filtro.filtred_usx()*/ (control.VDQ_rotor_d()),/*filtro.filtred_usy()*/ (control.VDQ_rotor_q()), (control.get_wr())/*filtro.filtred_Theta()*/);				
					}
					else/*if(!fim_calc_tr)*/{	
						if (future_is_ready(foo)){
								fim_calc_tr=true;
								if(ConstTr){delete ConstTr;ConstTr=nullptr;};
								ofstream Tr_s("Tr.txt",ios::app);
								Tr_s<<control.get_n()*T<<" sec. m_Tr_calc_gl:"<<m_Tr_calc_gl<<endl;//TODO:remover no fim
								
								//para calc valor med de Tr calculado:
								for(unsigned i=15;i>0;--i){v_med_Tr_calc[i-1]=v_med_Tr_calc[i];}
								for(unsigned i=15;i>0;--i){v_med_T[i-1]=v_med_T[i];}
								T_act=control.get_n()*T;
								v_med_Tr_calc[15]=Tr_calc_gl*(T_act-T_last);
								v_med_T[15]=T_act-T_last;
								double val_Tr_calc_gl=0.0;
								for(unsigned i=0;i<16;++i){val_Tr_calc_gl+=v_med_Tr_calc[i];}
								double m_T=0.0;
								for(unsigned i=0;i<16;++i){m_T+=v_med_T[i];}
								m_Tr_calc_gl=val_Tr_calc_gl/m_T;
								T_last=T_act;
								
								Tr_s.close();
						}
					}
					if(c==100 && ConstTr!=nullptr){		//TODO remove second part		
						//if(fim_calc_tr){
							//TODO:incluir try, throw catch
							foo= std::async(std::launch::async, mythread, /*std::move(*/ConstTr/*)*/);
							fim_calc_tr=false;
							c++;//c=0;
					}
					
				};//num++;
//_____________
		
		}
	fim_ciclo=true;
		
	auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    
    auto timeToWait = timeWindow - elapsed;
    if(timeToWait > std::chrono::microseconds::zero())
    {
        std::this_thread::sleep_for(timeToWait);
    }
	}disable_raw_mode();
	tcflush(0, TCIFLUSH);
	//while//
	return 0;
}
