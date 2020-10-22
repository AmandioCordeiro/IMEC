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
//#include "ClarkeParkTransforms.cpp"
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
#include "variables.hpp"
using namespace std;
using namespace Maths;

float Lrro=(ro*Lr);//0.0021891
#define R 0.34 //whell radius
#define MASSA (3500.0)//massa veiculo
/*#define*/float T_G_R = (4.313/*2.33*//*1.436 1 0.789*/*4.1/*reduction:1.9*/); //total gear racio. (gear_box*final_gear*"reduction_low_ratio")
float T_G_R_ant=T_G_R;	
float gradiente=0.0;//-0.25;//0.15;
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

float pl=0.0;
bool p=true;

float torq_L=0.0;
float J=(0.071);//TODO:?? 1pv5135-4ws14: 0.071

float IDC_med=0.0;
int it=0;

bool quit=false;


//clutch:
#define N_P 2.0 
#define us 0.15
#define ud 0.24
#define fi 1.5
#define R1_ 0.074
#define R2_ 0.104
#define XTO_CNT 0.00402
#define XTO_CLS 0.00778
#define XTO_MAX 0.012
#define F_MAX 17000//5000

#define CLUTCH_TIME 0.5//0.13//0.5 sec.
int clutch_time=-1;

ofstream diodes("diodes.txt");


FOC foc_;//TODO_ create it in embedded
FP_RotFluxAng	FOC::angle;//(0.0);//TODO_ create it in embedded
battery_simulation FOC::bat;//(0.0);

//float Vmax=VDC/sqrt(3.0);	//need actualization in pratice in function get_VDC( 

float t=0.0;

float roLs=(ro*Ls);//0.013998//ro*Ls
float fdr=0.0,fqr=0.0, Iqm=0.0, Idm=0.0, fqs=0.0, fds=0.0;


extern float ids, iqs, IDC;
float iaa_p=0.0, vaa_p=0.0, cos_phi=0.0, cos_phi_a=0.0, two_phi=0.0, desc_p=0.0, phi=0.0;

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
	float FLT_FROMFP_get_ias(){//TODO in real read from sensors
		FP_abc_current.a= (FP_InvClarkePark(FP_RotorFluxAngle/*np*theta_r*/,FP_tTwoPhaseDQ(FP_FROMFLT(ids),FP_FROMFLT(iqs))).a);
		return FLT_FROMFP(FP_abc_current.a);
	};
	float get_ibs(){//TODO in real read from sensors
		abc_current.b= (InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).b);
		return abc_current.b;
	};
	float FLT_FROMFP_get_ibs(){//TODO in real read from sensors
		FP_abc_current.b= (FP_InvClarkePark(FP_RotorFluxAngle/*np*theta_r*/,FP_tTwoPhaseDQ(FP_FROMFLT(ids),FP_FROMFLT(iqs))).b);
		return FLT_FROMFP(FP_abc_current.b);
	};
	float get_ics(){//TODO in real read from sensors or if system simetric and equilibrated: 
		//abc_current.c=-abc_current.a-abc_current.b; but is ??better get abc_current.c from measure
		abc_current.c=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).c);
		return abc_current.c;
	};
	float FLT_FROMFP_get_ics(){//TODO in real read from sensors or if system simetric and equilibrated: 
		//abc_current.c=-abc_current.a-abc_current.b; but is ??better get abc_current.c from measure
		FP_abc_current.c=(FP_InvClarkePark(FP_RotorFluxAngle/*np*theta_r*/,FP_tTwoPhaseDQ(FP_FROMFLT(ids),FP_FROMFLT(iqs))).c);
		return FLT_FROMFP(FP_abc_current.c);
	};
//-----------------	

//torque generated, frame generic . used #######"backward euler method"######
//frame used with rotor flux angle
//so, ids==IDQ.d and iqs==IDQ.q
float torque_g(float Vds,float Vqs,float angle_get_wm){
	float Lls=Ls-M;	
		//Vaa, vbb, vcc, geneated by motor
		//TODO calc the Idm, Iqm, fqs, fds, fqr, fdr with Ids, Iqs readed values??
		float Vds_g = Rs * ids /*- Rs*M*Idm/Lls*/ + (fds_ - _fds) / T - angle_get_wm * (/*_fqs+*//*fqs_*/iqs*((Ls*Lr-M*M)/Lr))/*/2.0*//*(iqs_ant+iqs)/2.0*Lls*/;
		float Vqs_g = Rs * iqs + (fqs_- _fqs) / T + angle_get_wm * fds_;
		
		float vaa_g, vbb_g, vcc_g;
		tThreePhase v_g= InvClarkePark( RotorFluxAngle, tTwoPhaseDQ( Vds_g , Vqs_g ) );
		FP_tThreePhase FP_v_g= FP_InvClarkePark( FP_RotorFluxAngle, FP_tTwoPhaseDQ( FP_FROMFLT(Vds_g) , FP_FROMFLT(Vqs_g) ) );// FP_ // 
		
		vaa_g=v_g.a;vbb_g=v_g.b;vcc_g=v_g.c;
		
		int a0=0,b0=0,c0=0;
		
		fTorque<<"vaa_g, vbb_g, vcc_g "<<vaa_g<<" "<<vbb_g<<" "<<vcc_g<<endl;
		float vaa_med=vaa_g/*(vaa+vaa_g)/2.0*/,vbb_med=vbb_g/*(vbb+vbb_g)/2.0*/,vcc_med=vcc_g/*(vcc+vcc_g)/2.0*/;
		
		if ((vaa_med-vbb_med)<(-VDC-0.5/*2.0*/))//estava 0.7 
			{	/*a1=1;a2=1;b1=0;b2=0;*/a0=0;b0=1;
				diodes<<n*T<<"sec. diode open-a_-b med"<<endl; 
				fTorque<<n*T<<"sec. diode open-a_-b med"<<endl;
				//vaa_med=(vaa+vaa_g)/2.0;vbb_med=(vbb+vbb_g)/2.0;		
			}
		if ((vaa_med-vcc_med)<(-VDC-0.5/*2.0*/)) 
			{	/*a1=1;a2=1;c1=0;c2=0;*/a0=0;c0=1;
				diodes<<n*T<<"sec. diode open-a_-c med"<<endl;
				fTorque<<n*T<<"sec. diode open-a_-c med"<<endl;
				//vaa_med=(vaa+vaa_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}
		if ((vbb_med-vaa_med)<(-VDC-0.5/*2.0*/))
			{	/*b1=1;b2=1;a1=0;a2=0;*/b0=0;a0=1;
				diodes<<n*T<<"sec. diode open-b_-a med"<<endl;
				fTorque<<n*T<<"sec. diode open-b_-a med"<<endl;
				//vaa_med=(vaa+vaa_g)/2.0;vbb_med=(vbb+vbb_g)/2.0;
			}
		if ((vbb_med-vcc_med)<(-VDC-0.5/*2.0*/)) 
			{	/*b1=1;b2=1;c1=0;c2=0;*/b0=0;c0=1;
				diodes<<n*T<<"sec. diode open-b_-c med"<<endl;
				fTorque<<n*T<<"sec. diode open-b_-c med"<<endl;
				//vbb_med=(vbb+vbb_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}	
		if ((vcc_med-vaa_med)<(-VDC-0.5/*2.0*/)) 
			{	/*c1=1;c2=1;a1=0;a2=0;*/c0=0;a0=1;
				diodes<<n*T<<"sec. diode open-c_-a med"<<endl;
				fTorque<<n*T<<"sec. diode open-c_-a med"<<endl;
				//vaa_med=(vaa+vaa_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}
		if ((vcc_med-vbb_med)<(-VDC-0.5/*2.0*/))
			{	/*c1=1;c2=1;b1=0;b2=0;*/c0=0;b0=1;
				diodes<<n*T<<"sec. diode open-c_-b med"<<endl;
				fTorque<<n*T<<"sec. diode open-c_-b med"<<endl;
				//vbb_med=(vbb+vbb_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}
		
		
		
		
		
		/*
		if ((vaa_g-vbb_g)>0 && (vaa-vbb)>0 && ((vaa_g-vbb_g)-(vaa-vbb))>0.7*2) 
			{	a1=1;a2=1;b1=0;b2=0;a0=1;b0=0;
				fTorque<<"diode open-a-b_ "<<endl; 
				vaa_med=(vaa+vaa_g)/2.0;vbb_med=(vbb+vbb_g)/2.0;		
			}
		if ((vaa_g-vcc_g)>0 && (vaa-vcc)>0 && ((vaa_g-vcc_g)-(vaa-vcc))>0.7*2) 
			{	a1=1;a2=1;c1=0;c2=0;a0=1;c0=0;
				fTorque<<"diode open-a-c_ "<<endl;
				vaa_med=(vaa+vaa_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}
		if ((vbb_g-vaa_g)>0 && (vbb-vaa)>0 && ((vbb_g-vaa_g)-(vbb-vaa))>0.7*2)
			{	b1=1;b2=1;a1=0;a2=0;b0=1;a0=0;
				fTorque<<"diode open-b-a_ "<<endl;
				vaa_med=(vaa+vaa_g)/2.0;vbb_med=(vbb+vbb_g)/2.0;
			}
		if ((vbb_g-vcc_g)>0 && (vbb-vcc)>0 && ((vbb_g-vcc_g)-(vbb-vcc))>0.7*2) 
			{	b1=1;b2=1;c1=0;c2=0;b0=1;c0=0;
				fTorque<<"diode open-b-c_ "<<endl;
				vbb_med=(vbb+vbb_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}	
		if ((vcc_g-vaa_g)>0 && (vcc-vaa)>0 && ((vcc_g-vaa_g)-(vcc-vaa))>0.7*2) 
			{	c1=1;c2=1;a1=0;a2=0;c0=1;a0=0;
				fTorque<<"diode open-c-a_ "<<endl;
				vaa_med=(vaa+vaa_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}
		if ((vcc_g-vbb_g)>0 && (vcc-vbb)>0 && ((vcc_g-vbb_g)-(vcc-vbb))>0.7*2)
			{	c1=1;c2=1;b1=0;b2=0;c0=1;b0=0;
				fTorque<<"diode open-c-b_ "<<endl;
				vbb_med=(vbb+vbb_g)/2.0;vcc_med=(vcc+vcc_g)/2.0;
			}
		*/
		//TODO :??in embedded its needed read tensions?? Power sources: battery + Load
//		VDQ_rtc=ClarkePark(RotorFluxAngle,tThreePhase (vaa,vbb,vcc));//((vaa_med/*+vaa_g*/)/*2.0*/,(vbb_med/*+vbb_g*/)/*2.0*/,(vcc_med/*+vcc_g*/)/*2.0*/));//TODO
////if (n%2 == 0 /*&& n != 0*/){ 
////				T=T*2.0;
				
			/*if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
			{	vaa=T1/T*2*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3+T2/T*VDC/3;vcc=-T1/T*VDC/3-T2/T*2*VDC/3;}
			else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
			{	vaa=T1/T*VDC/3-T2/T*VDC/3;vbb=T1/T*VDC/3+T2/T*2*VDC/3;vcc=-2*T1/T*VDC/3-T2/T*VDC/3;}
			else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
			{	vaa=-T1/T*VDC/3-2*T2/T*VDC/3;vbb=T1/T*VDC*2/3+T2/T*VDC/3;vcc=-T1/T*VDC/3+T2/T*VDC/3;}
			else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
			{	vaa=-T1/T*VDC*2/3-T2/T*VDC/3;vbb=T1/T*VDC/3-T2/T*VDC/3;vcc=T1/T*VDC/3+T2/T*VDC*2/3;}
			else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
			{	vaa=-T1/T*VDC/3+T2/T*VDC/3;vbb=-T1/T*VDC/3-T2/T*VDC*2/3;vcc=T1/T*VDC*2/3+T2/T*VDC/3;}
			else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
			{	vaa=T1/T*VDC/3+2*T2/T*VDC/3;vbb=-T1/T*VDC*2/3-T2/T*VDC/3;vcc=T1/T*VDC/3-T2/T*VDC/3;}
			*/
			
			
			//	if(a0==1&&b0==0&&c0==0/*&&a0==1&&b0==1&&c0==0*/)
			//{	vaa=vaa_med+T*2*(vaa_med-vbb_med)/3/*+T2/T*VDC/3*/;vbb=vbb_med-T*(vaa_med-vbb_med)/3/*+T2/T*VDC/3*/;vcc=vcc_med-T*(vaa_med-vbb_med)/3/*-T2/T*2*VDC/3*/;}
			//else if(a0==1&&b0==1&&c0==0/*&&a2==0&&b2==1&&c2==0*/)
			//{	vaa=vaa_med+T0/T*VDC/3/*-T2/T*VDC/3*/;vbb=vbb_med+T*VDC/3/*+T2/T*2*VDC/3*/;vcc=vcc_med-2.0*T*VDC/3/*-T2/T*VDC/3*/;}
			//else if(a0==0&&b0==1&&c0==0/*&&a2==0&&b2==1&&c2==1*/)
			//{	vaa=vaa_med-T0/T*VDC/3/*-2*T2/T*VDC/3*/;vbb=vbb_med+T*VDC*2/3/*+T2/T*VDC/3*/;vcc=vcc_med-T*VDC/3/*+T2/T*VDC/3*/;}
			//else if(a0==0&&b0==1&&c0==1/*&&a2==0&&b2==0&&c2==1*/)
			//{	vaa=vaa_med-T0/T*VDC*2/3/*-T2/T*VDC/3*/;vbb=vbb_med+T*VDC/3/*-T2/T*VDC/3*/;vcc=vcc_med+T*VDC/3/*+T2/T*VDC*2/3*/;}
			//else if(a0==0&&b0==0&&c0==1/*&&a2==1&&b2==0&&c2==1*/)
			//{	vaa=vaa_med-T0/T*VDC/3/*+T2/T*VDC/3*/;vbb=vbb_med-T*VDC/3/*-T2/T*VDC*2/3*/;vcc=vcc_med+T*VDC*2/3/*+T2/T*VDC/3*/;}
			//else if(a0==1&&b0==0&&c0==1/*&&a2==1&&b2==0&&c2==0*/)
			//{	vaa=vaa_med+T0/T*VDC/3/*+2*T2/T*VDC/3*/;vbb=vbb_med-T*VDC*2/3/*-T2/T*VDC/3*/;vcc=vcc_med+T*VDC/3/*-T2/T*VDC/3*/;}
			
////T=T/2.0;
////			}
		tTwoPhaseDQ V_new = ClarkePark(RotorFluxAngle,tThreePhase (vaa/*_med*/,vbb/*_med*/,vcc/*_med*/));//(((vaa+vaa_g)/2.0),((vbb+vbb_g)/2.0),((vcc+vcc_g)/2.0)));//((vaa_med/*+vaa_g*/)/*2.0*/,(vbb_med/*+vbb_g*/)/*2.0*/,(vcc_med/*+vcc_g*/)/*2.0*/));//TODO
		FP_tTwoPhaseDQ FP_V_new = FP_ClarkePark(FP_RotorFluxAngle,FP_tThreePhase (FP_FROMFLT(vaa)/*_med*/,FP_FROMFLT(vbb)/*_med*/,FP_FROMFLT(vcc)/*_med*/));//(((vaa+vaa_g)/2.0),((vbb+vbb_g)/2.0),((vcc+vcc_g)/2.0)));//((vaa_med/*+vaa_g*/)/*2.0*/,(vbb_med/*+vbb_g*/)/*2.0*/,(vcc_med/*+vcc_g*/)/*2.0*/));//TODO

		Vds=V_new.d;Vqs=V_new.q;
	
		if (angle_get_wm==0.0)angle_get_wm=0.00000000001;
		float	s_2=(angle_get_wm-wr*np)/angle_get_wm*(angle_get_wm-wr*np)/angle_get_wm;
		float Rfe=Rm/(s_2+1);		
		//usado (fds_n+1-fds_n)/T=fds_n+1 +(...), e nao: (fds_n+1/T-fds_n)/T=fds_n+(...)
	
		fds_=1/(1/T+Rs/Lls)*(Vds+fds/T+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		fqs_=1/(1/T+Rs/Lls)*(Vqs+fqs/T-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		float fdr_=1/(1/T+Rr/Llr)*(fdr/T+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm);
		float fqr_=1/(1/T+Rr/Llr)*(fqr/T-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm);
		float Idm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm/*-np*wr*/)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		float Iqm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+Iqm/T-(angle_get_wm/*-np*wr*/)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
//Idm = 1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*(ids * Lls + M * Idm)+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm/*-np*wr*/)*Iqm);
			
		//float _fds,_fqs;
		_fds=fds;_fqs=fqs;
		
		fds=fds_;
		fqs=fqs_;
		fdr=fdr_;
		fqr=fqr_;
		Idm=Idm_;
		Iqm=Iqm_;
		
		ids_ant=ids; iqs_ant=iqs;
		
		ids=(fds-M*Idm)/Lls;//fds = ids * Lls + M * Idm;
		iqs=(fqs-M*Iqm)/Lls;//fqs = iqs * Lls + M * Iqm;
		
	/*:2%for mecanic losses and magnetic losses*/
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_out = sum_power_out + abs(IDC*VDC)*T/3600 ; else sum_power_out = sum_power_out + (t*wr)*T/3600;
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_in = sum_power_in + abs(/*get_torq_L()*/t*wr)*T/3600; else sum_power_in = sum_power_in + (IDC*VDC)*T/3600;
	//sum_power_trac += t*wr;
		//if (n%2 == 0 /*&& n != 0*/){ 
			fTorque<<"Vds: "<<Vds<<" Vqs: "<<Vqs<<" angle_get_wm:"<<angle_get_wm<<endl;

		sum_power_battery += IDC*VDC*T/3600;fTorque<<"battery energy consumed : "<<sum_power_battery<<" wh"<<endl;
		if (n*T==195)fTorque<<"efficiency end cycle "<<sum_power_out/sum_power_in<<endl;
		fTorque<<FIXED_FLOAT(n*T)<<" sec., Torque_generated (previous): "<<t/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl<<"power (Torque_generated*rotor_velocity)(previous):"<<(t/*np*/*wr)<<"; power (Vdc*Idc)(previous):"<<(IDC*VDC)<<endl;if (IDC<0) fTorque<<"efficiency (instant. generat.)(previous) :"<<(IDC*VDC)/(t/*np*/*wr);else fTorque<<" efficiency (instant. motor)(previous):"<<(t/*np*/*wr)/(IDC*VDC);fTorque<<" machine medium efficiency: "<<sum_power_out/sum_power_in/*<<" ef: "<<sum_power_trac/sum_power_battery*/<<endl<<"Wm, same than We- sincronous speed in ang. electric: "<<angle_get_wm<<endl<<"(angle_get_wm-wr*np) \"slip angle:\" "<<(angle_get_wm-wr*np)<<endl;//<<"torq_l:"<<torq_L<<endl;	
		//};	
	t=0.975/*<-losses mec., histerese mag., "slip" */*(3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds)));//torque generated by the tension applied
	
	return t;
	
};

float torque(float vas,float vbs,float vcs){
		fTorque<<" FP VDQ.d "<<FLT_FROMFP(FP_ClarkePark(0.0/*RotorFluxAngle*//*np*theta_r*/,FP_tThreePhase(FP_FROMFLT(vas),FP_FROMFLT(vbs),FP_FROMFLT(vcs))).d)<</*alpha*/"FP VDQ.q "<<FLT_FROMFP(FP_ClarkePark(0.0/*RotorFluxAngle*//*np*theta_r*/,FP_tThreePhase(FP_FROMFLT(vas),FP_FROMFLT(vbs),FP_FROMFLT(vcs))).q)/*beta*/<<" FLT_FROMFP(FOC::angle.FP_get_wm()) "<<FLT_FROMFP(FOC::angle.FP_get_wm())<<std::endl;
fTorque<<" VDQ.d "<<ClarkePark(0.0/*RotorFluxAngle*//*np*theta_r*/,tThreePhase(vas,vbs,vcs)).d/*alpha*/<<" VDQ.q "<<ClarkePark(0.0/*RotorFluxAngle*//*np*theta_r*/,tThreePhase(vas,vbs,vcs)).q/*beta*/<<"angle.get_wm "<<FLT_FROMFP(FOC::angle.FP_get_wm())<<std::endl;

		return torque_g(ClarkePark(0.0/*RotorFluxAngle*//*np*theta_r*/,tThreePhase(vas,vbs,vcs)).d/*alpha*/,ClarkePark(0.0/*RotorFluxAngle*//*np*theta_r*/,tThreePhase(vas,vbs,vcs)).q/*beta*/,FLT_FROMFP(FOC::angle.FP_get_wm()));//test//

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
	
	float Ru=1.0/( R2_ -R1_ )*(us/2.0* R2_ * R2_ +(ud-us)*( R2_ +1.0)/(fi*Wfc)*log(cosh(fi* R2_ *Wfc))-(us/2.0* R1_ * R1_ +(ud-us)*( R1_ +1.0)/(fi*Wfc)*log(cosh(fi* R1_ *Wfc))));
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
		if (e==true && velocidade*R/T_G_R*3.6 < 19/*26*/){//TODO para bat 145 e nao 300 dividi por 1.5 a velocidade de mudança pois wn e wc é metade
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
		if (e==true && velocidade*R/T_G_R*3.6 > 22/*30*/ && velocidade*R/T_G_R*3.6 < 35/*66*/ )
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
		if (e==true && velocidade*R/T_G_R*3.6 > 37.5/*30*/ && velocidade*R/T_G_R*3.6 < 55.8/*66*/ )
			{T_G_R = (/*4.313*/ 1.436/*1.436 1 0.789*/*4.1/*1.9*/);
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
		if (e==true && velocidade*R/T_G_R*3.6 > 58.3/*30*/ && velocidade*R/T_G_R*3.6 < 81.5/*66*/ )
			{T_G_R = (/*4.313 1.436*/ 1/* 0.789*/*4.1/*1.9*/);
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
		
		if (e==true && velocidade*R/T_G_R*3.6 > 83.505/*70*/ )
			{T_G_R = (/*4.313 2.33 1.436 1*/ 0.789*4.1/*1.9*/);
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
			if (n%2 == 0 /*&& n != 0*/){ 
				fTorque<<" w_ref "<<w_ref<<" wr "<<wr<<endl;
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
	
	char rr=' ';//just a pause in driving cicle

//	long c=0;

	char o;

	ifstream driving_cycle("ececol.txt");
	string line;
	int i=/*0*/4;//here, 0 to full test 4 ece +1 eudc.. 4 for eudc only
	bool primeiro=true;
	int gear=1;
	
	float IDC_med_=0.0;
	
	enable_raw_mode();
	
	while (quit==false){
	
		if (kbhit())
		{
			o=getchar();
			switch(o){					
				case '\n':break;
				case 'r' :disable_raw_mode();tcflush(0, TCIFLUSH);cout<<"running..."<<endl;run=true;enable_raw_mode();break;
				case 'q' :quit=true;break;
				////case 'm' :cout<<"enter magnetizing current reference:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	////tcflush(0, TCIFLUSH);cin>>imrreff;enable_raw_mode();/*fflush(stdin);*/break;
				case 's' :cout<<"enter speed:";e=false;/*e-> to set by driving cycle or by speed*//*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>w_ref;enable_raw_mode();/*fflush(stdin);*/break;
				case 'l' :cout<<"enter torque load:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>load_s;enable_raw_mode();p=false;e=false;/*fflush(stdin);*/break; 
				case 'g' :cout<<"enter gear:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>gear;if(gear==0)T_G_R =0; if(gear==1)T_G_R = (4.313/* 2.331.436/* 1 0.789*/*4.1/*1.9*/);if(gear==2)T_G_R = (/*4.313*/ 2.33/*1.436 1 0.789*/*4.1/*1.9*/);if(gear==3)T_G_R = (/*4.313 2.33*/1.436/* 1 0.789*/*4.1/*1.9*/);if(gear==4)T_G_R = (/*4.313 2.33 1.436*/ 1 /*0.789*/*4.1/*1.9*/);if(gear==5)T_G_R = (/*4.313 2.33 1.436 1 */0.789*4.1/*1.9*/);enable_raw_mode();/*fflush(stdin);*/break; 
				case 'a' :cout<<"enter prop. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'b' :cout<<"enter int. gain speed:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>vel_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 'c' :cout<<"enter prop. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'z' :cout<<"enter prop. gain current_control_q_p:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>current_control_q_p;enable_raw_mode();/*fflush(stdin);*/break;
				
				case 'e' :cout<<"enter prop. gain flux:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>current_control_d_p;enable_raw_mode();/*fflush(stdin);*/break;
				case 'f' :cout<<"enter int. gain flux:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>current_control_d_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 'd' :cout<<"enter int. gain torq_controll:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>torque_control_i;enable_raw_mode();/*fflush(stdin);*/break;
				case 't' :cout<<"enter battery temperature:";/*if(flag==true){flag=false;break;};*/disable_raw_mode();
	tcflush(0, TCIFLUSH);cin>>temp_bat;enable_raw_mode();/*fflush(stdin);*/break;
				
				case ' ' :cout<<"pause, press r to return"<<endl;run=false;break;//disable_raw_mode();
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
				case 'D' :disable_raw_mode();tcflush(0, TCIFLUSH);cout<<"running driving cycle"<<endl;run=true;e=true;enable_raw_mode();break;//set by driving_cycle
				
				default:cout<<"invalid operator, \ns(commanded speed)"<<" \na (enter prop. gain speed)"<<" \nb (enter int. gain speed)"<<"\nc (enter prop. gain torq_controll)"<<"\nd (enter int. gain torq_controll)"<<" \nl(load), "<</*\nt(set by torque load), */"\nv(set by vehicle load), \n+(increment speed), \n-(decrement speed) \ng (switch gear)"<</*, k(decrement load),p(increment load),*/", \nr(run), \nq(quit)"<<endl;	
			};
		
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
			else load=load_s;//load_s set rum mode by load, specifying N.M
		//TODO remove above at end
			/*control.*///foc_.vel_tune_pid();//TODO remove at END, in embedded
			/*control.*///foc_.torque_control_tune_pid();
			
			//_T1T2<<FIXED_FLOAT(n*T)<<" sec. "<<endl;
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
						fVel<<" (b+196*"<<i<<"): "<<(b+196*i)/*<<" b e end :"<<b<<end*/<<endl;
						////
						//if(b==62){
						//	
						//	while(rr!='r'){cout<<"press r to continue"<<endl;
						//		cin>>rr;};
						//	};
						////
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
				if ((n*T) == b/*(b+196*i)*/)	//here to run only eudc change to b only
				{	w_ref=end_f*1000/3600*T_G_R/R;
					getline(driving_cycle,line);
				
				}
				if (b==199/*400*/)quit=true;//here to run only ece change to 175
				};	
				
			set_w_ref(w_ref);
			
			set_torq_L(load);//TODO this is only in simulation, not needed in real
			//set_Theta_r();
			////foc_.get_VDC();//TODO_ adapt in real(embedded).function must read DC voltage			
			//TODO in real the following reads from e Hall sensors			
			//TODO julgo n ser necessario seguinte, trat em GetDutyCycles
			//get_ias();/*cout<<"ia"<<abc_current.a<<endl;*/get_ibs();get_ics();//abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor, is better get abc_current.c from measure				

			//**float ia=get_ias();//TODO to measure cos phi
			//**função a seguir com tempo variavel, necessario resolver para tempo const
			//**svpwm(control);//news gates and times
			/*here*/
			iaa_p=FLT_FROMFP_get_ias();//used to calc cosphi
			vaa_p=vaa;//used to calc cosphi
			
			
			
/////____
			
			FOC::bat.get_VDC();//TODO implement in real
			Vmax = VDC/CONST_SQRT3_;
			//**get_wr(vaa,vbb,vcc); 
			
			//DOC: the following "if" is for run simulation of motor in half of T
			if (n%2 == 0 /*&& n != 0*/){ 
				T=T*2.0;
				foc_.GetDutyCycles((FLT_FROMFP_get_ias()), (FLT_FROMFP_get_ibs()), /*get_*/VDC/*()*/,/*vaa, vbb, vcc,*/ (get_w_ref())/*commanded rotor speed*/, (get_w_r())/*rotor speed*/);
				//foc_.GetDutyCycles(FLT_FROMFP_get_ias(), FLT_FROMFP_get_ibs, VDC, get_w_ref(), get_w_r());
				
				T=T/2.0;
			
//_T1T2<<" vaa_n:"<<vaa<<" vbb_n:"<<vbb<<" vcc_n:"<<vcc<<endl<<"IDC_previous: "<<IDC<<endl;
fTorque<<" vaa_n:"<<vaa<<" vbb_n:"<<vbb<<" vcc_n:"<<vcc<<endl<<"IDC_previous: "<<IDC<<" VDC: "<<VDC<<" "<<endl;
			}
			
			get_wr(vaa,vbb,vcc);
	
	{		//********** used in simulation to calc cos_phi
				if (n%2 == 0 /*&& n != 0*/){ 
					fTorque<<"power iaa*vaa+ibb*vbb+icc*vcc: "<<get_ias()*vaa+/*InvClarkePark(RotorFluxAngle,IDQ)*/get_ibs()*vbb+get_ics()*vcc<<endl;
					//fTorque<<"vaa: "<<vaa<<" iaa: "<<abc_current.a<<endl;
					sfData_va_ia<<" "<<FIXED_FLOAT(n*T)<<" "<<vaa<<" "<<get_ias()<<" "<<get_ibs()<<" "<<get_ics()<<endl;
					fTorque<<" ia "<<get_ias()<<" ib "<<get_ibs()<<" ic "<<get_ics()<<endl;
			fTorque<<" FP_ia "<<FLT_FROMFP_get_ias()<<" FP_ib "<<FLT_FROMFP_get_ibs()<<" FP_ic "<<FLT_FROMFP_get_ics()<<endl;
			
				}
					if ( vaa_p >0 && vaa_p*vaa < 0 ) {
						cos_phi=n*T;
						fTorque<<"vaa pass by zero"<<endl;}
					else {fTorque<<"vaa_p:"<<vaa_p<<"vaa:"<<vaa<<endl;}
					if ( iaa_p >0 && iaa_p*FLT_FROMFP_get_ias() < 0 ) {
						two_phi=n*T-desc_p;
						desc_p=n*T;
						cos_phi=n*T-cos_phi;
						cos_phi_a=cos_phi/two_phi*2*PI;
						fTorque/*<<"cos_phi_t "<<cos_phi*/<<" phi: "<<cos_phi_a<<" cos_phi: "<<cos(cos_phi_a)<<endl;} 
					else {
						fTorque<<"iaa_p"<<iaa_p<<"iaa"<<get_ias()<<" phi: "<<cos_phi_a<<" cos_phi: "<<cos(cos_phi_a)<<endl;}
		
			//**********
	}	
			
			//fTorque<<" ia "<<get_ias()<<" ib "<<get_ibs()<<" ic "<<get_ics()<<endl;
			distance_+=velocidade/T_G_R*R*T; 
			fVel<<FIXED_FLOAT(n*T)<<"sec.;"<< " velocity clutch (rad/s): "<<velocidade<<"; T_G_R(total gear ratio): "<<T_G_R;
			 fVel<<";vehicle speed (km/h): "<<velocidade*R/T_G_R*3.6<<"; distance: "<<distance_<<" meters"<<endl;
			
//_____________

//_____________
		//:::::::IDC medio
		
		if (it<30000){
			it++;
			IDC_med += IDC;
			//IDC_med_ = (IDC_med/it);
			//IDC_med/=2;
			
		}else{
			IDC_med_ = (IDC_med/it);
			it=0;
			IDC_med=0.0;		
			}
		if (n%2 ==0 )
			fTorque<<" IDC med "<<IDC_med_<<endl;
		//::::::::
		
		n++;
		fTorque<<endl;
	};
	
	//fim_ciclo=true;
		
	
	};disable_raw_mode();
	tcflush(0, TCIFLUSH);
	
	return 0;

};
