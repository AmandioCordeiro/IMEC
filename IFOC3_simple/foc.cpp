/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define CST_DIGITS 15
//#include "my_fp.h"
//#include "my_math.h"
#include "foc.hpp"
//#include "sine_core.h"



ofstream Log ("log.txt");//TODO remove at end
ofstream fTorque ("torque.txt");//TODO remove at end
ofstream fVel ("speed.txt");//TODO remove at end
ofstream fload ("load.txt");//TODO remove at end
ofstream T1T2 ("T1T2.txt");//TODO remove at end
ofstream fwref ("wref.txt");//TODO remove at end
ofstream fIDQ ("IDQ_q__e_Vs.txt");//TODO remove at end
ofstream IDQ_d__lma ("IDQ_d__lma.txt");//TODO remove at end
ofstream fImr ("Imr.txt");//TODO remove at end


//#define SQRT3           FP_FROMFLT(1.732050807568877293527446315059)




//static const s32fp sqrt3inv1 = FP_FROMFLT(0.57735026919); //1/sqrt(2)
//static const s32fp sqrt3inv2 = 2*sqrt3inv1; //2/sqrt(2)
//static const s32fp sqrt3ov2 = (SQRT3 / 2);
//static const s32fp sqrt32 = FP_FROMFLT(1.224744871); //sqrt(3/2)
//static int dir = 1;

/*
s32fp FOC::id;
s32fp FOC::iq;
uint32_t FOC::DutyCycles[3];
*/
/** @brief Transform current to rotor system using Clarke and Park transformation
  * @post flux producing (id) and torque producing (iq) current are written
  *       to FOC::id and FOC::iq
  */
 /*
void FOC::ParkClarke(s32fp il1, s32fp il2, uint16_t angle)
{
   s32fp sin = dir * SineCore::Sine(angle);
   s32fp cos = SineCore::Cosine(angle);
   //Clarke transformation
   s32fp ia = il1;
   s32fp ib = FP_MUL(sqrt3inv1, il1) + FP_MUL(sqrt3inv2, il2);
   //Park transformation
   s32fp idl = FP_MUL(sqrt32, (FP_MUL(cos, ia) + FP_MUL(sin, ib)));
   s32fp iql = FP_MUL(sqrt32, (-FP_MUL(sin, ia) + FP_MUL(cos, ib)));

   id = IIRFILTER(id, idl, 2);
   iq = IIRFILTER(iq, iql, 2);
}
*/
/*
void FOC::InvParkClarke(s32fp id, s32fp iq, uint16_t angle)
{
   s32fp sin = dir * SineCore::Sine(angle);
   s32fp cos = SineCore::Cosine(angle);

   //Inverse Park transformation
   s32fp ia = FP_MUL(sqrt32, (FP_MUL(cos, id) + FP_MUL(sin, iq)));
   s32fp ib = FP_MUL(sqrt32, (-FP_MUL(sin, id) + FP_MUL(cos, iq)));
   //Inverse Clarke transformation
   DutyCycles[0] = ia;
   DutyCycles[1] = FP_MUL(-FP_FROMFLT(0.5), ia) + FP_MUL(sqrt3ov2, ib);
   DutyCycles[2] = FP_MUL(-FP_FROMFLT(0.5), ia) - FP_MUL(sqrt3ov2, ib);
}
*/
/*
void FOC::SetDirection(int _dir)
{
   dir = _dir;
}
*/

float Wm = 0.0;	
float Rd_we=0.0;
float Rq_we=0.0;
float IDQ_d_lma=Idn;// rotor magnetization current
			
//float Wn = 0.0;
//float Wc = 0.0;	
float Tm=0.0;
float Tm2=0.0;
float Tm3=0.0;
float iqmax=0.0;
float error=0.0;
int sinal_=0.0;

float IDC=0.0;//TODO: remove at end. to calc medium value of IDC 
float vaa=0.0,vbb=0.0,vcc=0.0;

long n=0;
float ids=0.0, iqs=0.0;
float VDC;

float vel_p=20.0;////20.0;//6;//1000;//1;//1//9/ /1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO: speed controller gain 
float vel_i=0.0006;//0.00006;//TODO: speed controller integral gain 
float torque_control_p=/*to use current controller */128/*estava este val em torq_control:0.33,mas experimentei c 1.4;*//* e deu bons result.*//*3.6*0.124*/;//alterei*0.5 TODO: torque controller gain
float torque_control_i=0.000008/*alterei0.0009(0.001) 0.008*/;//TODO: torque controller integral gain
float current_control_x_p=0.54;//0.5 a experiencia tava 0.6
float current_control_x_i=0.00001;

float Tm1;

float Lsro=(ro*Ls);//0.00081261//TODO make define


/////////////////////////////////////////////////////////////////////////////////////////////////////////
float dy_nt_(float /*&*/y1, float /*&*/y_1){//y1 significa y[n+1] 
	return ((y1-y_1)/(2*T));}

float d2y_nt_(float /*&*/y1, float /*&*/y,float /*&*/y_1){//derived can only be obtained  when n>=? 
	return ((y1-2*y+y_1)/(T*T));}

float max_mod_v_ref=0.0;
float ang=0.0;
float ang_u=0.0;
void FOC::calc_max_mod_v_ref(tTwoPhase v_bi){
ang=atan2(v_bi.beta,v_bi.alpha);

	//float ang_u=0.0;

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
					
	max_mod_v_ref = VDC/(cos(ang_u)*CONST_SQRT3_);//TODO uncoment in the end cos for max 
};

FOC::FOC():abc_voltage_svpwm(0.0,0.0,0.0),n_rtc(0),VDQ_ant(0.0,0.0),const_VDQ_d(0.0),const_VDQ_q(0.0),RotorFluxAngle(0.0)/*,angle(0.0)*/,abc_current(0.0,0.0,0.0),vel_Min_pid_res(-300),vel_Max_pid_res(300),IDQ(0.0,0.0),VDQ(0.0,0.0),IDQ_d_1(0.0),IDQ_d_(0.0),IDQ_d1(0.0),IDQ_d_p(0.0),IDQ_d_pp(0.0),IDQ_q_1(0.0),IDQ_q_(0.0),IDQ_q1(0.0),IDQ_q_p(0.0),IDQ_q_pp(0.0),wmr_(0.0),wmr1(0.0),wmr_p(0.0),v(0.0,0.0)/*,vaa(0.0),vbb(0.0),vcc(0.0)*/,Tr_calc(Tr),VDQ_rtc(0.0,0.0){
	vel.init_pid(0.0/*wr_*/,/*w_ref*/0.0,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
	vel.tune_pid(vel_p,vel_i,vel_d);/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*/

			//	vel.calc_pid();
	torque_control.init_pid(IDQ.q*3.0/2.0*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
	torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);

				//torque_control.calc_pid();
	current_control_y.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
	current_control_y.tune_pid(current_control_y_p,current_control_y_i,current_control_y_d);

				//current_control_y.calc_pid();	
			
	current_control_x.init_pid(IDQ.d,/*TODO*/Idn,current_control_x_Min_pid_res,current_control_x_Max_pid_res ,current_control_x_cel);
	current_control_x.tune_pid(current_control_x_p,current_control_x_i,current_control_x_d);
		
	};


void FOC::vel_tune_pid(){vel.tune_pid(vel_p,vel_i,vel_d);};
void FOC::torque_control_tune_pid(){torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);};

void FOC::GetDutyCycles(float il1, float il2, /*float VDC, */float w_ref/*commanded rotor speed*/, float wr_/*rotor speed*/)
{	
	Vmax = VDC/CONST_SQRT3_;
	
	il3 = -il1 -il2;//if system simetric
	
	//to measure previous current DC://TODO not needed to run IFOC, just adicional information
	if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
	{	IDC = il1/*or abc_current.a*/*T1/T - il3*T2/T;}
	else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
	{   IDC = - il3*T1/T+il2*T2/T;}
	else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
	{	IDC = il2*T1/T - il1*T2/T;}
	else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
	{	IDC = - il1*T1/T+il3*T2/T;}
	else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
	{	IDC = il3*T1/T - il2*T2/T;}
	else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
	{	IDC = - il2*T1/T+il1*T2/T;}
			
	if(n==0){//first iteration
			//PIDs----------------------			
			 //vel.init_pid(wr_,w_ref,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
				vel.calc_pid();
			 //torque_control.init_pid(IDQ.q*3.0/2.0*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
				torque_control.calc_pid();
			 //current_control_y.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
				current_control_y.calc_pid();	
			
			//current_control_x.init_pid(IDQ.d,/*TODO*/Idn,current_control_x_Min_pid_res,current_control_x_Max_pid_res ,current_control_x_cel);
			//	current_control_x.set_process_point(/*angle.get_imr()*/IDQ.d);// i changed: put imr instead IDQd BUT not best
			//current_control_x.set_setpoint(IDQ_d_lma);
				current_control_x.calc_pid();
			VDQ.q=current_control_y.get_pid_result();//torque_control.get_pid_result();
			VDQ.d=current_control_x.get_pid_result();
			Log<<"VDQ"<<VDQ.d<<" "<<VDQ.q<<" "<<"IDQ_d_lma"<<IDQ_d_lma<<"\n";//TODO remove at end
			//+Vdecouple-------------			
			IDQ_d1=IDQ.d;
			IDQ_q1=IDQ.q;
			wmr1=angle.get_wm();
			}
	else{
		
			abc_current.a = il1;abc_current.b = il2;abc_current.c = il3;
			
			IDQ=ClarkePark(RotorFluxAngle,abc_current);
	
			//the following variables are used to calc rotor time constant (adapt value at change).
			abc_voltage_svpwm.a=vaa;abc_voltage_svpwm.b=vbb;abc_voltage_svpwm.c=vcc;
			//TODO_: the following i dont know if its needed measure or just use the previous applied
			VDQ_rtc=ClarkePark(RotorFluxAngle,abc_voltage_svpwm);		
			/*int*/ sinal_=1;
			
			//to get Rotor flux angle:
			RotorFluxAngle=angle.RotFluxAng_(IDQ.q,IDQ.d,T,Tr_calc,wr_*np);		
			//RotorFluxAngle-=PI/2;//TODO_:-PI/2 remove? in simulation doesn't work but in real i think it's needed??
			
		//PIDs------------------------------------------- To get Voltage direct and quadracture before decoupling: 
			Wm = angle.get_wm();//Wm == we, used in LMA, sincronous speed, aten.- electric speed- ..*number pole pairs	
			Rd_we=Rs+Wm*Wm*M*M/Rm;
			Rq_we=Rs+1.0/(Tr_calc*Lr)*M*M+Wm*Wm*M*M*Llr*Llr/Rm/Lr/Lr;
			IDQ_d_lma=0.0;// rotor magnetization current
			
			vel.set_process_point(wr_);			
			vel.set_setpoint(w_ref);
			
			torque_control.set_process_point(IDQ.q*KT*angle.get_imr());
			
			//Tm;
			fTorque<<"torque_process_point: "<< (IDQ.q*KT/*3/2*np*M*M/Lr*/*angle.get_imr())<<endl;//TODO remove at end

			if(Wm < Wn) 
				{
				//max torque limit region:
				if (n>T_LOAD_1_sec) Tm1=TM1;else Tm1=LOAD_1_sec;//TODO remove T in the final and calc, in real maybe this is limited by batteries//maybe limite set speed 
					 Tm=Tm1;
					 vel.act_min_max(-Tm1,Tm1);
					 vel.calc_pid();		
							fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
							fTorque<<"<Wn: "<<Wn<<endl;// TODO remove at end
							fTorque<<"Tm1: "<<Tm1<<endl;// TODO remove at end
				
				if(abs(IDQ.q)<=sqrt(Rd_we/Rq_we)*Idn)
					{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
					}
				else{IDQ_d_lma=Idn;
					}
				}
			else {if (Wm < Wc ){
					//max current(power) limit region:
					Tm2=KT*sqrt(pow(Vmax/(Wm*Ls),2.0)-Imax*Imax*ro*ro)*sqrt(Imax*Imax-pow(Vmax/(Wm*Ls),2.0))/(1.0-ro*ro);
					Tm=Tm2;
					vel.act_min_max(-Tm2,Tm2);
					vel.calc_pid();		
						fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
						fTorque<<"<Wc: "<<Wc<<endl;//TODO remove at end
						fTorque<<"Tm2: "<<Tm2<<endl;//TODO remove at end
					if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
						}
					else {IDQ_d_lma=sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0*vel.get_pid_result()*vel.get_pid_result()/KT/KT)/2);
						}
					}
				else {
					//max Power-speed(voltage) limit region:
					Tm3=KT_t_cag*(pow((Vmax/(Wm*Ls)),2.0)/(2.0*ro));
					Tm=Tm3;
					vel.act_min_max(-Tm3,Tm3);
					vel.calc_pid();		
						fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
						fTorque<<">Wc: "<<Wc<<endl;//TODO remove at end
						fTorque<<"Tm3: "<<Tm3<<endl;//TODO remove at end
					if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {
						IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls*vel.get_pid_result()*vel.get_pid_result()/KT/KT))/(2.0*pow(Wm*Ls,2.0)));
						}
					}
				}
			if (isnanf(IDQ_d_lma)){/*IDQ_d_lma=Idn;*/IDQ_d__lma<<"is not a number IDQ_d_lam "<<endl;}//TODO remove at end

			if ((IDQ_d_lma<Idmin) || isnanf(IDQ_d_lma)) IDQ_d_lma=Idmin;
			
				fTorque << "tc_ref:  "<< vel.get_pid_result() <<endl;//TODO remove at end

			
			torque_control.set_setpoint(vel.get_pid_result());
			torque_control.calc_pid();
				fTorque<<"torque current error: "<<torque_control.get_current_error()<<"torque pid result: "<<torque_control.get_pid_result()<<endl;//TODO remove at end

			iqmax=Tm/KT/angle.get_imr()/*0.97cag*/;
			if (IDQ.q>iqmax)current_control_y.set_process_point(iqmax);
			else
			  current_control_y.set_process_point(IDQ.q);
			current_control_y.set_setpoint(torque_control.get_pid_result());
			current_control_y.calc_pid();
			
						IDQ_d__lma<<FIXED_FLOAT(n*T)<<"sec.; IDQ_D_lma: "<<IDQ_d_lma<<endl<<"IDQ_d: "<<IDQ.d<<endl;//TODO remove at end

			
			current_control_x.set_process_point(/*angle.get_imr()*/IDQ.d);// i changed: put imr instead IDQd BUT not best
			current_control_x.set_setpoint(IDQ_d_lma);
			current_control_x.calc_pid();
		
			VDQ.d=current_control_x.get_pid_result();//because tension is not proporcional of currents, used in controll as currents but after decoupling, tension because is a Voltage source inverter 											   
						cout<<"iqmax: "<<iqmax;//TODO remove at end

			VDQ.q=current_control_y.get_pid_result()/*torque_control.get_pid_result()*/;			
			if (VDQ.q>0 && VDQ.q>iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/))VDQ.q=iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/);
			else if (VDQ.q<0 && VDQ.q<-iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/))VDQ.q=-iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/);
			
			/*here*/
			VDQ_ant=VDQ;
						fIDQ<<FIXED_FLOAT(n*T)<<" sec.;"<<" VDQ_rfa.d: "<<VDQ_rtc.d<<endl<<" IDQd: "<<IDQ.d<<" IDQq: "<<IDQ.q<<endl/*<<"(IDQd^2+IDQq^2)^(1/2): "<<sqrt(IDQd*IDQd+IDQq*IDQq)*//*IDQd+IDQq*/<<"IDC:"<<IDC/*<<"IDC2_3:"<<IDC2_3*/<<endl;//TODO remove at end

		}
		
	//+Vdecouple-------------To get Voltage direct and quadracture after decoupling	
			IDQ_d_1=IDQ_d_;
			IDQ_d_=IDQ_d1;
			IDQ_d1=IDQ.d;
			
			IDQ_d_p=dy_nt_(	IDQ_d1, IDQ_d_1);
			IDQ_d_pp=d2y_nt_(IDQ_d1,IDQ_d_ ,IDQ_d_1);
			
			IDQ_q_1=IDQ_q_;
			IDQ_q_=IDQ_q1;
			IDQ_q1=IDQ.q;
			
			IDQ_q_p=dy_nt_(	IDQ_q1, IDQ_q_1);
			IDQ_q_pp=d2y_nt_(IDQ_q1,IDQ_q_ ,IDQ_q_1);
			
			wmr__1=wmr_;
			wmr_=wmr1;
			wmr1=Wm;
			
			wmr_p=dy_nt_(wmr1, wmr__1);
		
			/*VDQ.d=VDQ.d+*/const_VDQ_d=(T_lag*Rs)*IDQ_d_p+T_lag*Lsro*IDQ_d_pp-wmr1*(Lsro-T_lag*Rs)*IDQ_q1+wmr1*wmr1*(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))-T_lag*Lsro*IDQ_q1*wmr_p;				
			/*VDQ.q=VDQ.q+*/const_VDQ_q=T_lag*Rs*IDQ_q_p+T_lag*Lsro*IDQ_q_pp+wmr1*(Lsro-T_lag*Rs)*IDQ_d1+wmr1*wmr1*T_lag*Lsro*IDQ_q1+(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))*wmr_p+(Ls-Lsro)*wmr1*abs(angle.get_imr());
			
			//if ( /*VDQ.q > 0 && */VDQ.q > VDC/sqrt(3)*0.96 ) VDQ.q = VDC/sqrt(3)*0.96;
			//else if ( /*VDQ.q < 0 && */VDQ.q < -VDC/sqrt(3)*0.96 ) VDQ.q = -VDC/sqrt(3)*0.96;
			VDQ.d+=const_VDQ_d;
			VDQ.q+=const_VDQ_q;
				fIDQ<<"VDQ.d +decoupling: "<<VDQ.d<<endl;
				fIDQ<<"VDQ.q +decoupling: "<<VDQ.q<<endl;

				fImr<<FIXED_FLOAT(n*T)<<"sec.; imr (rotor magnetization current): "<<angle.get_imr()/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;	
		
	//-------------			
		
		if (n < 10000000)n++;//TODO:number of iterations 
		v=InvPark((RotorFluxAngle),VDQ);//voltage to be applied
				

  //SVPWM:---------------
	{// v tTwoPhase- bifasico, tendo como referencia o estator	
		
	calc_max_mod_v_ref(v);
		if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_v_ref*max_mod_v_ref ){
			if (v.alpha>2.0/3.0*VDC || v.alpha<-2.0/3.0*VDC){	
			
				v.beta = sin(ang)*max_mod_v_ref;
				v.alpha = cos(ang)*max_mod_v_ref;
			}
			else if (v.alpha<=2.0/3.0*VDC && v.alpha>VDC/3.0)
				{
					if (v.beta>0)v.beta=(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);
				}
				else if(v.alpha>-VDC/3.0 && v.alpha<=VDC/3.0){
						if (v.beta>0)v.beta=VDC/CONST_SQRT3_;else /*if(v.beta<0)*/ v.beta=-VDC/CONST_SQRT3_;
						}
					 else if(v.alpha<=-VDC/3.0 && v.alpha>=-2.0/3.0*VDC){
							if (v.beta>0)v.beta=(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);
							}
			calc_max_mod_v_ref(v);	  
			if (((v.beta*v.beta+v.alpha*v.alpha)) > max_mod_v_ref*max_mod_v_ref ){
				v.beta = sin(ang)*max_mod_v_ref;
				v.alpha = cos(ang)*max_mod_v_ref;
			
			}
		}
	
	
		
		if (ang>=0&&ang<(PI/3)){
				T1=3.0/2.0*T/VDC*(v.alpha-v.beta/CONST_SQRT3_);
				a1=1;b1=0;c1=0;	//ozz 
				T2=CONST_SQRT3_*T/VDC*v.beta;
				a2=1;b2=1;c2=0	;/*v_ooz*/
				
				if ((T1+T2)<T) pwm_a=(T1+T2)/T; else pwm_a=1;
				if ((T1+T2)<T) pwm_b=T2/T; else pwm_b=T2/(T1+T2);
				pwm_c=0;
		}else if(ang>=(PI/3)&&ang<(2.0*PI/3)){
				T1=3.0*T/2.0/VDC*(v.alpha+1/CONST_SQRT3_*v.beta);
				a1=1;b1=1;c1=0;//ooz 
				T2=3.0*T/VDC/2.0*(v.beta/CONST_SQRT3_-v.alpha);
				a2=0;b2=1;c2=0;/*v_zoz*/
				
				if ((T1+T2)<T) pwm_a=T1/T; else pwm_a=T1/(T1+T2);
				if ((T1+T2)<T) pwm_b=(T1+T2)/T; else pwm_b=1;
				pwm_c=0;
		}else if(ang>=(2.0*PI/3)&&ang<(PI)){
				T1=T*CONST_SQRT3_/VDC*v.beta;
				a1=0;b1=1;c1=0;//zoz 
				T2=CONST_SQRT3_/2.0*T/VDC*(-v.beta-CONST_SQRT3_*v.alpha);
				a2=0;b2=1;c2=1;
				
				pwm_a=0;
				if ((T1+T2)<T) pwm_b=(T1+T2)/T; else pwm_b=1;
				if ((T1+T2)<T) pwm_c=T2/T; else pwm_c=T2/(T1+T2);
		}else if(ang>=(-PI)&&ang<(-2.0*PI/3)){
				T1=3.0*T/2.0/VDC*(-v.alpha+v.beta/CONST_SQRT3_);
				a1=0;b1=1;c1=1;//zoo 
				T2=-CONST_SQRT3_*T/VDC*v.beta;
				a2=0;b2=0;c2=1;
				
				pwm_a=0;
				if ((T1+T2)<T) pwm_b=T1/T; else pwm_b=T1/(T1+T2);
				if ((T1+T2)<T) pwm_c=(T1+T2)/T; else pwm_c=1;			
		}else if(ang>=(-2*PI/3)&&ang<(-PI/3)){
				T1=CONST_SQRT3_*T/2.0/VDC*(-v.alpha*CONST_SQRT3_-v.beta);
				a1=0;b1=0;c1=1;//zzo 
				T2=CONST_SQRT3_*T/2.0/VDC*(-v.beta+v.alpha*CONST_SQRT3_);
				a2=1;b2=0;c2=1;
				
				if ((T1+T2)<T) pwm_a=T2/T; else pwm_a=T2/(T1+T2);
				pwm_b=0;
				if ((T1+T2)<T) pwm_c=(T1+T2)/T; else pwm_c=1;
		}else /*if(ang>=(-PI/3)&&ang<(0))*/{
				T1=-1.0*T/VDC*CONST_SQRT3_*v.beta;
				a1=1;b1=0;c1=1;//ozo 
				T2=3.0*T/2.0/VDC*(v.beta/CONST_SQRT3_+v.alpha);
				a2=1;b2=0;c2=0;
				
				if ((T1+T2)<T) pwm_a=(T1+T2)/T; else pwm_a=1;
				pwm_b=0;
				if ((T1+T2)<T) pwm_c=T1/T; else pwm_c=T1/(T1+T2);		
		}
						
		 T0=T-(T1+T2);
		 
		//DutyCycles[0] = pwm_a;
		//DutyCycles[1] = pwm_b;
		//DutyCycles[2] = pwm_c;
	}
  //------
   	
	//--------------Rotor Time Constant value adjust:			
			if (n_rtc<90000)n_rtc++;//TODO needed or just use n. In simulation i made reset when changed gear
			if (n_rtc>70000 ) {// TODO	
							
				/*float*/ error = VDQ_rtc.d-(Rs*IDQ.d-angle.get_wm()*((Ls*Lr-M*M)/Lr)*IDQ.q);
				
				//the following is because it dont work at 0 power 0 speed: 
				if ((IDQ.q<-10.5*3 && wr_<-/*30*/4 ) || (IDQ.q>10.5*3 && wr_ >4) ){
				  sinal_=1;}
				else if((IDQ.q>10.5*3 && wr_<-4) || (IDQ.q<-10.5*3 && wr_ >4 )){
						sinal_=-1;} 
						else{ sinal_=0;}
						
				Tr_calc=(Tr_calc + sinal_*error*0.000002/*0.000025*//*pi_contr*/);//TODO_ tune numeric value to rate of change Rr
				} 
	//--------------			
				
//tension that will be applied next, aproximation
//TODO ==
//abc_voltage=InvClarke(v);//next == at this
			if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
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
//--------
}
FOC::~FOC(){};
