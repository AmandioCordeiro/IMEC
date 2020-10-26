/*
 * 
 *
 * 2019 amrc <amandio.miguel@gmail.com>
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
//#define CST_DIGITS 15
//#include "my_fp.h"
//#include "my_math.h"
#include "foc.hpp"
//#include "sine_core.h"



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

float Wm = 0.0;	//synchronous speed
float Rd_we=0.0;
float Rq_we=0.0;
float IDQ_d_lma=Idn;// rotor magnetization current
			
//float Wn = 0.0;
//float Wc = 0.0;	

float Tm1=0.0;//max. torque in max. torque limit region:
float Tm2=0.0;//max. torque in max. current(power) limit region:
float Tm3=0.0;//max. torque in max. Power-speed(voltage) limit region:
float Tm=0.0;//max. torque of iteration (Tm1||Tm2||Tm3)

float iqmax=0.0;
float error=0.0;
int sinal_=0.0;

//float IDC=0.0;//TODO: remove at end. to calc medium value of IDC 
float vaa=0.0,vbb=0.0,vcc=0.0;

float ids=0.0, iqs=0.0;

extern long n;

bool incr_decr_Rr;//TODO remove at end, to vary Rr

//TODO remove at end the folloving block, replace for defines
//PI controllers values
float vel_p=20.0;////20.0;//6;//1000;//1;//1//9/ /1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO: speed controller gain 
float vel_i=0.0006;//0.00006;//TODO: speed controller integral gain 
float torque_control_p=/*to use current controller */128/*estava este val em torq_control:0.33,mas experimentei c 1.4;*//* e deu bons result.*//*3.6*0.124*/;//alterei*0.5 TODO: torque controller gain
float torque_control_i=0.000008/*alterei0.0009(0.001) 0.008*/;//TODO: torque controller integral gain
float current_control_d_p=0.54;//0.5 a experiencia tava 0.6
float current_control_d_i=0.00001;
float current_control_q_p=2.0*0.99/*new 1.56*//*1.4*//*0.64/*0.082 1.0*/;
float current_control_q_Max_pid_res = 8.8*1.29;//new TODO
float current_control_q_Min_pid_res = -8.8*1.29;//new TODO

float Lsro=(ro*Ls);//0.00081261//TODO make define

float RotorFluxAngle=0.0;//TODO in each cycle use RotorFluxAngle+=PI/2;??
s32fp FP_RotorFluxAngle=1;

tThreePhase abc_current(0.0,0.0,0.0);
FP_tThreePhase FP_abc_current(1,1,1);


/////////////////////////////////////////////////////////////////////////////////////////////////////////
float dy_nt_(float /*&*/y1, float /*&*/y_1){//y1 significa y[n+1] 
	return ((y1-y_1)/(2*T));}

float d2y_nt_(float /*&*/y1, float /*&*/y,float /*&*/y_1){//derived can only be obtained  when n>=? 
	return ((y1-2*y+y_1)/(T*T));}

float max_mod_volt=0.0;//max. voltage that can be applied
s32fp FP_max_mod_volt = 0;
float ang=0.0;
s32fp FP_ang = 0;
float ang_u=0.0;
s32fp FP_ang_u = 0;


void code(){

//TODO remove the following function?, non sense, i forgot the decoupling...!? and Johannes is not taking the decoupling in consideration?. .It can be with "constVDQ" if it less than max voltage...
//float calc_max_mod_volt_DQ_d(){
//
//float RotorFluxAngle_u=PI/6-RotorFluxAngle;
//
//	//if (ang>=0&&ang<(PI/6))
//	//	ang_u = ang - PI/6;//gives negative
//	//else if (ang>=PI/6 && ang<(PI/3))
//	//		ang_u = ang - PI/6;//gives positive
//	if (RotorFluxAngle_u>=0&&RotorFluxAngle_u<(PI/3))
//		ang_u = RotorFluxAngle_u - PI/6;//cos positive number == cos negative number
//	
//	else if (RotorFluxAngle_u>=PI/3 && RotorFluxAngle_u<(2*PI/3))
//			ang_u = RotorFluxAngle_u - PI/2 ;
//	//else if (ang>=PI/2 && ang<(2*PI/3))
//		//	ang_u = ang - PI/2;
//	
//	else if (RotorFluxAngle_u>=2*PI/3 && RotorFluxAngle_u<(PI))
//		//	ang_u = 5*PI/6 - ang;
//	//else if (ang>= 5*PI/6 && ang<PI)
//			ang_u = RotorFluxAngle_u - 5*PI/6;
//	
//	
//	else if (RotorFluxAngle_u>=-PI && RotorFluxAngle_u<(-2*PI/3))
//			//ang_u = ang + 5*PI/6;//gives neg
//	//else if (ang>= -5*PI/6 && ang<-2*PI/3)
//			ang_u = RotorFluxAngle_u + 5*PI/6;//gives positive	 	
//	////////////
//	else if (RotorFluxAngle_u>=-2*PI/3 && RotorFluxAngle_u<(-PI/3))
//			//ang_u = 3*PI/2 - ang;
//	//else if (ang>= 3*PI/2 && ang<5*PI/3)
//			ang_u = RotorFluxAngle_u + PI/2;
//		
//	else //if (ang>=5*PI/3 && ang<(11*PI/6))
//			//ang_u = 11*PI/6 - ang;
//	//else /*if (ang>= 11*PI/3 && ang<0)*/
//			ang_u = RotorFluxAngle_u + PI/6;
//					
//	return (VDC/(cos(ang_u)*CONST_SQRT3_));//TODO uncoment in the end cos for max 
//	fTorque<<"module voltage that can be applied DQ: "<<(VDC/(cos(ang_u)*CONST_SQRT3_))<<endl;
//
//}
}

void FOC::calc_max_mod_volt(tTwoPhase v_bi){
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
	
	
	else if (ang>=-PI && ang<(-2*PI/3))//TODO ?? ang 0, 2PI ou 0-PI -PI -> 0
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
		fTorque<<" VDC "<<VDC<<" ang_u "<<ang_u<<std::endl;			
	 max_mod_volt = VDC/(cos(ang_u)*CONST_SQRT3_);//TODO uncoment in the end cos for max 
	//exp following circulo de tensao max: 
	//max_mod_volt = VDC/(/*cos(ang_u)**/CONST_SQRT3_);
	fTorque<<"module voltage that can be applied: "<<max_mod_volt<<endl;
};

uint64_t angD = 0;
//s32fp ang = 0;
#define FP_DIGIT_TO_RADIAN(d) FP_FROMFLT((float)(d)*(PI)*2/(1<<16))
#define FP_PI_3 FP_DIV(FP_PI,FP_FROMFLT(3))
#define FP_2PI_3 (FP_PI_3<<1)
#define FP_5PI_6 FP_MUL(FP_FROMFLT(5),FP_DIV(FP_PI,FP_FROMFLT(6.0)))
#define FP_PI_6 FP_DIV(FP_PI,FP_FROMFLT(6.0))
#define FP_PI_2 (FP_PI>>1)
#define FP_4PI_3 (FP_PI_3<<2)
#define FP_5PI_3 (FP_5PI_6<<1)
void FOC::FP_calc_max_mod_volt(FP_tTwoPhase v_bi){


float aa_ = FLT_FROMFP(FP_DIV(v_bi.alpha,((s32fp)(fpsqrt((u32fp)(FP_MUL(v_bi.alpha,v_bi.alpha)+FP_MUL(v_bi.beta,v_bi.beta)))))));
float bb_ = FLT_FROMFP(FP_DIV(v_bi.beta,((s32fp)(fpsqrt((u32fp)(FP_MUL(v_bi.alpha,v_bi.alpha)+FP_MUL(v_bi.beta,v_bi.beta)))))));
int32_t ap_ =((int32_t) (aa_ *(1<<12)));
int32_t bp_ =((int32_t) (bb_ *(1<<12)));
uint16_t angD=SineCore::Atan2((ap_),(bp_));//TODO ?? estara bem?

{
//#define FRAC_FAC (1 << CST_DIGITS)

//#define FP_FROMFLT(a) ((s32fp)((a) * FRAC_FAC))
//#define FLT_FROMFP(a) (((float) (a)) / FRAC_FAC)

//+//s32fp a = FP_FROMFLT((((float) (2048))/(1<<12))*1012);
//+//s32fp b = FP_FROMFLT((((float) (-3547))/(1<<12))*1012);

//+//float aa = FLT_FROMFP(FP_DIV(a,((s32fp)(fpsqrt((u32fp)(FP_MUL(a,a)+FP_MUL(b,b)))))));

//+//float bb = FLT_FROMFP(FP_DIV(b,((s32fp)(fpsqrt((u32fp)(FP_MUL(a,a)+FP_MUL(b,b)))))));
////float aa = FLT_FROMFP(a);
////float bb = FLT_FROMFP(b);
//+//int32_t ap =((int32_t) (aa *(1<<12)));
//+//int32_t bp =((int32_t) (bb *(1<<12)));
//+//uint16_t angD_=SineCore::Atan2((ap),(bp));//TODO ?? estara bem?
//+//s32fp ang_= FP_DIGIT_TO_RADIAN((angD_));//ang>>=16;ang=FP_FROMFLT(ang);//TODO ?? estara bem?
//+//fTorque<<"angulo_testangD_"<<(angD_)<<"angulo_test_ang_"<<FLT_FROMFP(ang_)<<std::endl;

}
//angD = FP_FROMFLT((angD>>16));
FP_ang= FP_DIGIT_TO_RADIAN((angD));//ang>>=16;ang=FP_FROMFLT(ang);//TODO ?? estara bem?
fTorque<<"angulo_calc_max_mod, FP_ang: "<<FLT_FROMFP(FP_ang)<<std::endl;
//ang= FP_FROMFLT(ang);
	//float ang_u=0.0;

	//if (ang>=0&&ang<(PI/6))
	//	ang_u = ang - PI/6;//gives negative
	//else if (ang>=PI/6 && ang<(PI/3))
	//		ang_u = ang - PI/6;//gives positive
	if (FP_ang>= 0 && FP_ang<FP_PI_3)
		FP_ang_u = FP_ang - FP_PI_6 ;//cos positive number == cos negative number

	else if (FP_ang>=FP_PI_3 && FP_ang<FP_2PI_3)
			FP_ang_u = FP_ang - FP_PI_2 ;
	//else if (ang>=PI/2 && ang<(2*PI/3))
		//	ang_u = ang - PI/2;

	else if (FP_ang>=FP_2PI_3&& FP_ang<(FP_PI))
		//	ang_u = 5*PI/6 - ang;
	//else if (ang>= 5*PI/6 && ang<PI)
			FP_ang_u = FP_ang - FP_5PI_6;


	else if (FP_ang>=/*-*/FP_PI && FP_ang<(/*-2*/FP_4PI_3))
			//ang_u = ang + 5*PI/6;//gives neg
	//else if (ang>= -5*PI/6 && ang<-2*PI/3)
			FP_ang_u = FP_ang + FP_5PI_6 - (FP_PI<<1);//gives positive
	////////////
	else if (FP_ang>=/*-2*/(FP_4PI_3) && FP_ang<(/*-*/FP_5PI_3))
			//ang_u = 3*PI/2 - ang;
	//else if (ang>= 3*PI/2 && ang<5*PI/3)
			FP_ang_u = FP_ang + FP_PI_2 - (FP_PI<<1);

	else //if (ang>=5*PI/3 && ang<(11*PI/6))
			//ang_u = 11*PI/6 - ang;
	//else /*if (ang>= 11*PI/3 && ang<0)*/
			FP_ang_u = FP_ang + FP_PI_6 - (FP_PI<<1);
//TODO esta com circulo, ou seja esta seccao nao esta a fazer nada no fim descomentar a seguinte e comentar a ultima
		FP_VDC = FP_FROMFLT(VDC);//FP_FROMFLT//
if (FP_ang_u > - FP_PI && FP_ang_u < 0)
	FP_ang_u += (FP_PI<<1);
//fTorque<<" FLT_FROMFP(FP_VDC) "<<FLT_FROMFP(FP_VDC)<<" FLT_FROMFP(FP_ang_u) "<<FLT_FROMFP(FP_ang_u)<<" FLT_FROMFP(FP_CONST_SQRT3_) "<<FLT_FROMFP(FP_CONST_SQRT3_)<<std::endl;
	 FP_max_mod_volt = ABS( FP_DIV(FP_VDC,FP_MUL(FP_FROMFLT(((float) (SineCore::Cosine((uint16_t)((FLT_FROMFP(FP_ang_u)/*>>CST_DIGITS*/)*(1<<16)/2/PI))))/(1<<15)),FP_CONST_SQRT3_)));//TODO??julgo que ta errado o seguinte comentario pois ang u varia?? uncoment in the end cos for max
	 
	 max_mod_volt = FLT_FROMFP(FP_max_mod_volt);//test//
		
		if (FP_ang > FP_PI && FP_ang < (FP_2PI))//test//
			FP_ang -= (FP_PI<<1);//test//
		ang = FLT_FROMFP(FP_ang);//test//
	
	fTorque<<"FP_ang "<<FLT_FROMFP(FP_ang)<<"ang "<<ang<<std::endl;
		
	//exp following circulo de tensao max:
	//FP_max_mod_volt = FP_DIV(FP_VDC,(/*cos(FP_ang_u)**/FP_CONST_SQRT3_));
	//fTorque<<"FP_VDC "<<FLT_FROMFP(FP_VDC)<<"FP_CONST_SQRT3_ "<<FLT_FROMFP(FP_CONST_SQRT3_)<<"FP_ang_u "<<FLT_FROMFP(FP_ang_u)<<" module voltage that can be applied from FP: "<<FLT_FROMFP(FP_max_mod_volt)<<std::endl;
};

FOC::FOC():Tr_calc(Tr), abc_voltage_svpwm(0.0,0.0,0.0), FP_abc_voltage_svpwm(1,1,1),n_rtc(0),VDQ_contr(0.0,0.0),FP_IDQ(1,1), FP_VDQ(1,1), FP_VDQ_contr(1,1),FP_VDQ_ant(1,1), VDQ_ant(0.0,0.0),const_VDQ_d(0.0),const_VDQ_q(0.0)/*,angle(0.0)*//*,abc_current(0.0,0.0,0.0)*/,vel_Min_pid_res(-300),vel_Max_pid_res(300),IDQ(0.0,0.0),VDQ(0.0,0.0),IDQ_d_1(0.0),IDQ_d_(0.0),IDQ_d1(0.0),IDQ_d_p(0.0),IDQ_d_pp(0.0),IDQ_q_1(0.0),IDQ_q_(0.0),IDQ_q1(0.0),IDQ_q_p(0.0),IDQ_q_pp(0.0),wmr_(0.0),wmr1(0.0),wmr_p(0.0),v(0.0,0.0),FP_v(1,1)/*,vaa(0.0),vbb(0.0),vcc(0.0)*//*,VDQ_rtc(0.0,0.0)*/{	
	//RotFluxAng FOC::angle;
	FOC::bat.get_VDC();//TODO implement in real
	fTorque<<"valor  sqrt"<<sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))<<std::endl;
	Wn = (Vmax/(Ls*sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))));
	Wc = /*0.98**/Vmax/(Imax*Ls)*sqrt((ro*ro+1.0)/(2.0*ro*ro));//TODO acrescentei 0.9??
	
	vel.init_pid(/*0.0*/FP_FROMFLT(wr),FP_FROMFLT(w_ref) /*TODO put 0.0 at end*//*752*/,FP_FROMFLT(vel_Min_pid_res),FP_FROMFLT(vel_Max_pid_res) ,FP_FROMFLT(vel_cel));//insirir val em tune...
	vel.tune_pid(FP_FROMFLT(vel_p),FP_FROMFLT(vel_i),FP_FROMFLT(vel_d));/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*/

			//	vel.calc_pid();
	torque_control.init_pid(FP_FROMFLT(IDQ.q*3.0/2.0*np*M*M/Lr*FLT_FROMFP(angle.FP_get_imr()))/*IDQ.q*/,vel.get_pid_result(),FP_FROMFLT(torque_control_Min_pid_res),FP_FROMFLT(torque_control_Max_pid_res) ,FP_FROMFLT(torque_control_cel));
	torque_control.tune_pid(FP_FROMFLT(torque_control_p),FP_FROMFLT(torque_control_i),FP_FROMFLT(torque_control_d));

				//torque_control.calc_pid();
	current_control_q.init_pid(FP_FROMFLT(IDQ.q),torque_control.get_pid_result(),FP_FROMFLT(current_control_q_Min_pid_res),FP_FROMFLT(current_control_q_Max_pid_res) ,FP_FROMFLT(current_control_q_cel));
	current_control_q.tune_pid(FP_FROMFLT(current_control_q_p),FP_FROMFLT(current_control_q_i),FP_FROMFLT(current_control_q_d));

				//current_control_q.calc_pid();	
			
	//current_control_d.init_pid(IDQ.d,/*TODO*/Idn,current_control_d_Min_pid_res,current_control_d_Max_pid_res ,current_control_d_cel);
	//current_control_d.tune_pid(current_control_d_p,current_control_d_i,current_control_d_d);
	current_control_d.init_pid(FP_FROMFLT(IDQ.d),FP_FROMFLT(Idn),FP_FROMFLT(current_control_d_Min_pid_res),FP_FROMFLT(current_control_d_Max_pid_res) ,FP_FROMFLT(current_control_d_cel));
	current_control_d.tune_pid(FP_FROMFLT(current_control_d_p),FP_FROMFLT(current_control_d_i),FP_FROMFLT(current_control_d_d));
	
	};
	
bool FOC::exc_v_PI(){	//inside a while
				fTorque<<"torque current error: "<<FLT_FROMFP(torque_control.get_current_error())<<"torque pid result: "<<FLT_FROMFP(torque_control.get_pid_result())<<endl;//TODO remove at end
{
			//wrong if (IDQ.q>iqmax)current_control_q.set_process_point(iqmax);
			//else
			 // current_control_q.set_process_point(IDQ.q);
			//we apply variation of tension to control current!!!. iqmax is at out of torque controller
			////if ( !Flag_reset_controller_idq_q ){
				////current_control_q.set_setpoint(torque_control.get_pid_result());
			////	}
			////else{
			////	fTorque<<"reset controller_q . Last set point q: "<<current_control_q.get_this_setpoint()<<endl;
			////	current_control_q.set_setpoint(IDQ.q);
			////	current_control_q.set_this_setpoint(IDQ.q);
			////	current_control_q.set_next_setpoint(IDQ.q);
			////	current_control_q.pid_set_integral(0.0);//TODO this is to reset integral componente, runing OK??
			////	Flag_reset_controller_idq_q =false;
			////	};
		}
			current_control_q.calc_pid();

						fTorque<<"current_control_q current error: "<<FLT_FROMFP(current_control_q.get_current_error())<<"current_control_q pid result: "<<FLT_FROMFP(current_control_q.get_pid_result())<<endl;//TODO remove at end
						IDQ_d__lma<<FIXED_FLOAT(n*T/2)<<"sec.; IDQ_D_lma: "<<IDQ_d_lma<<endl<<"IDQ_d: "<<IDQ.d<<endl;//TODO remove at end
			
			////current_control_d.set_process_point(/*angle.get_imr()*/IDQ.d);// i changed: put imr instead IDQd BUT not best
			////current_control_d.set_setpoint(IDQ_d_lma);
			////current_control_d.calc_pid();
				////		fTorque<<"current_control_d current error: "<<current_control_d.get_current_error()<<"current_control_d pid result: "<<current_control_d.get_pid_result()<<endl;//TODO remove at end
						
			////VDQ.d=current_control_d.get_pid_result();//We want control currents, we apply variation of tension for that (voltage source inverter)											   
						fTorque<<"iqmax: "<<iqmax;//TODO remove at end

			VDQ_contr.q=FLT_FROMFP(current_control_q.get_pid_result())/*torque_control.get_pid_result()*/;			
			//if (VDQ.q>0 && VDQ.q>iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/))
			//	VDQ.q=iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/);
			//else if (VDQ.q<0 && VDQ.q<-iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/))
			//	VDQ.q=-iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/);
			
			/*here*/
			//VDQ_ant=VDQ;//not used
					//_	fIDQ<<FIXED_FLOAT(n*T)<<" sec.;"<<" VDQ_rtc.d: "<<VDQ_rtc.d<<endl<<" IDQd gener.: "<<IDQ.d<<" IDQq gener.: "<<IDQ.q<<endl/*<<"(IDQd^2+IDQq^2)^(1/2): "<<sqrt(IDQd*IDQd+IDQq*IDQq)*//*IDQd+IDQq*/<<"IDC:"<<IDC/*<<"IDC2_3:"<<IDC2_3*/<<endl;//TODO remove at end
				//_fIDQ<<"VDQ.d controllers:"<<VDQ_contr.d<<endl<<"VDQ.q controllers: "<<VDQ_contr.q<<endl;//TODO remove at end
				fTorque<<FIXED_FLOAT(n*T)<<" sec.;"<<" VDQ_rtc.d: "<<VDQ_rtc.d<<endl<<" IDQd generat.: "<<IDQ.d<<" IDQq gener.: "<<IDQ.q<<endl/*<<"(IDQd^2+IDQq^2)^(1/2): "<<sqrt(IDQd*IDQd+IDQq*IDQq)*//*IDQd+IDQq*/<<"IDC:"<<IDC/*<<"IDC2_3:"<<IDC2_3*/<<endl<<"VDQ.d controllers:"<<VDQ_contr.d<<endl<<"VDQ.q controllers: "<<VDQ_contr.q<<endl;//TODO remove at end

		//}
		
	//+Vdecouple-------------To get Voltage direct and quadracture after decoupling.
	//because current component .d depends also from tension component q. . Same for componente .q
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
		
			/*VDQ.d=VDQ.d+*/const_VDQ_d = (T_lag*Rs)*IDQ_d_p+T_lag*Lsro*IDQ_d_pp-wmr1*(Lsro-T_lag*Rs)*IDQ_q1+wmr1*wmr1*(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(FLT_FROMFP(angle.FP_get_imr())))-T_lag*Lsro*IDQ_q1*wmr_p;				
			/*VDQ.q=VDQ.q+*/const_VDQ_q = T_lag*Rs*IDQ_q_p + T_lag*Lsro*IDQ_q_pp + wmr1*(Lsro-T_lag*Rs)*IDQ_d1 + wmr1*wmr1*T_lag*Lsro*IDQ_q1 + (T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(FLT_FROMFP(angle.FP_get_imr())))*wmr_p + (Ls-Lsro)*wmr1*abs(FLT_FROMFP(angle.FP_get_imr()));
			
			VDQ.d=VDQ_contr.d + const_VDQ_d;
			VDQ.q=VDQ_contr.q + const_VDQ_q;
				
				//_fIDQ<<"VDQ_contr.d +decoupling: "<<VDQ.d<<endl;//TODO remove at end
				//_fIDQ<<"VDQ_contr.q +decoupling: "<<VDQ.q<<endl;//TODO remove at end
				//_fImr<<FIXED_FLOAT(n*T)<<"sec.; imr (rotor magnetization current): "<<angle.get_imr()/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;//TODO remove at end	
				fTorque<<"VDQ.d +decoupling: "<<VDQ.d<<endl<<"VDQ.q +decoupling: "<<VDQ.q<<endl<<FIXED_FLOAT(n/2*T)<<"sec.; imr (rotor magnetization current): "<<FLT_FROMFP(angle.FP_get_imr())/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;//TODO remove at end	
				
		////};
	//-------------			
		
		//if (n < 10000000)n++;//TODO:number of iterations 
		//v=InvPark((RotorFluxAngle),VDQ);//voltage to be applied //floating point//
		FP_VDQ.d=FP_FROMFLT(VDQ.d);FP_VDQ.q=FP_FROMFLT(VDQ.q);
		FP_v=FP_InvPark((FP_RotorFluxAngle),FP_VDQ);//voltage to be applied
			v.alpha = FLT_FROMFP(FP_v.alpha);v.beta = FLT_FROMFP(FP_v.beta);//test//
			fTorque<<"v.alpha "<<v.alpha<<"v.beta "<<v.beta<<"FP_v.alpha "<<FLT_FROMFP(FP_v.alpha)<<"FP_v.beta "<<FLT_FROMFP(FP_v.beta)<<std::endl;
			
				//_T1T2<<"v alpha:  "<<v.alpha<<" v beta: "<<v.beta<<" RotorFluxAngle: "<<RotorFluxAngle<<endl;//TODO remove at end
				//fTorque<<"v alpha: "<<v.alpha<<" v beta: "<<v.beta<<" RotorFluxAngle: "<<RotorFluxAngle<<endl;//TODO remove at end
  //SVPWM:---------------
	{
	//{// v tTwoPhase- bifasico, tendo como referencia o estator	
	
	//	calc_max_mod_volt(v);
	//	if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt ){
	//		float max_voltage = calc_max_mod_volt_DQ();
	//		if (VDQ.d >=  max_voltage){
	//			VDQ.d = max_voltage;//TODO create variable to eliminate 2 function calls
	//			VDQ.q = 0.0;
	//			v=InvPark((RotorFluxAngle),VDQ);
	//		}else{
	//			if VDQ.q>=0
	//				//ang_a = PI - RotorFluxAngle - PI/6; ang_b = PI - ang_a => ang_b = RotorFluxAngle + PI/6; VDQ.q = sin( RotorFluxAngle + PI/6 ) * ( max_voltage - VDQ.d ) / cos( RotorFluxAngle + PI/6 )=> VDQ.q = tg( RotorFluxAngle + PI/6 ) * ( max_voltage - VDQ.d );
	//				VDQ.q = tg( RotorFluxAngle + PI/6 ) * ( max_voltage - VDQ.d );
 	//			else if VDQ.q<0
	//					
 	//			// if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt ){
	//				
	//			}
	//		}
	//	};
}
	    //calc_max_mod_volt(v);//floating point//
	    FP_calc_max_mod_volt(FP_v);
	    fTorque<<" module volt. "<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;
		if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt /*&& !IDC_min_flag*/){
					//_T1T2 <<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					fTorque<<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
{			
			////VDQ.d=/*IDQ.d+*/const_VDQ_d;//?? use the output of controllers?
			////VDQ.q=/*IDQ.q+*/const_VDQ_q;
			//////TODO: reset controller VDQ.q. Done already??
			////Flag_reset_controller_idq_q = true;//THIS HERE or return
			////v=InvPark((RotorFluxAngle),VDQ);//voltage to be applied
			////	T1T2<<"v alpha:  "<<v.alpha<<" v beta: "<<v.beta<<" RotorFluxAngle: "<<RotorFluxAngle<<endl;//TODO remove at end
			////	fTorque<<"v alpha: "<<v.alpha<<" v beta: "<<v.beta<<" RotorFluxAngle: "<<RotorFluxAngle<<" module volt. without out. contr. "<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
 
			//////if (v.alpha>2.0/3.0*VDC || v.alpha<-2.0/3.0*VDC){	
			
			////calc_max_mod_volt(v);
			////if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt ){
			////	T1T2 <<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
			////		fTorque<<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
			////	v.beta = sin(ang)*max_mod_volt;
			////	v.alpha = cos(ang)*max_mod_volt;
			////}
			//////}
			//////else if (v.alpha<=2.0/3.0*VDC && v.alpha>VDC/3.0)
				//////{
				//////	if (v.beta>0)v.beta=(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);
				//////}
				//////else if(v.alpha>-VDC/3.0 && v.alpha<=VDC/3.0){
				//////		if (v.beta>0)v.beta=VDC/CONST_SQRT3_;else /*if(v.beta<0)*/ v.beta=-VDC/CONST_SQRT3_;
				////	//	}
				////	 //else if(v.alpha<=-VDC/3.0 && v.alpha>=-2.0/3.0*VDC){
				////		//	if (v.beta>0)v.beta=(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);
				////			//}
			//////calc_max_mod_volt(v);	  
			//////if (((v.beta*v.beta+v.alpha*v.alpha)) > max_mod_volt*max_mod_volt ){
			////	//v.beta = sin(ang)*max_mod_volt;
			////	//v.alpha = cos(ang)*max_mod_volt;
			
			//////}
		}
		return true;}//TODO THIS HERE
	return false;};

//void FOC::vel_tune_pid(){vel.tune_pid(vel_p,vel_i,vel_d);};
//void FOC::torque_control_tune_pid(){torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);};

void FOC::GetDutyCycles(float il1, float il2, float VDC,/*(float vaa,float vbb,float vcc),*/float w_ref/*commanded rotor speed:(calculations can be modified to position...and not speed*/, float wr_/*rotor speed*/)//TODO in embedded needed read values of vaa, vbb, vcc(or not needed). Can be used angle( position) instead rotor speed
{	
	//Vmax = VDC/CONST_SQRT3_;//TODO uncoment in embedded
	
	il3 = -il1 -il2;//if system simetric//TODO in real maybe better measure 3 currents
	
	//to measure previous current DC://TODO not needed to run IFOC, just adicional information
	T = T - T_R_T_F_TIMES;
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
	T = T + T_R_T_F_TIMES;
		
			
	
	
	if(n==0){//first iteration
			//PIDs----------------------			
	//		 vel.init_pid(wr_,w_ref,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
				vel.calc_pid();
		//	 torque_control.init_pid(IDQ.q*3.0/2.0*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
				torque_control.calc_pid();
			// current_control_q.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_q_Min_pid_res,current_control_q_Max_pid_res ,current_control_q_cel);
				current_control_q.calc_pid();	
			
			//current_control_d.init_pid(IDQ.d,/*TODO*/Idn,current_control_d_Min_pid_res,current_control_d_Max_pid_res ,current_control_d_cel);
			//	current_control_d.set_process_point(/*angle.get_imr()*/IDQ.d);// i changed: put imr instead IDQd BUT not best
			//current_control_d.set_setpoint(IDQ_d_lma);
				current_control_d.calc_pid();
			VDQ.q=FLT_FROMFP(current_control_q.get_pid_result());//torque_control.get_pid_result();
			VDQ.d=FLT_FROMFP(current_control_d.get_pid_result());
			//Log<<"VDQ"<<VDQ.d<<" "<<VDQ.q<<" "<<"IDQ_d_lma"<<IDQ_d_lma<<"\n";//TODO remove at end
			//+Vdecouple-------------			
			IDQ_d1=IDQ.d;
			IDQ_q1=IDQ.q;
			wmr1=FLT_FROMFP(angle.FP_get_wm());//test//
			
			v=InvPark((RotorFluxAngle),VDQ);//TODO THIS BELONG HERE?
			FP_VDQ.d=FP_FROMFLT(VDQ.d);FP_VDQ.q=FP_FROMFLT(VDQ.q);
			FP_v=FP_InvPark((FP_RotorFluxAngle),FP_VDQ);
			fTorque<<"v.alpha "<<v.alpha<<"v.beta "<<v.beta<<"FP_v.alpha "<<FLT_FROMFP(FP_v.alpha)<<"FP_v.beta "<<FLT_FROMFP(FP_v.beta)<<std::endl;
			}
	else{		vel.tune_pid(FP_FROMFLT(vel_p),FP_FROMFLT(vel_i),FP_FROMFLT(vel_d));/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*///TODO remove at end
				torque_control.tune_pid(FP_FROMFLT(torque_control_p),FP_FROMFLT(torque_control_i),FP_FROMFLT(torque_control_d));//TODO remove at end
				current_control_q.tune_pid(FP_FROMFLT(current_control_q_p),FP_FROMFLT(current_control_q_i),FP_FROMFLT(current_control_q_d));//TODO remove at end
				current_control_d.tune_pid(FP_FROMFLT(current_control_d_p),FP_FROMFLT(current_control_d_i),FP_FROMFLT(current_control_d_d));//TODO remove at end
	
			abc_current.a = il1;abc_current.b = il2;abc_current.c = il3;
			//_T1T2<<abc_current.a<<" "<<IDQ.q<<" "<<IDQ.d<<" "<<T<<" "<<Tr_calc<<" "<<(wr_*np)<<endl;
			
			//IDQ=ClarkePark(RotorFluxAngle,abc_current);//floating point//
			
			//float abc_c_a = abc_current.a, abc_c_b = abc_current.b, abc_c_c = abc_current.c;
			FP_abc_current.a = FP_FROMFLT(abc_current.a);FP_abc_current.b = FP_FROMFLT(abc_current.b);FP_abc_current.c = FP_FROMFLT(abc_current.c);
			//s32fp zzz = FP_abc_current.a;
			//fTorque<<"FLT_FROMFP(zzz) "<<FLT_FROMFP(zzz)<<std::endl;
			fTorque<<" abc_current.a "<<abc_current.a<<" abc_current.b "<<abc_current.b<<" abc_current.c "<<abc_current.c<<" FP_abc_current.a "<<(FLT_FROMFP(((s32fp)(FP_abc_current.a))))<<" FP_abc_current.b "<<(FLT_FROMFP(FP_abc_current.b))<<" FP_abc_current.c "<<(FLT_FROMFP(FP_abc_current.c))<<std::endl;
			//the following variables are used to calc rotor time constant (adapt value at change).
			fTorque<<" RotorFluxAngle "<<RotorFluxAngle<<" FP_RotorFluxAngle "<<FLT_FROMFP(FP_RotorFluxAngle)<<std::endl;
			
			FP_IDQ=FP_ClarkePark(FP_RotorFluxAngle,FP_abc_current);
			IDQ.d = FLT_FROMFP(FP_IDQ.d);IDQ.q = FLT_FROMFP(FP_IDQ.q);//test//

			abc_voltage_svpwm.a=vaa;abc_voltage_svpwm.b=vbb;abc_voltage_svpwm.c=vcc;
			
			FP_abc_voltage_svpwm.a = FP_FROMFLT(vaa);FP_abc_voltage_svpwm.b = FP_FROMFLT(vbb);FP_abc_voltage_svpwm.c = FP_FROMFLT(vcc);
//fTorque<<"FP_abc_voltage_svpwm.a "<<FLT_FROMFP(FP_abc_voltage_svpwm.a)<<"vaa "<<(vaa)<<"FP_abc_voltage_svpwm.b "<<FLT_FROMFP(FP_abc_voltage_svpwm.b)<<"vbb "<<(vbb)<<"FP_abc_voltage_svpwm.c "<<FLT_FROMFP(FP_abc_voltage_svpwm.c)<<"vcc "<<(vcc)<<std::endl;

			//TODO_: the following i dont know if its needed measure or just use the previous applied
			
			//VDQ_rtc=ClarkePark(RotorFluxAngle,abc_voltage_svpwm);	//floating point//
			FP_VDQ_rtc =	FP_ClarkePark(FP_RotorFluxAngle,FP_abc_voltage_svpwm);
			VDQ_rtc.d = FLT_FROMFP(FP_VDQ_rtc.d);//test//
			
			/*int*/ sinal_=1;
			
			//to get Rotor flux angle:
			//RotorFluxAngle-=PI/2;//TODO ??//floating point//
			FP_RotorFluxAngle-=FP_PI_2;
			//RotorFluxAngle=FOC::angle.RotFluxAng_(IDQ.q,IDQ.d,T,Tr_calc,wr_*np);//floating point//
			FP_RotorFluxAngle=FOC::angle.FP_RotFluxAng_( FP_FROMFLT(IDQ.q), FP_FROMFLT(IDQ.d), FP_FROMFLT(T), FP_FROMFLT(Tr_calc), FP_FROMFLT(wr_*np));
						
			//TODO ??//floating point//
			//RotorFluxAngle+=PI/2;//floating point//
			FP_RotorFluxAngle+=FP_PI_2;		
			RotorFluxAngle = FLT_FROMFP(FP_RotorFluxAngle);//test//

			//RotorFluxAngle_used=RotorFluxAngle-PI/2;//TODO_:-PI/2 remove? in simulation doesn't work but in real i think it's needed??
			
		//PIDs------------------------------------------- To get Voltage direct and quadracture before decoupling: 
			Wn = (Vmax/(Ls*sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))));
			Wc = /*0.98**/Vmax/(Imax*Ls)*sqrt((ro*ro+1.0)/(2.0*ro*ro));//TODO acrescentei 0.9??
			Wm = FLT_FROMFP(angle.FP_get_wm());//Wm == we, used in LMA, sincronous speed, aten.- electric speed- ..*number pole pairs	
			Rd_we=Rs+Wm*Wm*M*M/Rm;
			Rq_we=Rs+1.0/(Tr_calc*Lr)*M*M+Wm*Wm*M*M*Llr*Llr/Rm/Lr/Lr;
			IDQ_d_lma=0.0;// rotor magnetization current
			
			vel.set_process_point(FP_FROMFLT(wr_));			
			vel.set_setpoint(FP_FROMFLT(w_ref));
			
			torque_control.set_process_point(FP_FROMFLT(IDQ.q*KT*FLT_FROMFP(angle.FP_get_imr())*0.98/*cag, losses*/));
			
			
			fTorque<<"torque_process_point: "<< (IDQ.q*KT/*3/2*np*M*M/Lr*/*FLT_FROMFP(angle.FP_get_imr()))<<endl;//TODO remove at end
			
			float IDQ_d_lma__not_a_number = IDQ_d_lma ;
			
			if(Wm >0 && Wm < Wn) 
				{
				//max torque limit region:
				if (n>T_LOAD_1_sec) Tm1=TM1;/*1.07;TODO tuning1.7*/else Tm1=LOAD_1_sec;//TODO remove T in the final and calc, in real maybe this is limited by batteries//maybe limite set speed 
					 Tm=Tm1;
					 //vel.act_min_max(-Tm1,Tm1);
					 //if (SIGNAL_bat_full)
						//vel.act_min_max(0.0,Tm1);//TODO
					 //vel.calc_pid();		
						//	fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
							fTorque<<" (max torque limit region,Wm less than Wn speed) < Wn:"<<Wn<<endl;// TODO remove at end
							fTorque<<"Tm1: "<<Tm1<<endl;// TODO remove at end
				
				if(abs(IDQ.q)<=sqrt(Rd_we/Rq_we)*Idn)
					{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
					}
				else{IDQ_d_lma=Idn;
					}
				}
			else {if (Wm >0 && Wm < Wc ){
					//max current(power) limit region:
					IDQ_d_min = IDQ_D_MIN * 0.9;//TODO
					Tm2=KT*sqrt(pow(Vmax/(Wm*Ls),2.0)-Imax*Imax*ro*ro)*sqrt(Imax*Imax-pow(Vmax/(Wm*Ls),2.0))/(1.0-ro*ro);//*1.07;//TODO tuning1.7
					Tm=Tm2;
					//vel.act_min_max(-Tm2,Tm2);
					//if (SIGNAL_bat_full)
						//vel.act_min_max(0.0,Tm2);//TODO, this is to not regenerative break when full battery(negative torque 0 in controller)
					//vel.calc_pid();		
						//fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
						fTorque<<"(max current(power) limit region,Wm less than Wc,bigger than Wn), < Wc:"<<Wc<<">Wn: "<<Wn<<endl;//TODO remove at end
						fTorque<<"Tm2: "<<Tm2<<endl;//TODO remove at end
					if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
						}
					else {IDQ_d_lma=sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT)/2);
						}
					}
				else if (Wm>0){
					//max Power-speed(voltage) limit region:
					fTorque<<" KT "<<KT<<" Vmax "<<Vmax<<" Wm "<<Wm<<" Ls "<<Ls<<" ro "<<ro<<std::endl;
					Tm3=/*0.9**cag*/KT/*_t_cag*/*(pow((Vmax/(Wm*Ls)),2.0)/(2.0*ro));//*1.07;//TODO tuning1.7
					Tm=Tm3;
					IDQ_d_min = IDQ_D_MIN * 0.77;//TODO
					//vel.act_min_max(-Tm3,Tm3);
					//if (SIGNAL_bat_full)
						//vel.act_min_max(0.0,Tm3);//TODO, this is to not regenerative break when full battery(negative torque 0 in controller)
					//vel.calc_pid();		
						//fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
						fTorque<<"(max Power-speed(voltage) limit region, Wm speed bigger than Wc) > Wc:"<<Wc<<endl;//TODO remove at end
						fTorque<<"Tm3: "<<Tm3<<endl;//TODO remove at end
					if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {
						IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT))/(2.0*pow(Wm*Ls,2.0)));
						}
					}
				}
				
				if (Wm<0){
				if(Wm > -Wn) 
					{
					//max torque limit region:
					if (n>T_LOAD_1_sec) Tm1=TM1;/*1.07;TODO tuning1.7*/else Tm1=LOAD_1_sec;//TODO remove T in the final and calc, in real maybe this is limited by batteries//maybe limite set speed 
						 Tm=Tm1;
						 //vel.act_min_max(-Tm1,Tm1);
						 //if (SIGNAL_bat_full)
							//vel.act_min_max(0.0,Tm1);//TODO, this is to not regenerative break when full battery(negative torque 0 in controller)
						// vel.calc_pid();		
							//	fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
								fTorque<<" (max torque limit region,Wm less than Wn speed) < Wn:"<<Wn<<endl;// TODO remove at end
								fTorque<<"Tm1: "<<Tm1<<endl;// TODO remove at end
					
					if(abs(IDQ.q)<=sqrt(Rd_we/Rq_we)*Idn)
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
						}
					else{IDQ_d_lma=Idn;
						}
					}
				else {if (Wm > -Wc ){
					//max current(power constant) limit region:
					IDQ_d_min = IDQ_D_MIN * 0.9;//TODO 10percent less in minim. mag. current
					Tm2=KT*sqrt(pow(Vmax/(Wm*Ls),2.0)-Imax*Imax*ro*ro)*sqrt(Imax*Imax-pow(Vmax/(Wm*Ls),2.0))/(1.0-ro*ro);//*1.07;//TODO tuning1.7
					Tm=Tm2;
					//vel.act_min_max(-Tm2,Tm2);
					//if (SIGNAL_bat_full)
						//vel.act_min_max(0.0,Tm2);//TODO, this is to not regenerative break when full battery(negative torque 0 in controller)
					//vel.calc_pid();		
						//fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
						fTorque<<"(max current(power) limit region,Wm less than Wc,bigger than Wn), < Wc:"<<Wc<<">Wn: "<<Wn<<endl;//TODO remove at end
						fTorque<<"Tm2: "<<Tm2<<endl;//TODO remove at end
					if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
						}
					else {IDQ_d_lma=sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT)/2);
						}
					}
				else {
					//max Power-speed(voltage) limit region:
					IDQ_d_min = IDQ_D_MIN * 0.77;//TODO 23percent less in minim. mag. current
					fTorque<<" Wm<0  KT "<<KT<<" Vmax "<<Vmax<<" Wm "<<Wm<<" Ls "<<Ls<<" ro "<<ro<<std::endl;
					Tm3=/*0.9**cag*/KT/*_t_cag*/*(pow((Vmax/(Wm*Ls)),2.0)/(2.0*ro));//*1.07;//TODO tuning1.7
					Tm=Tm3;
					//vel.act_min_max(-Tm3,Tm3);
					//if (SIGNAL_bat_full)
						//vel.act_min_max(0.0,Tm3);//TODO, this is to not regenerative break when full battery(negative torque 0 in controller)
					//vel.calc_pid();		
						//fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
						fTorque<<"(max Power-speed(voltage) limit region, Wm speed bigger than Wc) > Wc:"<<Wc<<endl;//TODO remove at end
						fTorque<<"Tm3: "<<Tm3<<endl;//TODO remove at end
					if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {
						IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT))/(2.0*pow(Wm*Ls,2.0)));
						}
					}
				}
			}
			
			//DOC: the following is wrong, set Imax (maximum current in IGBT) and Idn (nominal magnetizing current)
			//if ( Wm < 222/*n < (10/T*2)*/ ) {
			//	Tm*=1.3;//TODO overpower!
			//	
			//	}
			
			if ( Wm < Wn/*222*//*n < (10/T*2)*/ && (primeiro == false) ){
				current_control_q.act_min_max( FP_FROMFLT(-current_control_q_Max_pid_res*1.3),FP_FROMFLT(current_control_q_Max_pid_res*1.3));
				 IDQ_D_MIN*=1.3;
				primeiro = true;
				}
			if ( Wm > Wn/*222*/ && primeiro == true) {
				primeiro = false;
				current_control_q.act_min_max( FP_FROMFLT(-current_control_q_Max_pid_res),FP_FROMFLT(current_control_q_Max_pid_res));
				 IDQ_D_MIN/=1.3;//TODO use define value instead of calc
				}
			 fTorque<<"FP -Tm "<<(-Tm)<<"FP Tm "<<(Tm)<<std::endl;
			
				//new
<<<<<<< HEAD
			 if (Wm < 160/*111*/*np && time_bin_max < TIME_BIN_MAX && time_betw_bin_max > 600.0*1.0/T && temperatur_motor < 60)  {Tm *= 1.8; IDQ_d_min = Idn ; time_bin_max++; if (time_bin_max > TIME_BIN_MAX ) {time_betw_bin_max = 0; time_bin_max = 0;} } else {Tm *= 1.06/*1.06 1.1*/;/*time_bin_max = 0*/; time_betw_bin_max++;}///1.8 *1.263*/max power. edit. much worst in power and effic.. now i will try change controller prop. gain
=======
			 if (Wm < 10/*111*/*np && time_bin_max < TIME_BIN_MAX && time_betw_bin_max > 600.0*1.0/T && temperatur_motor < 60)  {Tm *= 1.8; time_bin_max++; time_bin_max > TIME_BIN_MAX ? time_betw_bin_max = 0 : time_betw_bin_max = time_betw_bin_max; } else {Tm *= 1.06/*1.1*/;time_bin_max = 0; time_betw_bin_max++;}///1.8 *1.263*/max power. edit. much worst in power and effic.. now i will try change controller prop. gain
>>>>>>> e85ac61dde7dfb5ee4f9c8079505b6b2b3b98600
			
			vel.act_min_max(FP_FROMFLT(-Tm),FP_FROMFLT(Tm));
			if (SIGNAL_bat_full)
				vel.act_min_max(0,FP_FROMFLT(Tm));//TODO
			vel.calc_pid();		
					fTorque<<"vel current error: "<<FLT_FROMFP(vel.get_current_error())<<"vel pid result: "<<FLT_FROMFP(vel.get_pid_result())<<endl;//TODO remove at end
							
			//if (isnanf(IDQ_d_lma)){/*IDQ_d_lma=Idn;*/IDQ_d__lma<<"is not a number IDQ_d_lam "<<IDQ_d_lma<<endl;}//TODO remove at end
			  if (!(IDQ_d_lma>=IDQ_d_min && IDQ_d_lma<=Idn)){
				  /*IDQ_d_lma=Idn;*/IDQ_d__lma<<"is not a number IDQ_d_lam "<<IDQ_d_lma<<";IDQ_d_min "<<IDQ_d_min<<endl;}//TODO remove at end
			 
			//if ( isnanf(IDQ_d_lma))
			  if (!(IDQ_d_lma>=IDQ_d_min && IDQ_d_lma<=Idn)) IDQ_d_lma = IDQ_d_lma__not_a_number;//TODO ?? i think is ok
			if ((IDQ_d_lma<IDQ_d_min) ) IDQ_d_lma = IDQ_d_min;//TODO ?? i think is ok
			
				fTorque << "tc_ref:  "<< FLT_FROMFP(vel.get_pid_result()) <<endl;//TODO remove at end
			iqmax=Tm/KT/FLT_FROMFP(angle.FP_get_imr())/*0.97/*cag*/;
			torque_control.act_min_max(FP_FROMFLT(-iqmax),FP_FROMFLT(iqmax));
		
		
	{	
				//TODO, limit iqmax and idmax to IDCmax
//{		//first algoritm
//	//TODO try also with(another idea) act in controller PI with imput IDC measured, PI give the oposit if current ....
//	//IDQ=ClarkePark(RotorFluxAngle,abc_current);
//	//float sub_diff=iqmax*13.0;
//	float Iqmax=iqmax;
//	tThreePhase abc_current_possivel(0.0,0.0,0.0);
//	float IDC_est = IDC;
//	bool first_it=true;
//	float IDCmin_it=IDCmin;
//	IDC_min_flag=false;
//	while(IDC_est<IDCmin_it){
//		//Iqmax+=sub_diff;
//		IDC_min_flag=true;
//		if(first_it){IDCmin_it =IDCmin_it*0.83;first_it = false;};/*17% de caganço*/
//		Iqmax=Iqmax*0.87;
//	abc_current_possivel=InvClarkePark(RotorFluxAngle,tTwoPhaseDQ(IDQ_d_lma,-Iqmax));
//	T = T - T_R_T_F_TIMES;
//	if(a1==1&&b1==0&&c1==0&&a2==1&&b2==1&&c2==0)
//	{	IDC_est = abc_current_possivel.a/*or abc_current.a*/*T1/T - abc_current_possivel.c*T2/T;}
//	else if(a1==1&&b1==1&&c1==0&&a2==0&&b2==1&&c2==0)
//	{   IDC_est = - abc_current_possivel.c*T1/T+abc_current_possivel.b*T2/T;}
//	else if(a1==0&&b1==1&&c1==0&&a2==0&&b2==1&&c2==1)
//	{	IDC_est = abc_current_possivel.b*T1/T - abc_current_possivel.a*T2/T;}
//	else if(a1==0&&b1==1&&c1==1&&a2==0&&b2==0&&c2==1)
//	{	IDC_est = - abc_current_possivel.a*T1/T+abc_current_possivel.c*T2/T;}
//	else if(a1==0&&b1==0&&c1==1&&a2==1&&b2==0&&c2==1)
//	{	IDC_est = abc_current_possivel.c*T1/T - abc_current_possivel.b*T2/T;}
//	else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
//	{	IDC_est = - abc_current_possivel.b*T1/T+abc_current_possivel.a*T2/T;}
//	T = T + T_R_T_F_TIMES;
//	fTorque<<"IDCmin"<<IDCmin<<"IDCmin_it"<<IDCmin_it<<"IDC_estimado "<<IDC_est<<"Iqmax "<<Iqmax<<endl;
//	//todo escrever restantesif(copiar....)
//	}
//			
//			torque_control.act_min_max(-Iqmax,iqmax);
//}		
}
	//second algorithm
	//:::::::::::::::
			
	//:::::::::::::::	
	//:::::::::::::::
				
				torque_control.set_setpoint( /*set_point_previous*/ vel.get_pid_result() );
				//};//TODO WHERE THIS BELONG
			
			//DOC: the following is wrong, set Imax (maximum current in IGBT) and Idn (nominal magnetizing current)
			//if ( Wm < 222/*n < (10/T*2)*/ ) {
			//	IDQ_d_lma*=1.3;//TODO overpower!	
			//	
			//	IDQ_d__lma<< "IDQ_d_lma e IDQ_D_MIN "<< IDQ_d_lma<<" "<<IDQ_D_MIN<<endl;
			//	}
			
			current_control_d.set_process_point(/*angle.get_imr()*/FP_FROMFLT(IDQ.d));// i changed: put imr instead IDQd BUT not best
			current_control_d.set_setpoint(FP_FROMFLT(IDQ_d_lma));
			current_control_d.calc_pid();
				fTorque<<"current_control_d current error: "<<FLT_FROMFP(current_control_d.get_current_error())<<"current_control_d pid result: "<<FLT_FROMFP(current_control_d.get_pid_result())<<endl;//TODO remove at end
									
			VDQ_contr.d=FLT_FROMFP(current_control_d.get_pid_result());//We want control currents, we apply variation of tension for that (voltage source inverter)											   
		
				
				torque_control.calc_pid();
				
				 current_control_q.set_process_point(FP_FROMFLT(IDQ.q));
				//set_point_previous_v = torque_control.get_pid_result();
				//current_control_q.set_setpoint(set_point_previous_v/*torque_control.get_pid_result()*/);
			if ( IDC < IDCmin ){
				//IDC_less_than_min = true;
				set_point_previous = set_point_previous*0.82;
				current_control_q.set_setpoint( FP_FROMFLT(set_point_previous) );
				fTorque<<"reset controller_q . Last set point q: "<<FLT_FROMFP(current_control_q.get_this_setpoint())<<endl;
				}
			else{
				set_point_previous = FLT_FROMFP(torque_control.get_pid_result());
				current_control_q.set_setpoint( FP_FROMFLT(set_point_previous) );
				//IDC_less_than_min = false;
				};
				//IDC_less_than_min = false;
				
				//TODO if ( IDC > IDCmax ){
				
			it_exc_v = 1;
			while (this->exc_v_PI()/*Flag_reset_controller_idq_q*/){//if voltage exceed maximum->reduce setpoint
				//IDC_less_than_min = true;
			
				if (it_exc_v == 1){//4TODO experience
					//TODO uncoment the following
			////calc_max_mod_volt(v);
			////if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt )
			////{
			//	T1T2 <<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					fTorque<<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
				v.beta = sin(ang)*max_mod_volt;
				v.alpha = cos(ang)*max_mod_volt;
			////}
			////}
			//////else if (v.alpha<=2.0/3.0*VDC && v.alpha>VDC/3.0)
				//////{
				//////	if (v.beta>0)v.beta=(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);
				//////}
				//////else if(v.alpha>-VDC/3.0 && v.alpha<=VDC/3.0){
				//////		if (v.beta>0)v.beta=VDC/CONST_SQRT3_;else /*if(v.beta<0)*/ v.beta=-VDC/CONST_SQRT3_;
				////	//	}
				////	 //else if(v.alpha<=-VDC/3.0 && v.alpha>=-2.0/3.0*VDC){
				////		//	if (v.beta>0)v.beta=(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);
				////			//}
			//////calc_max_mod_volt(v);	  
			//////if (((v.beta*v.beta+v.alpha*v.alpha)) > max_mod_volt*max_mod_volt ){
			////	//v.beta = sin(ang)*max_mod_volt;
			////	//v.alpha = cos(ang)*max_mod_volt;
			
			//////}

					break;};
				////Flag_reset_controller_idq_q =false;//TODO this or the other
				//set_point_previous_v = set_point_previous_v*0.8;//ATention, wrong previous set in IDC< IDCmin
				current_control_q.set_setpoint( /*set_point_previous_v*/FP_FROMFLT(IDQ.q) );
					current_control_q.set_this_setpoint(FP_FROMFLT(IDQ.q));
				current_control_q.set_next_setpoint(FP_FROMFLT(IDQ.q));
				current_control_q.pid_set_integral(0);//TODO this is to reset integral componente, runing OK??
				it_exc_v++;
				//if it_exc_v == 4, is the number that is put as max. number of iterations
				fTorque<<"it_exc_v "<<it_exc_v<<" exc_v, point q: "<<FLT_FROMFP(current_control_q.get_this_setpoint())<<endl;
				};
			
		}
				
	//:::::::::::::::
		

		
		T = T - T_R_T_F_TIMES;
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
			//_T1T2<<" T1:"<<FIXED_FLOAT_L(T1)<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<FIXED_FLOAT_L(T2)<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<FIXED_FLOAT_L(T0)<<endl;//TODO remove at end
			//_T1T2<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;//TODO remove at end
			fTorque<<" T1:"<<FIXED_FLOAT_L(T1)<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<FIXED_FLOAT_L(T2)<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<FIXED_FLOAT_L(T0)<<endl<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;//TODO remove at end
			
		//DutyCycles[0] = pwm_a;
		//DutyCycles[1] = pwm_b;
		//DutyCycles[2] = pwm_c;
	//}
  //------
   	
	//--------------Rotor Time Constant value adjust:	
			
			//TODO remove at end, this is to simulate variations of Tr, the real value influences the gain error
			//	if (n_rtc>70000 ){if (Rr<=Rr_initial*0.83)incr_decr_Rr=true;
			//		if (incr_decr_Rr==true){Rr*=1.0000022/*1/1.0000002*/;}else {Rr*=0.999997;};
			//		if (Rr>Rr_initial*1.4)
			//			incr_decr_Rr=false;
			//	}
			
			if (n_rtc<90000)n_rtc++;//TODO needed or just use n. In simulation i made !!RESET WHEN CHANGED GEAR!!
			if (n_rtc>70000 ) {// 70000 TODO	
							
				/*float*/ error = VDQ_rtc.d-(Rs*IDQ.d-FLT_FROMFP(angle.FP_get_wm())*((Ls*Lr-M*M)/Lr)*IDQ.q);
				
				//the following is because it dont work at 0 power 0 speed: //steady-state 
				if ((IDQ.q<-10.5*2.5 && wr_<-/*30*/4 ) || (IDQ.q>10.5*2.5 && wr_ >4) ){
				  sinal_=1;}
				else if((IDQ.q>10.5*2.5 && wr_<-4) || (IDQ.q<-10.5*2.5 && wr_ >4 )){
						sinal_=-1;} 
						else{ sinal_=0;}
						
				Tr_calc=(Tr_calc + sinal_*error*0.000004/*0.000002, 0.000025*//*pi_contr*/);//TODO_ tune numeric value to rate of change Rr
				//_fIDQ<<" TR of motor: "<<Lr/Rr<<" Tr estimated: "<<Tr_calc/*Tr___*/<<" error: "<<error<<endl;
				fTorque<<" TR of motor: "<<Lr/Rr<<" Tr estimated: "<<Tr_calc/*Tr___*/<<"IDQ.q "<<IDQ.q<<"sinal_"<<sinal_<<" error: "<<error<<endl;
				
				} 
	//--------------			
//TODO in real, in embedded needed read the three tensions to estimate the rotor time constant value. Tension equal at ??(tension of svpwm) +	(tension of load)??		
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
		T = T + T_R_T_F_TIMES;

}
FOC::~FOC(){};
