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
//#include "my_fp_2.h"
//#include "my_math.h"
#include "ifoc.hpp"
#include "terminal.h"
#include "terminalcommands.h"

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
float torquePercent = 0.0 ;

#ifdef IFOC_ALG || VehicleSimulation
bool flag_inv = false;
#endif

#ifdef FLOATP

tThreePhase abc_current( 0.0 , 0.0 , 0.0 );
float RotorFluxAngle = 0.000001 ;//TODO in each cycle use RotorFluxAngle+=PI/2;??

float max_mod_volt = 0.0 ;//max. voltage that can be applied//TODO JAN
float ang = 0.0 ;//TODO JAN
float ang_u = 0.0;

#else
#ifdef FP
FP_tThreePhase FP_abc_current( 1 , 1 , 1 );
s32fp FP_RotorFluxAngle = 1 ;

s32fp FP_max_mod_volt = 0 ;
s32fp FP_ang = 0 ;
s32fp FP_ang_u = 0 ;
#else
tThreePhase abc_current( 0.0 , 0.0 , 0.0 );
FP_tThreePhase FP_abc_current( 1 , 1 , 1 );
float RotorFluxAngle = 0.0 ;//TODO in each cycle use RotorFluxAngle+=PI/2;??
s32fp FP_RotorFluxAngle = 1 ;

float max_mod_volt = 0.0 ;//max. voltage that can be applied//TODO JAN
s32fp FP_max_mod_volt = 0 ;

float ang = 0.0 ;//TODO JAN
float ang_u = 0.0;

s32fp FP_ang = 0 ;
s32fp FP_ang_u = 0 ;

#endif
#endif
/*TODO FP??*/float Wm = 0.0;	//synchronous speed


float Rd_we = 0.0 ;//used on loss minimization alg.
float Rq_we = 0.0 ;//used on loss minimization alg.
float IDQ_d_lma = Idn ;//to "set" rotor magnetization current. Idn-> nominal value

//float Wn = 0.0;
//float Wc = 0.0;

static float Tm1 = 0.0 ;//max. torque in max. torque limit region:
static float Tm2 = 0.0 ;//max. torque in max. current(power) limit region:
static float Tm3 = 0.0 ;//max. torque in max. Power-speed(voltage) limit region:
static float Tm = 0.0 ;//max. torque of iteration (Tm1||Tm2||Tm3)
#ifndef FLOATP
s32fp FP_sqrt_wc = FP_FROMFLT ( sqrt (( ro * ro + 1.0 ) / ( 2.0 * ro * ro )));//TODO calc value for each motor?
#endif

//static float iqmax = 0.0 ;//max. value of q component of current
static float error = 0.0 ;
static int sinal_ = 0.0 ;

//float IDC=0.0;//TODO: remove at end. to calc medium value of IDC
float vaa = 0.0001 , vbb = 0.001 , vcc = 0.00001 ;

float ids = 0.0000001 , iqs = 0.0000001 ;



//bool incr_decr_Rr ;//TODO remove at end, to vary Rr

//TODO remove at end the folloving block, replace for defines
//PI controllers values
static float vel_p = 20.0 ;////20.0;//6;//1000;//1;//1//9/ /1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO: speed controller gain
static float vel_i = 0.0009;//006 ;//0.00006;//TODO: speed controller integral gain
static float torque_control_p = 4.3;//*to use current controller */ 128 /*estava este val em torq_control:0.33,mas experimentei c 1.4;*//* e deu bons result.*//*3.6*0.124*/;//alterei*0.5 TODO: torque controller gain
static float torque_control_i = 0.00025;//0.0008;//8 /*alterei0.0009(0.001) 0.008*///TODO: torque controller integral gain
static float current_control_d_p = 1.51 / 4;//1.51 2.2 ;//mudei0.54;//0.5 a experiencia tava 0.6
static float current_control_d_i = 0.00088;//0.0004 ;//mudei0.00001;<---testes otem 0.03
static float current_control_q_p = 0.66 / 4;//0.66 1.61;//here1.56 ;//2.0*0.99/*new 1.56*//*1.4*//*0.64/*0.082 1.0*/;

static float current_control_q_Max_pid_res = 7.7 ;// 8.8//* 1.28;//1.9//*3 better ef, but bigger over tension//*1.5;//*1.29;//new TODO
static float current_control_q_Min_pid_res = -7.7 ;//* 3;//* 3 better """"";//*1.29;//new TODO doing

//#define Lsro ( ro * Ls ) //0.00081261//TODO make define

int secc=0;//TODO eliminar ist e o k esta relacionado??

/////////////////////////////////////////////////////////////////////////////////////////////////////////
float dy_nt_(float /*&*/y1, float /*&*/y_1){//y1 significa y[n+1]
	return ((y1-y_1)/(2*T));
	}

float d2y_nt_(float /*&*/y1, float /*&*/y,float /*&*/y_1){//derived can only be obtained  when n>=?
	return ((y1-2*y+y_1)/(T*T));}

//float max_mod_volt=0.0;//max. voltage that can be applied
//float ang_u=0.0;


s32fp IFOC::get_torq_setpoint(){
	return torque_control.get_setpoint();
	};

	uint64_t angD = 0;
//s32fp ang = 0;
#define FP_DIGIT_TO_RADIAN( d ) FP_FROMFLT(( float )( d ) * ( PI ) * 2 / ( 1<<16 ))
#define FP_PI_3 FP_DIV ( FP_PI , FP_FROMFLT ( 3 ))
#define FP_2PI_3 ( FP_PI_3 << 1)
#define FP_5PI_6 FP_MUL( FP_FROMFLT( 5 ) , FP_DIV( FP_PI , FP_FROMFLT( 6.0 )))
#define FP_PI_6 FP_DIV( FP_PI , FP_FROMFLT( 6.0 ))
//#define FP_PI_2 ( FP_PI >> 1 )
#define FP_4PI_3 ( FP_PI_3 << 2 )
#define FP_5PI_3 ( FP_5PI_6 << 1 )
#define FP_PI_2 ( FP_PI >> 1 )//TODO ??? porque #else nao funciona

/*
void IFOC::calc_max_mod_volt_0_1(tTwoPhase v_bi){
ang = atan2 ( v_bi.beta , v_bi.alpha );

FP_tTwoPhase FP_v_bi(FP_FROMFLT(v_bi.alpha),FP_FROMFLT(v_bi.beta));

float aa_ = FLT_FROMFP ( FP_DIV ( FP_v_bi.alpha ,(( s32fp )( fp_sqrt (( u32fp )(FP_MUL(FP_v_bi.alpha,FP_v_bi.alpha)+FP_MUL(FP_v_bi.beta,FP_v_bi.beta)))))));
float bb_ = FLT_FROMFP(FP_DIV(FP_v_bi.beta,((s32fp)(fp_sqrt((u32fp)(FP_MUL(FP_v_bi.alpha,FP_v_bi.alpha)+FP_MUL(FP_v_bi.beta,FP_v_bi.beta)))))));
int32_t ap_ =((int32_t) (aa_ *(1<<12)));
int32_t bp_ =((int32_t) (bb_ *(1<<12)));
uint16_t angD=SineCore::Atan2((ap_),(bp_));//TODO ?? estara bem?


ang= FLT_FROMFP(FP_DIGIT_TO_RADIAN ( angD ));//ang>>=16;ang=FP_FROMFLT(ang);//TODO ?? estara bem?



	if ( ang >= 0 && ang < ( PI / 3 ))
		ang_u = ang - PI / 6 ;//cos positive number == cos negative number

	else if ( ang >= PI / 3 && ang < ( 2 * PI / 3 ))
			ang_u = ang - PI / 2 ;


	else if ( ang >= 2 * PI / 3 && ang < ( PI ))

			ang_u = ang - 5 * PI / 6 ;


	else if ( ang >= -PI && ang < (-2 * PI / 3))//TODO ?? ang 0, 2PI ou 0-PI -PI -> 0

			ang_u = ang + 5 * PI / 6;//gives positive
	////////////
	else if (ang >= -2 * PI / 3 && ang < (-PI / 3))

			ang_u = ang + PI / 2;

	else
			ang_u = ang + PI / 6;
		//fTorque<<" VDC "<<VDC<<" ang_u "<<ang_u<<std::endl;
	 max_mod_volt = VDC / ( cos ( ang_u ) * CONST_SQRT3_ );//TODO uncoment in the end cos for max
*///	//exp following circulo de tensao max:
//	//max_mod_volt = VDC/(/*cos(ang_u)**/CONST_SQRT3_);
//	//fTorque<<"module voltage that can be applied: "<<max_mod_volt<<endl;
//};
//
void IFOC::calc_max_mod_volt(tTwoPhase v_bi){
//ang = atan2 ( v_bi.beta , v_bi.alpha );
/**
FP_tTwoPhase FP_v_bi(FP_FROMFLT(v_bi.alpha),FP_FROMFLT(v_bi.beta));

float aa_ = FLT_FROMFP ( FP_DIV ( FP_v_bi.alpha ,(( s32fp )( fp_sqrt (( u32fp )(FP_MUL(FP_v_bi.alpha,FP_v_bi.alpha)+FP_MUL(FP_v_bi.beta,FP_v_bi.beta)))))));
float bb_ = FLT_FROMFP(FP_DIV(FP_v_bi.beta,((s32fp)(fp_sqrt((u32fp)(FP_MUL(FP_v_bi.alpha,FP_v_bi.alpha)+FP_MUL(FP_v_bi.beta,FP_v_bi.beta)))))));
int32_t ap_ =((int32_t) (aa_ *(1<<12)));
int32_t bp_ =((int32_t) (bb_ *(1<<12)));
uint16_t angD=SineCore::Atan2((ap_),(bp_));//TODO ?? estara bem?


ang= FLT_FROMFP(FP_DIGIT_TO_RADIAN ( angD ));//ang>>=16;ang=FP_FROMFLT(ang);//TODO ?? estara bem?
**/
ang = atan2 ( v_bi.beta , v_bi.alpha );


	if ( ang >= 0 && ang < ( PI / 3 ))
		ang_u = ang - PI / 6 ;//cos positive number == cos negative number

	else if ( ang >= PI / 3 && ang < ( 2 * PI / 3 ))
			ang_u = ang - PI / 2 ;


	else if ( ang >= 2 * PI / 3 && ang < ( PI ))

			ang_u = ang - 5 * PI / 6 ;


	else if ( ang >= -PI && ang < (-2 * PI / 3))//TODO ?? ang 0, 2PI ou 0-PI -PI -> 0

			ang_u = ang + 5 * PI / 6;//gives positive
	////////////
	else if (ang >= -2 * PI / 3 && ang < (-PI / 3))

			ang_u = ang + PI / 2;

	else
			ang_u = ang + PI / 6;
		//fTorque<<" VDC "<<VDC<<" ang_u "<<ang_u<<std::endl;
	 max_mod_volt = VDC / ( cos ( ang_u ) * CONST_SQRT3_ );//TODO uncoment in the end cos for max
	//exp following circulo de tensao max:
	//max_mod_volt = VDC/(/*cos(ang_u)**/CONST_SQRT3_);
	//fTorque<<"module voltage that can be applied: "<<max_mod_volt<<endl;
};

s32fp set_point_previous_1;
float module_volt_1 = 1;
float module_volt = 1;

IFOC::IFOC():
    #ifdef FLOATP
	abc_voltage_svpwm(0.0,0.0,0.0),
	IDQ(0.00001,0.00001),
	VDQ(0.00000001,0.00000001),
	v(0.0,0.0),
	VDQ_rtc(0.0,0.0),
	#else
	#ifdef FP

	FP_abc_voltage_svpwm(1,1,1),
	FP_IDQ(1,1),
	FP_VDQ(1,1),
	FP_v(1,1),
	FP_VDQ_rtc(1,1),
	FP_VDQ_contr(1,1),
	//TODO ?? FP_VDQ_ant(1,1),
		VDQ_rtc(0.0,0.0),//TODO FP

	#else

	abc_voltage_svpwm(0.0,0.0,0.0),
	IDQ(0.0,0.0),
	VDQ(0.0,0.0),
	v(0.0,0.0),

	FP_abc_voltage_svpwm(1,1,1),
	FP_IDQ(1,1),
	FP_VDQ(1,1),
	FP_v(1,1),

	FP_VDQ_contr(1,1),
	//TODO JAN??FP_VDQ_ant(1,1),
	FP_VDQ_rtc(1,1),
	VDQ_rtc(0.0,0.0),

	#endif
	#endif
	//TODO JAN
	//angle(0.0),FP_angle(1,1)/*,abc_current(0.0,0.0,0.0)*//*,vaa(0.0),vbb(0.0),vcc(0.0)*//*


	IDQ_d_lma_avr_(Idn),IDQ_d1(0.0001),IDQ_d_1(0.00001),IDQ_d_(0.000001),IDQ_d_p(0.0000001),IDQ_d_pp(0.00000001),IDQ_q1(0.000001),IDQ_q_1(0.000001),IDQ_q_(0.000001),IDQ_q_p(0.000001),IDQ_q_pp(0.000001),/*const_VDQ_d(0.000001), const_VDQ_q(0.000001),*/ Tr_calc(Tr), /*Tr_calc_this(Tr),*/ n_rtc(0),/*TODO FP*/ VDQ_contr(0.0,0.0),/*TODO FP*/  VDQ_ant(0.0,0.0), vel_Min_pid_res(-300), vel_Max_pid_res(300),wmr_(0.0),wmr1(0.0),wmr_p(0.0)
	{
	//RotFluxAng FOC::angle;
		//for (uint i = 0; i < SIZE_AVR_FILT_LMA ; i++ ) IDQ_d_lma_v[ i ]= {Idn};
		//			// TR_v[SIZE_AVR_FILT_TR]={Tr};
		//
		//			 TR_avr = Tr * SIZE_AVR_FILT_TR;
		//		for(uint i=0; i<SIZE_AVR_FILT_TR;i++)TR_v[i]=Tr;
		//
		//		IDQ_d_lma_avr = Idn * SIZE_AVR_FILT_LMA;

	IFOC::bat.get_VDC();//TODO implement in real

	#ifndef FLOATP
//	FP_VDC = FP_FROMFLT(VDC);
	#endif

	Wn = (Vmax/(Ls*sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))));
	Wc = /*0.98**/Vmax/(Imax*Ls)*sqrt((ro*ro+1.0)/(2.0*ro*ro));//TODO acrescentei 0.9??

	//fTorque<<"valor  sqrt"<<sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))<<std::endl;

	vel.init_pid(/*0.0*/FP_FROMFLT(wr),FP_FROMFLT(w_ref) /*TODO put 0.0 at end*//*752*/,FP_FROMFLT(vel_Min_pid_res),FP_FROMFLT(vel_Max_pid_res) ,FP_FROMFLT(vel_cel));//insirir val em tune...
	vel.tune_pid(FP_FROMFLT(vel_p),FP_FROMFLT(vel_i),FP_FROMFLT(vel_d));/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*/

			//	vel.calc_pid();
#ifdef FLOATP
	torque_control.init_pid( FP_FROMFLT( /*FP_IDQ*/(IDQ.q ) * 3.0 / 2.0 * np * M * M / Lr * angle.get_imr() ) /*IDQ.q*/ , vel.get_pid_result() , FP_FROMFLT(torque_control_Min_pid_res) ,FP_FROMFLT( torque_control_Max_pid_res ) , FP_FROMFLT( torque_control_cel ) ) ;
#else
torque_control.init_pid( FP_FROMFLT( FLT_FROMFP( FP_IDQ.q ) * 3.0 / 2.0 * np * M * M / Lr * /*TODO FP FLT_FROMFP( FP_angle.FP_get_imr() )*/angle.get_imr() ) /*IDQ.q*/ , FLT_FROMFP( vel.get_pid_result()) , FP_FROMFLT(torque_control_Min_pid_res) ,FP_FROMFLT( torque_control_Max_pid_res ) , FP_FROMFLT( torque_control_cel ) ) ;
#endif
	torque_control.tune_pid( FP_FROMFLT( torque_control_p ) , FP_FROMFLT( torque_control_i ) , FP_FROMFLT( torque_control_d ) ) ;

				//torque_control.calc_pid();
#ifdef FLOATP

	current_control_q.init_pid( FP_FROMFLT(/*FP_*/IDQ.q ) , torque_control.get_pid_result() , FP_FROMFLT( current_control_q_Min_pid_res ) , FP_FROMFLT( current_control_q_Max_pid_res ) , FP_FROMFLT( current_control_q_cel ) ) ;
#else
	current_control_q.init_pid( FP_IDQ.q , torque_control.get_pid_result() , FP_FROMFLT( current_control_q_Min_pid_res ) , FP_FROMFLT( current_control_q_Max_pid_res ) , FP_FROMFLT( current_control_q_cel ) ) ;
#endif
	current_control_q.tune_pid( FP_FROMFLT( current_control_q_p ) , FP_FROMFLT( current_control_q_i ) , FP_FROMFLT( current_control_q_d ) ) ;

				//current_control_q.calc_pid();

	//current_control_d.init_pid(IDQ.d,/*TODO*/Idn,current_control_d_Min_pid_res,current_control_d_Max_pid_res ,current_control_d_cel);
	//current_control_d.tune_pid(current_control_d_p,current_control_d_i,current_control_d_d);
	#ifdef FLOATP
	current_control_d.init_pid( FP_FROMFLT(/*FP_*/IDQ.d ) , FP_FROMFLT( Idn ) , FP_FROMFLT( current_control_d_Min_pid_res ) , FP_FROMFLT( current_control_d_Max_pid_res ) , FP_FROMFLT( current_control_d_cel ) ) ;
	#else
	current_control_d.init_pid( /*FP_FROMFLT(*/FP_IDQ.d /*)*/ , FP_FROMFLT( Idn ) , FP_FROMFLT( current_control_d_Min_pid_res ) , FP_FROMFLT( current_control_d_Max_pid_res ) , FP_FROMFLT( current_control_d_cel ) ) ;
	#endif

	current_control_d.tune_pid( FP_FROMFLT( current_control_d_p ) , FP_FROMFLT( current_control_d_i ) , FP_FROMFLT( current_control_d_d ) ) ;

	}
	
	float redu = 0.96;
 const float T_lagx_Ls_less_Ls_ro = T_lag * (Ls - Ls_ro);
 const float Ls_less_Ls_ro = Ls - Ls_ro;
 const float T_lagxRs = T_lag * Rs;
 const float T_lagxLs_ro = T_lag * Ls_ro;
 const float T_lagxLs_rox2 = T_lag * Ls_ro * 2;

bool IFOC::exc_v_PI()
{		//inside a while
			//fTorque<<"torque current error: "<<FLT_FROMFP( torque_control.get_current_error() )<<"torque pid result: "<<FLT_FROMFP( torque_control.get_pid_result() )<<endl;//TODO remove at end
//TODO 030222
			current_control_q.calc_pid() ;

						//fTorque<<"current_control_q current error: "<<FLT_FROMFP(current_control_q.get_current_error())<<"current_control_q pid result: "<<FLT_FROMFP(current_control_q.get_pid_result())<<endl;//TODO remove at end
				#ifdef FLOATP
						//IDQ_d__lma<<FIXED_FLOAT(n*T/2)<<"sec.; IDQ_D_lma: "<<IDQ_d_lma<<endl<<"IDQ_d: "<<IDQ.d<<endl;//TODO remove at end
				#else
				//IDQ_d__lma<<FIXED_FLOAT(n*T/2)<<"sec.; IDQ_D_lma: "<<IDQ_d_lma<<endl<<"FP_IDQ_d: "<<FLT_FROMFP( FP_IDQ.d )<<endl;//TODO remove at end
				#endif
					//fTorque<<"iqmax: "<<iqmax;//TODO remove at end

			VDQ_contr.q = FLT_FROMFP( current_control_q.get_pid_result() )/*torque_control.get_pid_result()*/;
			#ifdef FLOATP
			//fTorque<<FIXED_FLOAT(n*T)<<" sec.;"<<" VDQ_rtc.d: "<<VDQ_rtc.d<<endl<<" FP_IDQd generat.: "<</*FP_IDQ*/(IDQ.d)<<" FP_IDQq gener.: "<</*FP_IDQ*/(IDQ.q ) << endl/*<<"(IDQd^2+IDQq^2)^(1/2): "<<sqrt(IDQd*IDQd+IDQq*IDQq)*//*IDQd+IDQq*/ << "IDC:" << IDC /*<<"IDC2_3:"<<IDC2_3*/ << endl << "VDQ.d controllers:" << VDQ_contr.d <<endl<<"VDQ.q controllers: "<<VDQ_contr.q<<endl;//TODO remove at end
	#else
	//fTorque<<FIXED_FLOAT(n*T)<<" sec.;"<<" VDQ_rtc.d: "<<VDQ_rtc.d<<endl<<" FP_IDQd generat.: "<</*FP_IDQ*/FLT_FROMFP(FP_IDQ.d)<<" FP_IDQq gener.: "<</*FP_IDQ*/FLT_FROMFP(FP_IDQ.q)<< endl/*<<"(IDQd^2+IDQq^2)^(1/2): "<<sqrt(IDQd*IDQd+IDQq*IDQq)*//*IDQd+IDQq*/ << "IDC:" << IDC /*<<"IDC2_3:"<<IDC2_3*/ << endl << "VDQ.d controllers:" << VDQ_contr.d <<endl<<"VDQ.q controllers: "<<VDQ_contr.q<<endl;//TODO remove at end
	#endif
		//}

	//+Vdecouple-------------To get Voltage direct and quadracture after decoupling.
	//because current component .d depends also from tension component q. . Same for componente .q
			IDQ_d_1 = IDQ_d_ ;
			IDQ_d_ = IDQ_d1 ;
			#ifdef FLOATP
			IDQ_d1 = /*FP_IDQ*/(IDQ.d ) ;
			#else
			//TODO FP
			IDQ_d1 = FLT_FROMFP( FP_IDQ.d );
			#endif
			//T = 0.000125;
		//	if ( ni < 20 )
		//		printf("IDQ_d_1*1000000 %f IDQ_d_*1000000 %f IDQ_d1*1000000 %f \r\n",FP_FROMFLT(IDQ_d_1*1000000) ,FP_FROMFLT(IDQ_d_*1000000), FP_FROMFLT(IDQ_d1*1000000));
			IDQ_d_p = dy_nt_( IDQ_d1 , IDQ_d_1 ) ;
			IDQ_d_pp = d2y_nt_( IDQ_d1 , IDQ_d_ , IDQ_d_1 ) ;

			IDQ_q_1 = IDQ_q_ ;
			IDQ_q_ = IDQ_q1 ;
			#ifdef FLOATP
			IDQ_q1 = /*FP_IDQ*/(IDQ.q ) ;
			#else
			//TODO FP
			IDQ_q1 = FLT_FROMFP( FP_IDQ.q );
			#endif

			IDQ_q_p = dy_nt_(	IDQ_q1, IDQ_q_1 ) ;
			IDQ_q_pp = d2y_nt_( IDQ_q1 , IDQ_q_ ,IDQ_q_1 ) ;

			wmr__1 = wmr_ ;
			wmr_ = wmr1 ;
			wmr1 = Wm ;

			wmr_p = dy_nt_( wmr1 , wmr__1 ) ;
		#ifdef FLOATP
		#ifdef NODECOUPLING
			/*VDQ.d=VDQ.d+*/const_VDQ_d = /**( T_lag * Rs ) * IDQ_d_p +**/ /**T_lag * Lsro * IDQ_d_pp**/ - wmr1 * ( Ls_ro +/**/ T_lag * Rs ) * IDQ_q1 +/**/  2 * T_lag * Ls_ro * IDQ_q_p + /**wmr1 * wmr1 * ( **//**/ /**- T_lag * Lsro * IDQ_d1 **//**-**//**/ /**T_lag * ( Ls - Lsro ) * abs( angle.get_imr() )) **/- T_lag * Ls_ro * IDQ_q1 * wmr_p ;				
			/*VDQ.q=VDQ.q+*/const_VDQ_q = /**( T_lag * Rs ) * IDQ_q_p **//**+ T_lag * Lsro * IDQ_q_pp **/+ wmr1 * ( Ls_ro +/**/ T_lag * wmr1 /**/ * Rs ) * IDQ_d1 + /**/ 2 * wmr1 * T_lag * Ls_ro * IDQ_d_p /**-**/ /**//** wmr1 * wmr1 *   T_lag * Lsro * IDQ_q1 **/+ ( T_lag * Ls_ro * IDQ_d1 /**+ T_lag * ( Ls - Lsro ) * abs( angle.get_imr() ) **/) * wmr_p /**+ ( Ls - Lsro ) * wmr1 * abs( angle.get_imr() ) **/;
		#else
		const_VDQ_q = ( T_lag * Rs ) * IDQ_q_p + T_lag * Ls_ro * IDQ_q_pp + wmr1 * ( Ls_ro +/**/ T_lag * wmr1 /**/ * Rs ) * IDQ_d1 + /**/ 2 * wmr1 * T_lag * Ls_ro * IDQ_d_p - /**/ wmr1 * wmr1 *   T_lag * Ls_ro * IDQ_q1 + ( T_lag * Ls_ro * IDQ_d1 + T_lag * ( Ls - Ls_ro ) * abs( angle.get_imr() ) ) * wmr_p + ( Ls - Ls_ro ) * wmr1 * abs( angle.get_imr() ) ;

		/*VDQ.d=VDQ.d+*/
		const_VDQ_d = ( T_lagxRs ) * IDQ_d_p ;//+ T_lagxLs_ro * IDQ_d_pp - wmr1 * ( Ls_ro + T_lagxRs ) * IDQ_q1 +  2 * T_lagxLs_ro * IDQ_q_p + wmr1 * wmr1 * (  - T_lagxLs_ro * IDQ_d1 - T_lagx_Ls_less_Ls_ro * abs( angle.get_imr() )) - T_lagxLs_ro * IDQ_q1 * wmr_p ;				
			/*VDQ.q=VDQ.q+*/
	//TODO WHY DOESNT WORK??	const_VDQ_q = ( T_lagxRs ) * IDQ_q_p + T_lagxLs_ro * IDQ_q_pp + wmr1 * ( Ls_ro +/**/ T_lagxRs * wmr1 /**/ /** Rs*/ ) * IDQ_d1 + /**/  wmr1 * T_lagxLs_rox2 * IDQ_d_p - /**/ wmr1 * wmr1 *   T_lagxLs_ro * IDQ_q1 + ( T_lagxLs_ro * IDQ_d1 + T_lagx_Ls_less_Ls_ro  * abs( angle.get_imr() ) ) * wmr_p +  Ls_less_Ls_ro  * wmr1 * abs( angle.get_imr() ) ;
//					printf("IDQ_d_p*100000 %f const_VDQ_d %f \r\n",FP_FROMFLT(IDQ_d_p*100000), FP_FROMFLT( const_VDQ_d ));

		#endif
		
			//VDQ.d=VDQ.d+
			//const_VDQ_d = ( T_lag * Rs ) * IDQ_d_p + T_lag * Lsro * IDQ_d_pp - wmr1 * ( Lsro - T_lag * Rs ) * IDQ_q1 + wmr1 * wmr1 * ( T_lag * Lsro * IDQ_d1 + T_lag * ( Ls - Lsro ) * abs( angle.get_imr() )) - T_lag * Lsro * IDQ_q1 * wmr_p ;
		//VDQ.q=VDQ.q+*
				//const_VDQ_q = T_lag * Rs * IDQ_q_p + T_lag * Lsro * IDQ_q_pp + wmr1 * ( Lsro - T_lag * Rs ) * IDQ_d1 + wmr1 * wmr1 * T_lag * Lsro * IDQ_q1 + ( T_lag * Lsro * IDQ_d1 + T_lag * ( Ls - Lsro ) * abs( angle.get_imr() ) ) * wmr_p + ( Ls - Lsro ) * wmr1 * abs( angle.get_imr() ) ;
//float az = float( float( VDQ_contr.d ) + float(const_VDQ_d) ) ;
	//float az = float( VDQ_contr.d ) ;//+ float(const_VDQ_d) ) ;
//VDQ_contr.d = VDQ_contr.d + const_VDQ_d ;
//const_VDQ_d = const_VDQ_d + VDQ_contr.d ;
			//float azzz = ( ( VDQ_contr.d ) + (const_VDQ_d) ) ;
			VDQ.d = ( ( VDQ_contr.d ) + (const_VDQ_d) ) ;
			VDQ.q = ( ( VDQ_contr.q ) + const_VDQ_q );
			//VDQ(float( float( VDQ_contr.d ) + float(const_VDQ_d) ), ( ( VDQ_contr.q ) + const_VDQ_q ) );
//			printf("azzz %f VDQ.d %f VDQ_contr.d %f \r\n",FP_FROMFLT(azzz), FP_FROMFLT( float(VDQ.d) ),FP_FROMFLT( VDQ_contr.d ));
			//fTorque<<"VDQ_contr.d +decoupling: "<<VDQ.d<<endl;//TODO remove at end
				//fTorque<<"VDQ_contr.q +decoupling: "<<VDQ.q<<endl;//TODO remove at end

		#else
			/*VDQ.d=VDQ.d+*/const_VDQ_d = ( T_lag * Rs ) * IDQ_d_p + T_lag * Lsro * IDQ_d_pp - wmr1 * ( Lsro - T_lag * Rs ) * IDQ_q1 + wmr1 * wmr1 * ( T_lag * Lsro * IDQ_d1 + T_lag * ( Ls - Lsro ) * abs( (/*FLT_FROMFP(TODO FP_*/angle.get_imr()) )) - T_lag * Lsro * IDQ_q1 * wmr_p ;
			/*VDQ.q=VDQ.q+*/const_VDQ_q = T_lag * Rs * IDQ_q_p + T_lag * Lsro * IDQ_q_pp + wmr1 * ( Lsro - T_lag * Rs ) * IDQ_d1 + wmr1 * wmr1 * T_lag * Lsro * IDQ_q1 + ( T_lag * Lsro * IDQ_d1 + T_lag * ( Ls - Lsro ) * abs((/*TODO FP FLT_FROMFP(FP_*/angle.get_imr() ) ) ) * wmr_p + ( Ls - Lsro ) * wmr1 * abs( /*FLT_FROMFP( FP_*/(angle.get_imr() ) ) ;
			//TODO FP FOLLOWING
			FP_VDQ.d = FP_FROMFLT( ( VDQ_contr.d + const_VDQ_d ) ) ;
			FP_VDQ.q = FP_FROMFLT( ( VDQ_contr.q + const_VDQ_q ) ) ;

		#endif

				//_fIDQ<<"VDQ_contr.d +decoupling: "<<VDQ.d<<endl;//TODO remove at end
				//_fIDQ<<"VDQ_contr.q +decoupling: "<<VDQ.q<<endl;//TODO remove at end
				//TODO LOG fImr<<FIXED_FLOAT(n*T/2)<<"sec.; imr (rotor magnetization current): "<<FLT_FROMFP(FP_angle.FP_get_imr())/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;//TODO remove at end
				//TODO LOGfTorque<<"VDQ.d +decoupling: "<<FLT_FROMFP(FP_VDQ.d)<<endl<<"VDQ.q +decoupling: "<<FLT_FROMFP(FP_VDQ.q)<<endl<<FIXED_FLOAT(n/2*T)<<"sec.; imr (rotor magnetization current): "<<FLT_FROMFP(FP_angle.FP_get_imr())/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;//TODO remove at end

		////};
	//-------------

		//if (n < 10000000)n++;//TODO:number of iterations
		//v=InvPark((RotorFluxAngle),VDQ);//voltage to be applied //floating point//
		#ifdef FLOATP
		
		v = InvPark( ( RotorFluxAngle ) , VDQ ) ;//voltage to be applied
		//printf("v.alpha %f ",FP_FROMFLT(v.alpha));
		#else
		#ifdef FP
		//FP_VDQ.d = FP_FROMFLT( VDQ.d ) ; FP_VDQ.q = FP_FROMFLT( VDQ.q ) ;
		FP_v = FP_InvPark( ( FP_RotorFluxAngle ) , FP_VDQ ) ;//voltage to be applied

		#else
		FP_v = FP_InvPark( ( FP_RotorFluxAngle ) , FP_VDQ ) ;//voltage to be applied
		v = InvPark( ( RotorFluxAngle ) , VDQ ) ;//voltage to be applied

		#endif
		#endif
	//SVPWM:---------------
	

		#ifdef FLOATP
	    calc_max_mod_volt(v);//floating point//
	    module_volt = sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q);
	   
	      if ( VDQ.q /**current_control_q_Min_pid_res**/ < -/**140**/max_mod_volt * redu || VDQ.q /**current_control_q_Min_pid_res**/ > /**140**/max_mod_volt * redu  )
		{ //fTorque<<"max_mod_volt_redu"<<endl;
			VDQ.q = -/**140**/max_mod_volt * redu ;
			VDQ.q = /**140**/max_mod_volt * redu ;
			
			v = InvPark( ( RotorFluxAngle ) , VDQ ) ;//voltage to be applied
		//}	return false;
		};
		#else
	    #ifdef FP
	    //TODO calc_max_mod_volt(v);//floating point//
		FP_calc_max_mod_volt(FP_v);
		//fTorque<<" module volt. "<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;
		if (((FP_MUL( FP_VDQ.d , FP_VDQ.d ) + FP_MUL( FP_VDQ.q , FP_VDQ.q ))) > FP_MUL( FP_max_mod_volt , FP_max_mod_volt ) /*&& !IDC_min_flag*/)
		{
					//_T1T2 <<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					//fTorque<<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
	    }
	    //#endif
	    #endif
	    
	    #endif
		if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > 1.1 * max_mod_volt * max_mod_volt /*&& !IDC_min_flag*/)
		{
					//_T1T2 <<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					//fTorque<<"exceeded by 1.1*:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					return true;
		}
						
		if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt /*&& !IDC_min_flag*/)
	   {
					//_T1T2 <<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					//fTorque<<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					it_exc_v = 1 ;
					return true;
		}
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
//		}
		//TODO THIS HERE
	return false;
	};



//void IFOC::vel_tune_pid(){vel.tune_pid(vel_p,vel_i,vel_d);};
//void IFOC::torque_control_tune_pid(){torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);};
static float IDQ_d_min = 60 ;//(71.0*0.7)
float IDCmin = -66 ;//-55;;
float IDCmax = 240;//new
//static bool IDC_min_flag = false ;

static int time_bin_max = 0 ;
static int time_betw_bin_max = ( int ) ( 610 * 2 * 1 / T ) ;
static float temperatur_motor = 40 ;

static int time_IDCmin = 0 ;
static int time_betw_IDCmin_max =  ( int ) ( 610 * 2 * 1 / T ) ;
static float temperatur_battery = 28 ;
const float Lls= 0.000140;
const float Ls = ( Lls + M ) ;
const s32fp FP_Ls = FP_FROMFLT ( Ls );
const s32fp FP_Idn = FP_FROMFLT( Idn );
const s32fp FP_Rs = FP_FROMFLT( Rs );
const float ro = ( 1.0 - ( M * M ) / ( Ls * Lr )) ;
const float KT = ( 3.0 / 2.0 * np * M * M / Lr ) ; 
const s32fp FP_KT = FP_FROMFLT( KT ) ; 
const float TM1 = ( KT * Idn * sqrt ( Imax * Imax - Idn * Idn ) ) ;// Maximum torque to apply in first zone
const s32fp FP_TM1 = FP_FROMFLT( TM1 ); 
const s32fp FP_ro = FP_FROMFLT ( ro ) ;
const float Ls_ro = ( Ls * ro ) ;
s32fp const FP_INV__ro_ro = FP_FROMFLT ( 1 / ( ro * ro )) ;
s32fp const FP_ro_ro = FP_FROMFLT ( ro * ro ) ;
s32fp const FP_KT__2__ro = FP_FROMFLT ( KT / 2.0 / ro) ;
float const ro_ro_Ls_Ls_Ls_Ls__KT__KT = ( ro * ro * Ls * Ls * Ls * Ls / KT / KT ) ;
s32fp const FP_Imax = FP_FROMFLT ( Imax ) ;
s32fp const FP_Imax_Imax = FP_FROMFLT( Imax * Imax ) ; //TODO remove
s32fp const FP_Imax_Imax_ro_ro = FP_FROMFLT( Imax * Imax * ro * ro ) ; // TODO !! was without FP_
float const Imax_Imax = ( Imax * Imax ) ; 
float const Imax_Imax_Imax_Imax = ( Imax * Imax * Imax * Imax ) ; 
float const Imax_Imax__2 = ( Imax_Imax / 2.0 ) ; 

static const float T_t = T - T_R_T_F_TIMES;
static const float T_orig = T ;

float const four__KT_KT = ( 4.0 / KT / KT ) ; 
s32fp const FP_CONST_Tm2 = FP_FROMFLT( KT / ( 1.0 - ro * ro ) ) ; 
float const T_LOAD_1_sec = 0.135 / T ;

float const kwn = ( Ls*sqrt(Idn*Idn+ro*ro*(Imax*Imax -Idn*Idn)));
float const kwc = (Imax*Ls) * sqrt( ( ro*ro+1.0 ) / ( 2.0*ro*ro ) );

float const Imax_Imax_ro_ro = Imax*Imax*ro*ro;
float const ro_ro = ro * ro ;
float const ro_ro_Ls_Ls_Ls_Ls = ro * ro * Ls * Ls * Ls * Ls ;
float const KT__KT = KT / KT ;

s32fp FP_iqmax = FP_FROMFLT(1);
s32fp FP_Tm = FP_FROMFLT( 100 );
static const float pow_ro__2 =  pow(ro,-2.0) ;
static const float one_less_ro_ro = (1.0 - ro_ro ) ;
static const float PI_2 = PI/2;
static const float rox2 = 2.0 * ro;
void IFOC::GetDutyCycles(float il1_, float il2_, float VDC,/*(float vaa,float vbb,float vcc),*/float w_ref/*commanded rotor speed*/, float wr_/*rotor speed*/)//TODO in embedded needed read values of vaa, vbb, vcc
{

	#ifdef FP

	FP_VDC = FP_FROMFLT(VDC);
	FP_Vmax = FP_DIV(FP_VDC,FP_CONST_SQRT3_);
	#endif 
	
	Vmax = VDC/CONST_SQRT3_;//TODO uncoment in embedded

	il3 = -il1_ -il2_ ;//if system simetric//TODO in real maybe better measure 3 currents

	//to measure previous current DC://TODO not needed to run IFOC, just adicional information
	T = T_t ;//T - T_R_T_F_TIMES;
	#ifdef VehicleSimulationFull
	if( a1 == 1 && b1 == 0 && c1 == 0 && a2 == 1 && b2 == 1 && c2 == 0 )
	{	IDC = il1_ /*or abc_current.a*/ * T1 / T - il3 * T2 / T ;}
	else if( a1 == 1 && b1 == 1 && c1 == 0 && a2 == 0 && b2 == 1 && c2 == 0 )
	{   IDC = - il3 * T1 / T + il2_ * T2 / T ;}
	else if( a1 == 0 && b1 == 1 && c1 == 0 && a2 == 0 && b2 == 1 && c2 == 1 )
	{	IDC = il2_ * T1 / T - il1_ * T2 / T ;}
	else if( a1 == 0 && b1 == 1 && c1 == 1 && a2 == 0 && b2 == 0 && c2 == 1 )
	{	IDC = - il1_ * T1 / T + il3 * T2 / T ;}
	else if( a1 == 0 && b1 == 0 && c1 == 1 && a2 == 1 && b2 == 0 && c2 == 1 )
	{	IDC = il3 * T1 / T - il2_ * T2 / T ;}
	else //(a1==1&&b1==0&&c1==1&&a2==1&&b2==0&&c2==0)
	{	IDC = - il2_ * T1 / T + il1_ * T2 / T ;}
	#endif
	T = T_orig;// + T_R_T_F_TIMES ;

	//test with errors
	//com_test float rand_= rand()%10;rand_=rand_/1000*3+1.0-0.1/3;//10/3 % error average

			//il1_*=rand_*0.997;//-0.0000000001;
//			std::cout<<rand_<<"rand"<<il1_<<"il1_"<<std::endl;
	//test with errors
	//com_test  rand_= rand()%10;rand_=rand_/1000*3+1.0-0.1/3;//10/3 % error average
			//std::cout<<rand_<<"rand"<<std::endl;
			//il2_*=rand_*0.997;//-0.000000000001;

	//test with errors
	//com_test  rand_= rand()%10/*number 0-10*/;rand_=rand_/1000*2+1.0-0.1/10;// % error average
			//std::cout<<rand_<<"rand_wr_"<<std::endl;
			//wr_=(wr_* rand_-0.3/**0.99839*/);//20arcsen de erro == 0.0001 rad


		//	std::cout<<(wr_* rand_-0.3)/wr_<<"ratio wr_ error"<<std::endl;
			//wr_=wr_* rand_*0.997;




	il3 = -il1_ -il2_ ;//TODO remove at end
	//TODO implement in real 3 measures and transforms with 3 components and omopolar

	if( ni == 0 ){//first iteration
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
			#ifdef FLOATP
			VDQ.q = FLT_FROMFP( current_control_q.get_pid_result() ) ;//torque_control.get_pid_result();
			VDQ.d = FLT_FROMFP( current_control_d.get_pid_result() ) ;
			#else
			#ifdef FP
			FP_VDQ.q = ( current_control_q.get_pid_result() ) ;//torque_control.get_pid_result();
			FP_VDQ.d = ( current_control_d.get_pid_result() ) ;
			#endif
			#endif
			//Log<<"VDQ"<<VDQ.d<<" "<<VDQ.q<<" "<<"IDQ_d_lma"<<IDQ_d_lma<<"\n";//TODO remove at end
			//+Vdecouple-------------			
			
			#ifdef FLOATP
			IDQ_d1 = /*FP_IDQ*/(IDQ.d ) ;
			IDQ_q1 = /*FP_IDQ*/(IDQ.q ) ;
			wmr1 = angle.get_wm() ;//test//
			v=InvPark((RotorFluxAngle),VDQ);//TODO THIS BELONG HERE?
			#else
			#ifdef FP
			IDQ_d1 = FLT_FROMFP( FP_IDQ.d ) ;
			IDQ_q1 = FLT_FROMFP( FP_IDQ.q ) ;
			
			wmr1 = FLT_FROMFP( FP_angle.FP_get_wm() ) ;//test//
			//FP_VDQ.d = FP_FROMFLT( VDQ.d) ; FP_VDQ.q = FP_FROMFLT( VDQ.q );
			FP_v=FP_InvPark( ( FP_RotorFluxAngle ) , FP_VDQ );
			#endif
			#endif
			
			//fTorque<<"v.alpha "<<v.alpha<<"v.beta "<<v.beta<<"FP_v.alpha "<<v.alpha<<"FP_v.beta "<<v.beta<<std::endl;
			}
	else{		
				
				//vel.tune_pid( FP_FROMFLT( vel_p ) , FP_FROMFLT( vel_i ) , FP_FROMFLT( vel_d ) ) ;/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*///TODO remove at end
				//torque_control.tune_pid( FP_FROMFLT( torque_control_p ) , FP_FROMFLT( torque_control_i ) , FP_FROMFLT( torque_control_d ) ) ;//TODO remove at end
				//current_control_q.tune_pid( FP_FROMFLT( current_control_q_p ) , FP_FROMFLT( current_control_q_i ) , FP_FROMFLT( current_control_q_d ) ) ;//TODO remove at end
				//current_control_d.tune_pid( FP_FROMFLT( current_control_d_p ) , FP_FROMFLT( current_control_d_i ) , FP_FROMFLT( current_control_d_d ) ) ;//TODO remove at end

			#ifdef FLOATP
			abc_current.a = il1_ ; abc_current.b = il2_ ; abc_current.c = il3 ;
			//_T1T2<<abc_current.a<<" "<<IDQ.q<<" "<<IDQ.d<<" "<<T<<" "<<Tr_calc<<" "<<(wr_*np)<<endl;
//			if( ni < 22 )
//				printf("il1_*1000000 %f  il2_*1000000 %f RotorFluxAngle*1000 %f \r\n",FP_FROMFLT(il1_*1000000 ), FP_FROMFLT( il2_*1000000),FP_FROMFLT(RotorFluxAngle*1000));
			IDQ=ClarkePark(RotorFluxAngle,abc_current);//floating point//
			abc_voltage_svpwm.a=vaa;abc_voltage_svpwm.b=vbb;abc_voltage_svpwm.c=vcc;
			//float abc_c_a = abc_current.a, abc_c_b = abc_current.b, abc_c_c = abc_current.c;
			#else
			#ifdef FP
			FP_abc_current.a = FP_FROMFLT( il1_ /*abc_current.a*/);FP_abc_current.b = FP_FROMFLT( il2_ /*abc_current.b*/);FP_abc_current.c = FP_FROMFLT( il3 /*abc_current.c*/);
			FP_IDQ = FP_ClarkePark( FP_RotorFluxAngle , FP_abc_current ) ;
			FP_abc_voltage_svpwm.a = FP_FROMFLT(vaa);FP_abc_voltage_svpwm.b = FP_FROMFLT(vbb);FP_abc_voltage_svpwm.c = FP_FROMFLT(vcc);
			//s32fp zzz = FP_abc_current.a;
			#else
			abc_current.a = il1_ ; abc_current.b = il2_ ; abc_current.c = il3 ;
			FP_abc_current.a = FP_FROMFLT( il1_ /*abc_current.a*/);FP_abc_current.b = FP_FROMFLT( il2_ /*abc_current.b*/);FP_abc_current.c = FP_FROMFLT( il3 /*abc_current.c*/);
			IDQ=ClarkePark(RotorFluxAngle,abc_current);//floating point//
			FP_IDQ = FP_ClarkePark( FP_RotorFluxAngle , FP_abc_current ) ;
			abc_voltage_svpwm.a=vaa;abc_voltage_svpwm.b=vbb;abc_voltage_svpwm.c=vcc;
			FP_abc_voltage_svpwm.a = FP_FROMFLT(vaa);FP_abc_voltage_svpwm.b = FP_FROMFLT(vbb);FP_abc_voltage_svpwm.c = FP_FROMFLT(vcc);
			#endif
			#endif
			//fTorque<<"FLT_FROMFP(zzz) "<<FLT_FROMFP(zzz)<<std::endl;
			//fTorque<<" abc_current.a "<<abc_current.a<<" abc_current.b "<<abc_current.b<<" abc_current.c "<<abc_current.c<<" FP_abc_current.a "<<(FLT_FROMFP(((s32fp)(FP_abc_current.a))))<<" FP_abc_current.b "<<(FLT_FROMFP(FP_abc_current.b))<<" FP_abc_current.c "<<(FLT_FROMFP(FP_abc_current.c))<<std::endl;
			//the following variables are used to calc rotor time constant (adapt value at change).
			//fTorque<<" RotorFluxAngle "<<RotorFluxAngle<<" FP_RotorFluxAngle "<<FLT_FROMFP(FP_RotorFluxAngle)<<std::endl;
			//TODO NOWIDQ = ClarkePark( RotorFluxAngle, abc_current);
			
			//TODO porque n usar vaa ... directamente
			
			//IDQ.d = FLT_FROMFP(FP_IDQ.d);IDQ.q = FLT_FROMFP(FP_IDQ.q);//test//
			
//fTorque<<"FP_abc_voltage_svpwm.a "<<FLT_FROMFP(FP_abc_voltage_svpwm.a)<<"vaa "<<(vaa)<<"FP_abc_voltage_svpwm.b "<<FLT_FROMFP(FP_abc_voltage_svpwm.b)<<"vbb "<<(vbb)<<"FP_abc_voltage_svpwm.c "<<FLT_FROMFP(FP_abc_voltage_svpwm.c)<<"vcc "<<(vcc)<<std::endl;

			//TODO_: the following i dont know if its needed measure or just use the previous applied
			#ifdef FLOATP
			VDQ_rtc = ClarkePark( RotorFluxAngle , abc_voltage_svpwm ) ;	//floating point//
			#else
			#ifdef FP
			FP_VDQ_rtc = FP_ClarkePark( FP_RotorFluxAngle , FP_abc_voltage_svpwm ) ;	//floating point//
			VDQ_rtc.d = FLT_FROMFP( FP_VDQ_rtc.d );
			VDQ_rtc.q = FLT_FROMFP( FP_VDQ_rtc.q );
			#endif
			#endif
			//VDQ_rtc.d = FLT_FROMFP(FP_VDQ_rtc.d);//test//
			
			/*int*/ sinal_=1;
			
			//to get Rotor flux angle://TODO JAN
			#ifdef FLOATP
		//	printf("IDQ.d %f \r\n", FP_FROMFLT( IDQ.d ) );
			RotorFluxAngle-=PI_2;//TODO ??//floating point//
			RotorFluxAngle=IFOC::angle.RotFluxAng_(/*FP_IDQ*/(IDQ.q)/*TODO JAN*/,/*FP_IDQ*/(IDQ.d),T,Tr_calc,wr_*np);//floating point//
			RotorFluxAngle+=PI_2;//floating point//
			#else		
			#ifdef FP
			FP_RotorFluxAngle-=FP_PI_2;
			FP_RotorFluxAngle=FOC::FP_angle.FP_RotFluxAng_( FP_IDQ.q, FP_IDQ.d, FP_FROMFLT(T), FP_FROMFLT(Tr_calc), FP_FROMFLT(wr_*np));
			//TODO ??//floating point//
			FP_RotorFluxAngle+=FP_PI_2;		
			#endif
			#endif
			//cout<<RotorFluxAngle<<" Ang "<<FLT_FROMFP(FP_RotorFluxAngle)<<endl;			
			
			
			
			//FP_RotorFluxAngle = FP_FROMFLT(RotorFluxAngle);//test//
			//RotorFluxAngle = FLT_FROMFP(FP_RotorFluxAngle);//test//
			
			

			//RotorFluxAngle_used=RotorFluxAngle-PI/2;//TODO_:-PI/2 remove? in simulation doesn't work but in real i think it's needed??
			#ifndef FLOATP
			//TODO FP??FP_RotorFluxAngle = FP_FROMFLT( RotorFluxAngle );//TODO JAN
			#endif
		//PIDs------------------------------------------- To get Voltage direct and quadracture before decoupling: 
			#ifdef FLOATP
			Wn = ( Vmax / kwn/*( Ls * sqrt( Idn * Idn + ro * ro * ( Imax * Imax - Idn * Idn )) )*/);//doing
			//exemp
//			cout<<(Imax*Imax)<<" "<<FLT_FROMFP(FP_Imax_Imax)<<endl;
			//float bb_ = FLT_FROMFP(FP_DIV(v_bi.beta,((s32fp)(fpsqrt((u32fp)(FP_MUL(v_bi.alpha,v_bi.alpha)+FP_MUL(v_bi.beta,v_bi.beta)))))));
				//com 	fTorque<<"Wn_orig"<<Wn/*sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))*/<<endl;
			Wc = /*0.98**/ Vmax / kwc/*( Imax * Ls ) * sqrt(( ro * ro + 1.0 ) /( 2.0 * ro * ro ) )*/;//TODO acrescentei 0.9??
					
					//com fTorque<<"Wc_orig"<<Wc<<endl;
			Wm = angle.get_wm();//Wm == we, used in LMA, sincronous speed, aten.- electric speed- ..*number pole pairs	

			#else
			#ifdef FP
			Wn = 0.95 * FLT_FROMFP(FP_DIV(FP_Vmax,(FP_MUL(FP_Ls,((s32fp)(fpsqrt((u32fp)(FP_MUL(FP_Idn,FP_Idn)+(FP_MUL(FP_ro_ro,(FP_FROMFLT((Imax_Imax)-FLT_FROMFP(FP_MUL(FP_Idn,FP_Idn))))))))))))));
			Wc = 0.97 * FLT_FROMFP(FP_MUL(FP_DIV(FP_Vmax,FP_MUL(FP_Imax,FP_Ls)),FP_sqrt_wc/*((s32fp)(fpsqrt((u32fp)(FP_DIV((FP_ro2+1.0),(FP_ro_ro<<1))))))*/));//TODO acrescentei 0.9??
			//com 		fTorque<<"Wc_FP"<<Wc<<endl;
			/*TODO FP???*///FP_Wm = FP_angle.FP_get_wm();
			
			//erro de cerca de 4%
					//com fTorque<<"Wn_atraves_de_FP"<<Wn/*FLT_FROMFP(((s32fp)(fpsqrt((u32fp)(FP_MUL(FP_Idn,FP_Idn)+FP_MUL(FP_ro,FP_MUL(FP_ro,(FP_MUL(FP_Imax,FP_Imax)-FP_MUL(FP_Idn,FP_Idn)))))))))*/<<endl;
			
			//algum erro tb:
			Wm = FLT_FROMFP( FP_angle.FP_get_wm() );//Wm == we, used in LMA, sincronous speed, aten.- electric speed- ..*number pole pairs	
			#endif
			#endif
			
			Rd_we = Rs + Wm * Wm * M_M__Rm ;//TODO criar constant M*M/Rm
				//com 	fTorque << "Rd_we"<<Rd_we<<"M*M"<<FIXED_FLOAT_L((M_M))<<endl;
			//Rd_we=FLT_FROMFP(FP_Rs+((FP_FROMFLT((Wm*Wm)*M_M__Rm))));//TODO pouco preciso, talvez simplesmente usar calculo todo em FLT
				//com 	fTorque << "FP Rd_we"<<Rd_we<<"FP M*M"<<FIXED_FLOAT_L(FLT_FROMFP(FP_M_M))<<endl;
			Rq_we = Rs + /*1.0/*/ M_M / ( Tr_calc * Lr) + Wm * Wm * M_M_Llr_Llr__Rm__Lr__Lr ;
			//com 		fTorque << "Rq_we"<<Rq_we<<endl;
			IDQ_d_lma = 0.0 ;// rotor magnetization current
			
			vel.set_process_point( FP_FROMFLT( wr_ ) ) ;			
			vel.set_setpoint( FP_FROMFLT( w_ref ) ) ;
			
//			float coef_loss_ = 0.025 ; //slip
	
	//wr(sentido do mov.)
	// setpoint do torque ou wm < wr ...
	
	//TODO in real maibe doesnt make sense trying so much precision
	//if ( wr > 0 angle_get_wm > np * wr ) //perdas_mec_positivas, histerese positiva, slip positiva
//	if ( wr > 0  &&  get_torq_setpoint() < 0 /*angle_get_wm < np * wr */) coef_loss_ *= -1.0;//perdas_mec_neg, histerese neg, slip neg?
	//if ( wr < 0 angle_get_wm < np * wr < 0 )//perdas_mec_positivas, histerese positiva, slip positiva
//	if ( wr < 0 && get_torq_setpoint() > 0 /*angle_get_wm > np * wr*/ ) coef_loss_ *= -1.0; //perdas_mec_neg, histerese neg, slip neg?
	//t= ( 1.0 - coef_loss ) * /* *0.975*//*<-losses mec., histerese mag., "slip" */ (3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds)));//torque generated by the tension applied
			
			// fTorque<<"torque_process_point: "<< (IDQ.q*KT/*3/2*np*M*M/Lr*/*(angle.get_imr()) * ( 1.0 - coef_loss_ ) )<<endl;//TODO remove at end
			#ifdef FLOATP
			//torque_control.set_process_point(FP_MUL(FP_MUL(FP_MUL(FP_FROMFLT(IDQ.q),FP_KT),/*3/2*np*M*M/Lr*/(angle.FP_get_imr())) , (FP_FROMFLT( 1.0 - coef_loss_ ))) /*FP_FROMFLT(IDQ.q*KT*FLT_FROMFP(angle.FP_get_imr()) * ( 1.0 - coef_loss_ )*/ /*cag, losses*/);
				torque_control.set_process_point( FP_FROMFLT(IDQ.q * KT * angle.get_imr() /** ( 1.0 - coef_loss_ )*//*cag, losses*/));
			#else
			#ifdef FP
			//com fTorque<<"FP torque_process_point: "<< FLT_FROMFP(FP_MUL( FP_MUL( FP_MUL( /*TODO NOWFP_FROMFLT(*/FP_IDQ.q , FP_KT ) , /*3/2*np*M*M/Lr*/ ( FP_angle.FP_get_imr() ) ) , ( FP_FROMFLT( 1.0 - coef_loss_ ) ) ) /*FP_FROMFLT(IDQ.q*KT*FLT_FROMFP(angle.FP_get_imr()) * ( 1.0 - coef_loss_ )*/ /*cag, losses*/); //FP_MUL(FP_MUL(FP_MUL(/*TODO NOWFP_FROMFLT(*/FP_IDQ.q,FP_KT),/*3/2*np*M*M/Lr*/FP_FROMFLT(angle.get_imr())) , (FP_FROMFLT( 1.0 - coef_loss_ ))) )<<endl;//TODO remove at end
			torque_control.set_process_point( FP_MUL( FP_MUL( FP_MUL( FP_IDQ.q , FP_KT ) , /*3/2*np*M*M/Lr*/ ( FP_angle.FP_get_imr() ) ) , ( FP_FROMFLT( 1.0 - coef_loss_ ) ) ) /*FP_FROMFLT(IDQ.q*KT*FLT_FROMFP(angle.FP_get_imr()) * ( 1.0 - coef_loss_ )*/ /*cag, losses*/);
			#endif
			#endif
				
				if ( ( Wm > -Wn ) && ( Wm <  Wn ) )
				{//max torque limit region:
				if ( ni > T_LOAD_1_sec) FP_Tm /* Tm1 */ =FP_TM1 /* TM1 */;/*1.07;TODO tuning1.7*/else  FP_Tm /* Tm1 */ = FP_FROMFLT( LOAD_1_sec );//TODO remove T in the final and calc, in real maybe this is limited by batteries//maybe limite set speed 
					 //Tm = Tm1;
 					 IDQ_d_min = IDQ_D_MIN;
 					 //TODO 030222
					 #ifdef NODECOUPLING
					 current_control_q_Min_pid_res = -11 * 13.8 ;//14 * 6 ;//60;
					 current_control_q_Max_pid_res = 11 * 13.8 ;//14 * 6 ;//60;
					 redu = 1 ;
					#else
					 current_control_q_Min_pid_res = -26;//14 * 6 ;//60;
					 current_control_q_Max_pid_res = 26;//14 * 6 ;//60;
					 redu = 1 ;
					  if ( abs( Wm ) > abs( Wn ) * 0.8 )
					 {
						current_control_q_Min_pid_res = -20 ;
					 current_control_q_Max_pid_res = 20; 
					 }; //IDQ_d_min = 50.0;
					//  else ;// IDQ_d_min = 55.6;//IDQ_D_MIN;
					 #endif
					 //vel.act_min_max(-Tm1,Tm1);
					 //if (SIGNAL_bat_full)
						//vel.act_min_max(0.0,Tm1);//TODO
					 //vel.calc_pid();		
						//	fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;//TODO remove at end
							//com fTorque<<" (max torque limit region,Wm less than Wn speed) < Wn:"<<Wn<<endl;// TODO remove at end
							//com fTorque<<"Tm1: "<<Tm1<<endl;// TODO remove at end
				
				//com fTorque<<"orig"<<sqrt(Rd_we/Rq_we)*Idn<<endl;
				
				#ifdef FP
				s32fp FP_SQRT_Rd__Rq___Idn = FP_MUL((s32fp)(fp_sqrt((u32fp)(FP_DIV(FP_FROMFLT(Rd_we),FP_FROMFLT(Rq_we))))),FP_Idn);
				#endif
				
				//com fTorque<<"FP"<<FLT_FROMFP(FP_SQRT_Rd__Rq___Idn)<<endl;
			//com 	fTorque<<"orig"<<sqrt(Rq_we/Rd_we)<<endl;
				//com fTorque<<FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Rq_we),FP_FROMFLT(Rd_we))))))<<endl;
				#ifdef FLOATP
				
				if(abs(IDQ.q)</*=*/ sqrt(Rd_we/Rq_we) * Idn)
					{IDQ_d_lma = sqrt(Rq_we/Rd_we) * abs(IDQ.q);
					}
				else{IDQ_d_lma = Idn;
					}
				
				#else
				#ifdef FP
				if( abs( /*FLT_FROMFP*/(FP_IDQ.q ) ) <= /*FLT_FROMFP*/( FP_MUL( (s32fp)(fpsqrt( (u32fp)( FP_DIV(FP_FROMFLT( Rd_we ) , FP_FROMFLT( Rq_we ) ) ) ) ) , FP_Idn ) )/*sqrt(Rd_we/Rq_we)*Idn*/)
					{ IDQ_d_lma = /*sqrt(Rq_we/Rd_we)*/FLT_FROMFP( (s32fp)(fpsqrt( (u32fp)( FP_DIV( FP_FROMFLT( Rq_we ) , FP_FROMFLT( Rd_we ) ) ) ) ) )*abs( FLT_FROMFP(FP_IDQ.q)  ) ;
					}
				else{ IDQ_d_lma = Idn ;
					}
				#endif
				#endif
				}
				else
				 if( ( ( Wm < Wc ) && ( Wm > Wn ) ) || ( ( Wm > -Wc ) && ( Wm < -Wn ) ) )
				{//max current(power) limit region:
					//IDQ_d_min = IDQ_D_MIN * 0.9;//TODO julgo que dá menos effic
					 #ifdef NODECOUPLING
					 current_control_q_Min_pid_res = -10.5 * 13.8 ;
					 current_control_q_Max_pid_res = 10.5 * 13.8 ;
					 redu = 0.987 ;
					 #else
					current_control_q_Min_pid_res = -14.4;
					 current_control_q_Max_pid_res = 14.4;
					 redu = 0.987 ;
					 //} 
					
					#endif
					#ifdef FLOATP
	
					//Tm2=KT*sqrt(pow(Vmax/(Wm*Ls),2.0)-Imax*Imax*ro*ro)*sqrt(Imax*Imax-pow(Vmax/(Wm*Ls),2.0))/(1.0-ro*ro);//*1.07;//TODO tuning1.7
					FP_Tm /* Tm2 */ = FP_FROMFLT( KT * sqrt(pow( Vmax / (Wm * Ls),2.0) - Imax_Imax_ro_ro)*sqrt( Imax_Imax - pow( Vmax / ( Wm * Ls ), 2.0)) / one_less_ro_ro/*(1.0 - ro_ro )*/ );//*1.07;//TODO tuning1.7
					
					//com fTorque <<"orig Tm2  "<<Tm2<<"  "<<KT/(1.0-ro*ro)<<"  "<<pow(Vmax/(Wm*Ls),2.0)<<" "/*<<Wm*Ls*/<<endl;
					#else
				
					#ifdef FP
					s32fp FP_Vmax__Wm_Ls = FP_DIV( FP_Vmax , FP_FROMFLT( Wm * Ls ) ) ;
					s32fp fp_ = FP_MUL( FP_Vmax__Wm_Ls , FP_Vmax__Wm_Ls ) ;
					Tm2 = FLT_FROMFP( FP_MUL( FP_MUL( FP_CONST_Tm2 , (s32fp)( fpsqrt( (u32fp)( fp_ - Imax_Imax_ro_ro ) ) ) ) , (s32fp)(fpsqrt( (u32fp)( FP_Imax_Imax - fp_ ) ) ) ) ) ;//*1.07;//TODO tuning1.7
					
					#endif
					#endif
					//fTorque<<" "/*<<FLT_FROMFP(FP_MUL(FP_Wm,FP_Ls))*/<<"  "<<FLT_FROMFP(fp_)<<"  "<<FLT_FROMFP(FP_CONST_Tm2)<<" FP_TM2 "<<FLT_FROMFP(FP_MUL(FP_MUL(FP_CONST_Tm2,(s32fp)(fpsqrt((u32fp)(fp_-Imax_Imax_ro_ro)))),(s32fp)(fpsqrt((u32fp)(FP_FROMFLT(Imax_Imax)-fp_)))))<<endl;//*1.07;//TODO tuning1.7
					
					//Tm = Tm2 ;
					if ( abs(Wm )> abs(Wn) * 1.6 ) IDQ_d_min = 50.0; else IDQ_d_min = 55.6;//IDQ_D_MIN;

					#ifdef FLOATP

					if( abs(IDQ.q) </*= */ Vmax / ( Wm * Ls_ro) /  sqrt( pow_ro__2 /* pow(ro,-2.0) */+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);
						//com fTorque<<" primeiro" <<Rq_we/Rd_we<<endl;
						}
					else {
						//IDQ_d_lma=sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0* FLT_FROMFP( vel.get_pid_result() )* FLT_FROMFP( vel.get_pid_result() )/KT/KT)/2);
						IDQ_d_lma=sqrt(Imax_Imax/2.0-sqrt(Imax_Imax_Imax_Imax-4.0* FLT_FROMFP( vel.get_pid_result() )* FLT_FROMFP( vel.get_pid_result() )/KT__KT)/2);
						
						//com fTorque<<" dois"<<(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0* FLT_FROMFP( vel.get_pid_result() )* FLT_FROMFP( vel.get_pid_result() )/KT/KT)/2)<<endl;
						}
						
					}
					
					#else	
					#ifdef FP
					if(abs( (FP_IDQ.q) ) <= (FP_DIV(FP_MUL(FP_Vmax,(s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Rd_we),FP_FROMFLT(Rq_we)))))),FP_MUL(FP_FROMFLT(Wm*Ls_ro),(s32fp)(fpsqrt((u32fp)(FP_INV__ro_ro+(FP_DIV(FP_FROMFLT(Rd_we),FP_FROMFLT(Rq_we))))))))))
						{
							//cout <<"orig "<<sqrt(Rq_we/Rd_we)*abs(IDQ.q)<<" fp "<<FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Rq_we),FP_FROMFLT(Rd_we))))))<<endl; 
							IDQ_d_lma=FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Rq_we),FP_FROMFLT(Rd_we))))));
						}
					else {
						//com fTorque<<" ultimo "<<sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT)/2)<<endl;
						//com fTorque<<FLT_FROMFP((s32fp)(fpsqrt((u32fp)FP_FROMFLT(Imax_Imax__2-(512.0*FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_FROMFLT((Imax_Imax_Imax_Imax-four__KT_KT*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result()))/262144.0))))))/2))))<<endl;
						
						IDQ_d_lma= FLT_FROMFP((s32fp)(fpsqrt((u32fp)FP_FROMFLT(Imax_Imax__2-(512.0*FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_FROMFLT((Imax_Imax_Imax_Imax-four__KT_KT*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result()))/262144.0))))))/2))));
						//sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT)/2);
						}
						}
					#endif	
					#endif
				else
				{	//max Power-speed(voltage) limit region:
					FP_Tm /* Tm3 */ = FP_FROMFLT( /*0.9**cag*/KT/*_t_cag*/ * ( pow( ( Vmax / ( Wm * Ls ) ) , 2.0 ) / ( /*2.0 * ro*/rox2 ) ) );//*1.07;//TODO tuning1.7
					//TODO 030222
					 #ifdef NODECOUPLING
					current_control_q_Min_pid_res = -6.5 * 22;//* 6 * ( 1+( max_mod_volt - module_volt ) / 30 );//3.5*0.5
					// cout<<" TM3 "<<endl;
					 current_control_q_Max_pid_res = 6.5 * 22;//* 6 * ( 1+( max_mod_volt - module_volt ) / 30 );
					redu = 0.942 ;
					 #else
					 current_control_q_Min_pid_res = -8 ;//* 6 * ( 1+( max_mod_volt - module_volt ) / 30 );//3.5*0.5
					 //cout<<" TM3 "<<endl;
					 current_control_q_Max_pid_res = 8 ;//* 6 * ( 1+( max_mod_volt - module_volt ) / 30 );
					redu = 0.942 ;
					#endif
					
					//TODO put TM3 on FP
					//Tm=Tm3;
					
					if ( abs(Wm) > abs(Wc) * 1.4 ) IDQ_d_min = 35; else IDQ_d_min = 49;  
					
					#ifdef FLOATP
					//if(abs(IDQ.q)<=Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						if(abs(IDQ.q) < /*=*/ Vmax / ( Wm * Ls_ro )/  sqrt( pow_ro__2 /* pow(ro,-2.0) */ +Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{ IDQ_d_lma = sqrt(Rq_we/Rd_we)*abs(IDQ.q);
							//com fTorque<<" um_Tm3 "<<Rq_we/Rd_we<<endl;
							}
					else {
						//IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls* FLT_FROMFP( vel.get_pid_result() ) * FLT_FROMFP( vel.get_pid_result() )/KT/KT))/(2.0*pow(Wm*Ls,2.0)));
						IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0) * ro_ro_Ls_Ls_Ls_Ls * FLT_FROMFP( vel.get_pid_result() ) * FLT_FROMFP( vel.get_pid_result() ) / KT__KT))/(2.0*pow(Wm*Ls,2.0)));
						
						//com fTorque<<" dois_Tm3 "<<(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls* FLT_FROMFP( vel.get_pid_result() ) * FLT_FROMFP( vel.get_pid_result() )/KT/KT)<<" espaço "<<((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls* FLT_FROMFP( vel.get_pid_result() ) * FLT_FROMFP( vel.get_pid_result() )/KT/KT))/(2.0*pow(Wm*Ls,2.0)))<<endl;
						}
					
					#else	
					#ifdef FP
					if(abs( ( FP_IDQ.q ) )<= (FP_DIV(FP_MUL(FP_Vmax,(s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Rd_we),FP_FROMFLT(Rq_we)))))),FP_MUL(FP_FROMFLT(Wm*Ls_ro),(s32fp)(fpsqrt((u32fp)(FP_INV__ro_ro+(FP_DIV(FP_FROMFLT(Rd_we),FP_FROMFLT(Rq_we))))))))))//Vmax/(Wm*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{
							IDQ_d_lma=/*sqrt(Rq_we/Rd_we)*/FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Rq_we),FP_FROMFLT(Rd_we))))))*abs( FLT_FROMFP( FP_IDQ.q ));
							}
					else {
						//cout <<"z "<<sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())/KT/KT))/(2.0*pow(Wm*Ls,2.0)))<<"   "<< 0.97 * FLT_FROMFP((s32fp)(fpsqrt((u32fp)(FP_DIV(FP_FROMFLT(Vmax*Vmax+FLT_FROMFP((s32fp)(fpsqrt((u32fp)FP_FROMFLT(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro_ro_Ls_Ls_Ls_Ls__KT__KT*FLT_FROMFP(vel.get_pid_result())*FLT_FROMFP(vel.get_pid_result())))))),((FP_MUL(FP_MUL(FP_Wm,FP_Ls),FP_MUL(FP_Wm,FP_Ls)))<<1))))))<<endl;
						
						//cout << Wm << " " << FP_Wm << endl ; 
						IDQ_d_lma = 0.97 * FLT_FROMFP( (s32fp)(fpsqrt( (u32fp)( FP_DIV( FP_FROMFLT( Vmax * Vmax + FLT_FROMFP( (s32fp)( fpsqrt( (u32fp)FP_FROMFLT( pow( Vmax , 4.0 ) - 4.0 * pow( Wm , 4.0 ) * ro_ro_Ls_Ls_Ls_Ls__KT__KT * FLT_FROMFP( vel.get_pid_result() ) * FLT_FROMFP( vel.get_pid_result() ) ) ) ) ) ) , ( ( FP_MUL( FP_MUL( FP_FROMFLT( Wm ) , FP_Ls ) , FP_MUL( FP_FROMFLT( Wm ) , FP_Ls ) ) ) << 1 ) ) ) ) ) ) ; }//TODO Wm... 						
					
					#endif
					#endif
			};
			current_control_q.act_min_max( FP_FROMFLT(current_control_q_Min_pid_res/**1.29**/),FP_FROMFLT(current_control_q_Max_pid_res/**1.29*/));
				
			//s32fp FP_Tm = /*FP_MUL( */FP_FROMFLT ( Tm );//, FP_FROMFLT(0.97));//FP_FROMFLT ( Tm )
			//	printf("IDQ.d %f   \r\n",FP_FROMFLT( IDQ.d ));// ,FP_FROMFLT( IDQ_d_lma ) , FP_FROMFLT((1 / T)), FP_FROMFLT(w_ref), FP_FROMFLT(((ni)*T)), FP_FROMFLT(distance_), FP_FROMFLT(wr));

			if ((Wm < 140*np && Wm > -140*np )/*111*/ && time_bin_max < TIME_BIN_MAX && time_betw_bin_max > 600.0*1.0/T && temperatur_motor < 60)
			  {	
				  ///*TODO*/ if ( FLT_FROMFP( vel.get_pid_result() )*0.9 > Tm){
					/*TODO*/ if ( FP_MUL( vel.get_pid_result() , FP_FROMFLT( 0.9 ) ) > FP_Tm )
					{	
					  //Tm *= 1.8; 
					FP_Tm = FP_MUL( FP_Tm , FP_FROMFLT( 1.8 ) ); 
					IDQ_d_min = Idn ;
					} 
				time_bin_max++;
				if (time_bin_max > TIME_BIN_MAX ) 
				{	time_betw_bin_max = 0; 
					time_bin_max = 0;} 
				}
				else 
				{/*Tm *= 1.03;*//*1.03 1.06 1.1*//*time_bin_max = 0;*/  
						 //if (Wm < Wn && Wm > -Wn)  IDQ_d_min = IDQ_D_MIN; /*TODO alterei*1.29*///: /*IDQ_d_min = IDQ_D_MIN*/;
											  //IDQ_d_min = IDQ_D_MIN; /*TODO doing alterei*1.29*///: /*IDQ_d_min = IDQ_D_MIN*/;

						 time_betw_bin_max++;
				}///1.8 *1.263*/max power. edit. much worst in power and effic.. now i will try change controller prop. gain
			
			vel.act_min_max( ( -FP_Tm ), ( FP_Tm ) );

			if (SIGNAL_bat_full){
				//vel.act_min_max(0,FP_FROMFLT(Tm));//TODO
				vel.act_min_max(FP_0, FP_Tm );//TODO

				//com fTorque<<" bat full"<<endl;
			}
			vel.calc_pid();		
				if (( IDQ_d_lma > 0.01/*IDQ_d_min*/ && IDQ_d_lma < 700/*145*/))
				{ //IDQ_d_lma = IDQ_d_lma__not_a_number;//TODO ?? i think is ok
				//IDQ_d_lma_avr = 0.0;
						IDQ_d_lma_avr -= IDQ_d_lma_v[ 0 ];
						IDQ_d_lma_avr += IDQ_d_lma ;
						
						
						for ( int i = 0 ; i < ( SIZE_AVR_FILT_LMA__1) ; i++ )
							IDQ_d_lma_v[ i ] = IDQ_d_lma_v[ i + 1 ] ;
						
						
						IDQ_d_lma_v[ SIZE_AVR_FILT_LMA__1 ] = IDQ_d_lma ;
						
			
						IDQ_d_lma_avr_ = IDQ_d_lma_avr / ( SIZE_AVR_FILT_LMA  ); 
			 
			}
			if ((IDQ_d_lma_avr_/* new lma*/ < IDQ_d_min) ) IDQ_d_lma = IDQ_d_min;//TODO ?? i think is ok
			else if ((IDQ_d_lma_avr_/* new lma*/ > Idn /** 1.04*/) ) IDQ_d_lma = Idn;//TODO ?? i think is ok
			else IDQ_d_lma = IDQ_d_lma_avr_;
			
			//com fImr << " IDQ_d_lma_avr_ " << IDQ_d_lma_avr_ << endl ;  
//			float ccv = FLT_FROMFP( vel.get_pid_result() ) ;
				//com fTorque << "tc_ref:  "<< ccv <<endl;//TODO remove at end
			//TODO FLOATP
			#ifdef FLOATP
			//float iqmax = ((( Tm )/ KT) / angle.get_imr() )*0.88/*cag*/ ;
			FP_iqmax = FP_DIV( ( FP_DIV( FP_Tm , FP_FROMFLT( KT ) ) ) , FP_FROMFLT( angle.get_imr() ) )/***0.88cag**/ ;

			torque_control.act_min_max( ( -FP_iqmax  ) , ( FP_iqmax  ) ) ;
			#else
			#ifdef FP
			FP_iqmax = FP_MUL( FP_DIV(FP_DIV( FP_FROMFLT( Tm ), FP_KT) , FP_angle.FP_get_imr() ) , FP_FROMFLT( 0.88 ) )/*cag*/ ;
			torque_control.act_min_max( ( -FP_iqmax ) , ( FP_iqmax ) ) ;
			//com fTorque<<" iqmax" << FLT_FROMFP(FP_iqmax)<<endl;//TODO LOG floatp
			#endif
			#endif
	
				//TODO, limit iqmax and idmax to IDCmax

				
				if ( flag_no_torque_motor ){
					torque_control.set_setpoint( 1 );
					flag_no_torque_motor = false ;
					//com fTorque << "set point 0.0 " << endl ;
				}
				else torque_control.set_setpoint( /*set_point_previous*/ vel.get_pid_result() );
				//};//TODO WHERE THIS BELONG
			
			//DOC: the following is wrong, set Imax (maximum current in IGBT) and Idn (nominal magnetizing current)
			//(...)
			#ifdef FLOATP
			current_control_d.set_process_point(/*angle.get_imr()*/ /*FP??*/(FP_FROMFLT( IDQ.d ) ) ) ;// i changed: put imr instead IDQd BUT not best
			#else
			#ifdef FP
			current_control_d.set_process_point(/*angle.get_imr()*/ /*FP??*/(FP_IDQ.d ) ) ;// i changed: put imr instead IDQd BUT not best
			#endif
			#endif
			current_control_d.set_setpoint( FP_FROMFLT( IDQ_d_lma) ) ;//new IDQD_dlma avr//TODO AVR
			current_control_d.calc_pid() ;
				//com fTorque<<"current_control_d current error: "<<FLT_FROMFP(current_control_d.get_current_error())<<"current_control_d pid result: "<<FLT_FROMFP(current_control_d.get_pid_result())<<endl;//TODO remove at end
	//printf("IDQ.d %f (current_control_d.get_current_error()) %f (current_control_d.get_pid_result()) %f \r\n",FP_FROMFLT( IDQ.d )),(current_control_d.get_current_error())  ,(current_control_d.get_pid_result());// ,FP_FROMFLT( IDQ_d_lma ) , FP_FROMFLT((1 / T)), FP_FROMFLT(w_ref), FP_FROMFLT(((ni)*T)), FP_FROMFLT(distance_), FP_FROMFLT(wr));
			
			VDQ_contr.d = FLT_FROMFP( current_control_d.get_pid_result() );//We want control currents, we apply variation of tension for that (voltage source inverter)											   
		//printf("VDQ_contr.d m %f \r\n",FP_FROMFLT( VDQ_contr.d ) );
				
				torque_control.calc_pid();
				#ifdef FLOATP
				 current_control_q.set_process_point( /*FP??*/( FP_FROMFLT( IDQ.q )) );

				#else
				#ifdef FP
				 current_control_q.set_process_point( /*FP??*/( FP_IDQ.q ) );
				//set_point_previous_v = torque_control.get_pid_result();
				//current_control_q.set_setpoint(set_point_previous_v/*torque_control.get_pid_result()*/);
				#endif
				#endif
				
			//TODO THERE hare a table, inputs: soc, temperature; outputs: C rate for 10 sec
			//lifepo4 72ah osn power: "After each brake charging, the battery needs to rest for certain time, wich should be equal to or longer than the duration of the pulse charging." 
			if ( IDC < IDC_MIN /* 0.92*/ /*0.82*/ && time_IDCmin < TIME_IDCMIN_MAX && time_betw_IDCmin_max > TIME_IDCMIN_MAX && temperatur_battery < 40 )
			{//second osn power time_betw_IDCmin_max >= TIME_IDCMIN_MAX
					IDCmin = IDC_MIN_MAX ; 
					time_IDCmin++ ;
					if ( time_IDCmin > TIME_IDCMIN_MAX ) 
					{
						time_betw_IDCmin_max = 0 ; time_IDCmin = 0 ;
					} 
						//com fTorque<<"IDCmin_max "<<IDCmin<<std::endl;
			} 
			else {
					IDCmin = IDC_MIN/*time_bin_max = 0*/ ;
					time_betw_IDCmin_max++ ;
					time_IDCmin-- ;//TODO right??
				 //com fTorque<<"IDCmin "<<IDCmin<<std::endl;
			}			
			
			if (  IDC < IDCmin * 0.92 )
			{
				//IDC_less_than_min = true;
				set_point_previous = FP_MUL( set_point_previous , FP_FROMFLT( 0.82 ));
				current_control_q.set_setpoint(( set_point_previous ) );
				//com fTorque<<"reset controller_q . Last set point q: "<<FLT_FROMFP(current_control_q.get_this_setpoint())<<endl;
			}
			else//
				//set_point_previous = FLT_FROMFP(torque_control.get_pid_result());
				//current_control_q.set_setpoint( FP_FROMFLT(set_point_previous) );
				//IDC_less_than_min = false ;
				//};
				// IDC_less_than_min = false ;
				
				//TODO if ( IDC > IDCmax ){
				if ( IDC > IDCmax || flag_decrease_idq_q ){
				//IDC_less_than_min = true ;
				flag_decrease_idq_q = false ;
				set_point_previous = FP_MUL( set_point_previous , FP_FROMFLT( 0.8 ));//0.82
				current_control_q.set_setpoint(( set_point_previous ) );
				//com fTorque<<"reset controller_q . Last set point q: "<<FLT_FROMFP(current_control_q.get_this_setpoint() ) << endl;
				}			
				else{
				set_point_previous = ( torque_control.get_pid_result() );
				current_control_q.set_setpoint( ( set_point_previous ) );
				//IDC_less_than_min = false;
				};
							
			it_exc_v = 6;
			while (this->exc_v_PI()/*Flag_reset_controller_idq_q*/)
			{//if voltage exceed maximum->reduce setpoint
				//IDC_less_than_min = true;
			
				if ( it_exc_v == 1 ){//4TODO experience
					//TODO uncoment the following
			////calc_max_mod_volt(v);
			////if (((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_volt*max_mod_volt )
			////{
			//	T1T2 <<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
					//TODO FP
					//fTorque<<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
				//flag_decrease_idq_q = true;//TODO
				#ifdef FLOATP
				//com fTorque<<"second exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;//TODO remove at end
				v.beta = sin(ang)*max_mod_volt;
				v.alpha = cos(ang)*max_mod_volt;
			
				#else
				#ifdef FP
				FP_v.beta = FP_MUL( FP_FROMFLT(((float) (SineCore::Sine((uint16_t)((FLT_FROMFP(FP_ang)/*>>CST_DIGITS*/)*(1<<16)/2/PI))))/(1<<15)) , FP_max_mod_volt );

				FP_v.alpha = FP_MUL( FP_FROMFLT(((float) (SineCore::Cosine((uint16_t)((FLT_FROMFP(FP_ang)/*>>CST_DIGITS*/)*(1<<16)/2/PI))))/(1<<15)) , FP_max_mod_volt );
				#endif
				#endif
				{
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
		}

					break;
					};
				set_point_previous = FP_MUL( set_point_previous , FP_FROMFLT(0.4/*66*/));//ATention, wrong previous set in IDC< IDCmin
				
				////Flag_reset_controller_idq_q =false;//TODO this or the other
				//set_point_previous_v = set_point_previous_v*0.8;//ATention, wrong previous set in IDC< IDCmin
				#ifdef FLOATP
				current_control_q.set_setpoint(/** FP_0**/ set_point_previous/*FP_FROMFLT( IDQ.q ) */ );
					current_control_q.set_this_setpoint( /** FP_0**/ set_point_previous/*FP_FROMFLT( IDQ.q ) */ );
				current_control_q.set_next_setpoint(/** FP_0**/ set_point_previous/*FP_FROMFLT( IDQ.q ) */);
				#else
				
				current_control_q.set_setpoint( /*set_point_previous_v*/ FP_IDQ.q  );//TODO put set poin t previos 
					current_control_q.set_this_setpoint( FP_IDQ.q  );
				current_control_q.set_next_setpoint( FP_IDQ.q  );
				#endif
				current_control_q.pid_set_integral(FP_0);//TODO this is to reset integral componente, runing OK??
				it_exc_v--;
				//if it_exc_v == 4, is the number that is put as max. number of iterations
				//com fTorque<<"it_exc_v "<<it_exc_v<<" exc_v, point q: "<<FLT_FROMFP(current_control_q.get_this_setpoint())<<endl;
			};
			
	
			
	//:::::::::::::::


/*
TODO doc
<--------------->
        Ts
<----><-----><-->
  T1     T2   T0
a1=1||0 a2=1||0    all closed(T0)
b1=1||0 b2=1||0
c1=1||0 c2=1||0

  a,b,c: upper legs of triphasic bridge
         down  ""  ""    ""       "     inverse open (if a1 = 1 down leg=0 ....)
  */
		T = T_t;// - T_R_T_F_TIMES;
		//printf("v.alpha %f vaa %f IDQ.d %f   \r\n",FP_FROMFLT(v.alpha), FP_FROMFLT(vaa), FP_FROMFLT( IDQ.d ));// ,FP_FROMFLT( IDQ_d_lma ) , FP_FROMFLT((1 / T)), FP_FROMFLT(w_ref), FP_FROMFLT(((ni)*T)), FP_FROMFLT(distance_), FP_FROMFLT(wr));

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

		//TODO JAN FOLLOWING:
				//if (imr  > 0.03 colocar pwm a 0. no fim, antes de ligar novamente set wr a 0( ou nao sera preciso)
//		 if (1==2/*!!!!!!!!!!*/ && ( /*n > (26*2*1/T) ||*/ (Rr_est< ( Rr / 1.37 )) || (Rr_est> ( Rr * 1.37 )) ) )
//		 {////reset sitem 60 sec
//			 Rr_est = Rr ;
//			 Tr_calc_this = Tr ;
			 //fImr<<"reset"<<" ids "<<ids<<" iqs "<<iqs<<endl;
			 //T1=0;
			 //T2=0;
			 //if(angle.get_imr()  < 1.0 && n > 3*2*1/T)
//			 {
				// n=0;
				//wr=0;
//				}
//			}//colocar pwm a 0. no fim, antes de ligar novamente set wr a 0( ou nao sera preciso)


		 T0 = T - ( T1 + T2 ) ;
			//_T1T2<<" T1:"<<FIXED_FLOAT_L(T1)<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<FIXED_FLOAT_L(T2)<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<FIXED_FLOAT_L(T0)<<endl;//TODO remove at end
			//_T1T2<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;//TODO remove at end
			//fTorque<<" T1:"<<FIXED_FLOAT_L(T1)<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<FIXED_FLOAT_L(T2)<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<FIXED_FLOAT_L(T0)<<endl<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;//TODO remove at end

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

			if (n_rtc<90000) n_rtc++ ;//TODO needed or just use n. In simulation i made !!RESET WHEN CHANGED GEAR!!
			if (n_rtc>20000 ) {// 70000 TODO	
							
				
				//the following is because it dont work at 0 power 0 speed: //steady-state 
				#ifdef FLOATP
				/*float*/ error = VDQ_rtc.d - ( Rs * /*FP_IDQ*/(IDQ.d ) - ( ( angle.get_wm() )) * ( ( Ls * Lr -M * M ) / Lr ) * /*FP_IDQ*/(IDQ.q )) ;
				
				if (( /*FP_IDQ*/(IDQ.q ) < -10.5 * 2.5 && wr_ < -4/*30*/ ) || ( /*FP_IDQ*/(IDQ.q ) > 10.5 * 2.5 && wr_ > 4 ) ){  
				  sinal_= 1 ;}
				else if( ( /*FP_IDQ*/(IDQ.q ) > 10.5 * 2.5 && wr_ < -4 ) || (/*FP_IDQ*/(IDQ.q ) < -10.5 * 2.5 && wr_ > 4 )){
				
				#else
				#ifdef FP
				/*float*/ error = VDQ_rtc.d - ( Rs * /*FP_IDQ*/FLT_FROMFP(FP_IDQ.d ) - ( FLT_FROMFP( FP_angle.FP_get_wm() )) * ( ( Ls * Lr -M * M ) / Lr ) * /*FP_IDQ*/FLT_FROMFP(FP_IDQ.q )) ;
				
				if (( /*FP_IDQ*/FLT_FROMFP(FP_IDQ.q ) < -10.5 * 2.5 && wr_ < -4/*30*/ ) || ( /*FP_IDQ*/FLT_FROMFP(FP_IDQ.q ) > 10.5 * 2.5 && wr_ > 4 ) ){  
				  sinal_= 1 ;}
				else if( ( /*FP_IDQ*/FLT_FROMFP(FP_IDQ.q ) > 10.5 * 2.5 && wr_ < -4 ) || (/*FP_IDQ*/FLT_FROMFP(FP_IDQ.q ) < -10.5 * 2.5 && wr_ > 4 )){
				#endif
				#endif
						sinal_ = -1 ;} 
						else{ sinal_ = 0 ;}
						
				 Tr_calc/*_this*/ = ( Tr_calc/*_this*/ + sinal_ * error * 0.000000003 /* 4->44   0.000002, 0.000025*//*pi_contr*/) ;//TODO_ tune numeric value to rate of change Rr
				
	
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
					//A conclusao a que chego em relaçao ao COMPENSADOR de TR, não faz sentido usar valores medios pois julgo que tambem compensa outros eventuais erros, por exemplo de medida
	//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

				//Tr_calc = Tr_calc_this ;//TODO needed 2 variables??
				//if ( Tr_calc < 2.0 / 3.0 * Tr || Tr_calc > 5.0 / 3.0 * Tr ) Tr_calc = Tr;
				Rr_est = Lr / Tr_calc;//TODO !!! atention this needs be here
				#ifdef FP
				FP_Rr_est = FP_FROMFLT(Rr_est);
				#endif
}
	//--------------
//TODO in real, in embedded needed read the three tensions to estimate the rotor time constant value. Tension equal at ??(tension of svpwm) +	(tension of load)??
//tension that will be applied next, aproximation
//TODO ==
//abc_voltage=InvClarke(v);//next == at this
	 
		#ifdef VehicleSimulationFull//TODO IN SIMULATION OF IFOC, uncomment
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

#endif

//--------
		T = T_orig;//T + T_R_T_F_TIMES;


}
};


IFOC::~IFOC(){};

