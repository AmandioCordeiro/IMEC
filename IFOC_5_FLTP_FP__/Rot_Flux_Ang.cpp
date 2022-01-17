//------------------------------------------------------------------------------
//   Tumanako - Electric Vehicle software
//   Copyright (C) 2012 Bernard Mentink <bmentink@gmail.com>
//   Copyright (C) 2011 Graham Osborne <gjoengineer@users.sourceforge.net>
//
//  This file is part of Tumanako_QP.
//
// 	This software may be distributed and modified under the terms of the GNU
// 	General Public License version 2 (GPL) as published by the Free Software
// 	Foundation and appearing in the file GPL.TXT included in the packaging of
// 	this file. Please note that GPL Section 2[b] requires that all works based
// 	on this software must also be made publicly available under the terms of
// 	the GPL ("Copyleft").
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// DESCRIPTION:
//   This file contains some simple C code for calculating slip angle from the torque and flux genartaing currents
//	 The rotor time constant and PWM period are also required.
//	 The method used is from Richard Valentines book "Motor Control Electronics Handbook"
// 	 It is described here - sourceforge.net/apps/mediawiki/tumanako/index.php?title=Field_Oriented_Control#Calculating_Slip_Angle
//
//	 Parameters required
//   iqs - torque generating current
//   ids - flux generating current
//   sPWM - PWM loop period
//   tRotor - Rotor Time Constant
//
//	 Variables
//   OmegaSlip - slip frequency (rad/sec)
//   ThetaSlip - flux slip angle (rad)
//
// HISTORY:
//   Graham Osborne 21/9/2011 - First Cut in C (needs refinement!)
//   Bernard Mentink 28/3/2012 - ported to C++, included in Tumanko_QP project.
//
// Usage:
//
//  call slip angle calculation each PWM loop period
//	Angle = CalcSlipAngle (iqs,ids,sPWM,tRotor);
//------------------------------------------------------------------------------

//#include "bsp.h"//TODO
#include "Rot_Flux_Ang.hpp"
#include <iostream>
#include <stdio.h>
//#define FLOATP 1


std::ofstream fTorque ("torque.txt");//TODO remove at end

long reset_imr=0;

//---------Constructor------------------------------------------------------------------------------------------
/*SlipAngle::SlipAngle()
{
	//  Initialisation of slip frequency and angle
	OmegaSlip[0] = 0;
	ThetaSlip[0] = 0;
}*/



//#ifdef FLOATP//TODO JAN

RotFluxAng::RotFluxAng(/*float imr_*/)//TODO necess rfa?? para invocar construc? NEW
{
	//  Initialisation of slip frequency and angle
	OmegaSlip = 0.0;
	rot_f_ang= 0.0000000000001;
	theta_r=0.0;
	theta_slip=0.0;
	imr=0.0;//TODO(remove)nao sera necessario se nao voltage compensator
	wm=FLT_FROMFP(1);
	
	
	
	idm=0.00000001;
	iqm=0.0000001;
	F=0.00001;

};
void RotFluxAng::reset(){
	OmegaSlip = 0.0;
	rot_f_ang= 0.0000000000001;
	theta_r=0.0;
	theta_slip=0.0;
	//imr=0.0;//TODO(remove)nao sera necessario se nao voltage compensator

};

float RotFluxAng::/*CalcSlipAngle*/RotFluxAng_(float iqs, float ids, float sPWM, float tRotor,float Omega_r)//added Omega_r
{
	// Calculate current slip velocity
	// imr=imr+sPWM/tRotor*(ids-imr);
	// OmegaSlip=(iqs/(tRotor*imr));
		//std::cout<<_imr<<" imr "<<_OmegaSlip<<" omegaslip "<<std::endl;

	if (wm==0.0)wm=0.00000000001;
	float s_2=(wm-Omega_r)/wm*(wm-Omega_r)/wm;
//			std::cout<<wm<<" wm "<<std::endl;

//		std::cout<<s_2<<" r_s_2 "<<std::endl;
		float Rfe=Rm/(s_2+1);
//		std::cout<<Rfe<<" Rfe "<<std::endl;
		float Tfe = (M/Rfe);
//std::cout<<Tfe<<" Tfe "<<std::endl;
		float T_ro_r = Llr/Rr_est;
//	std::cout<<T_ro_r<<" T_ro_r "<<std::endl;
		
	float F0=F, idm0=idm, iqm0=iqm;
//	std::cout<<F<<" F "<<idm<<" idm "<<iqm<<" iqm "<<std::endl;
	
	
	//float k1_1= M/T_ro_r*idm-F/T_ro_r;
	//std::cout<<k1_1<<" k11 "<<std::endl;
	//float k1_2= ids/Tfe+F/Llr/Tfe+wm*iqm-Lr/Llr/Tfe*idm;
	//std::cout<<k1_2<<" k12 "<<std::endl;
	//float k1_3= iqs/Tfe-wm*idm-Lr/Llr/Tfe*iqm;
	//std::cout<<k1_3<<" k13 "<<std::endl;
	
	
	//Runge-kutta:
	//F= F0 +sPWM/2.0*k1_1;
	//idm = idm0 + sPWM/2.0 * k1_2;
	//iqm = iqm0 + sPWM/2.0 * k1_3;
	//std::cout<<F<<" F "<<idm<<" idm "<<iqm<<" iqm "<<std::endl;
	
	
	//float k2_1= M/T_ro_r*(idm/*idm+k1_2*sPWM/2*/)-1/T_ro_r*(F/*F+k1_1*sPWM/2*/);
	//std::cout<<k2_1<<" k21 "<<std::endl;
	//float k2_2= 1/Tfe*(ids/*+sPWM/2*/)+1/Llr/Tfe*(F/*F+k1_1*sPWM/2*/)+(/*??*/wm/*+sPWM/2*/)*(iqm/*iqm+k1_3*sPWM/2*/)-Lr/Llr/Tfe*(idm/*idm+k1_2*sPWM/2*/);
	//std::cout<<k2_2<<" k22 "<<std::endl;
	//float k2_3= 1/Tfe*(iqs/*+sPWM/2*/)-(/*??*/wm/*+sPWM/2*/)*(idm/*idm+k1_2*sPWM/2*/)-Lr/Llr/Tfe*(iqm/*iqm+k1_3*sPWM/2*/);
	//std::cout<<k2_3<<" k23 "<<std::endl;
//	F= F0 +sPWM/2.0*k2_1;
	//idm = idm0 + sPWM/2.0 * k2_2;
	//iqm = iqm0 + sPWM/2.0 * k2_3;
	//std::cout<<F<<" F "<<idm<<" idm "<<iqm<<" iqm "<<std::endl;
	
	//float k3_1= M/T_ro_r*(idm/*+k2_2*sPWM/2*/)-1/T_ro_r*(F/*+k2_1*sPWM/2*/);
	//std::cout<<k3_1<<" k31 "<<std::endl;
//	float k3_2= 1/Tfe*(ids/*+sPWM/2*/)+1/Llr/Tfe*(F/*+k2_1*sPWM/2*/)+(/*??*/wm/*+sPWM/2*/)*(iqm/*+k2_3*sPWM/2*/)-Lr/Llr/Tfe*(idm/*+k2_2*sPWM/2*/);
	//std::cout<<k3_2<<" k32 "<<std::endl;
	//float k3_3= 1/Tfe*(iqs/*+sPWM/2*/)-(/*??*/wm/*+sPWM/2*/)*(idm/*+k2_2*sPWM/2*/)-Lr/Llr/Tfe*(iqm/*+k2_3*sPWM/2*/);
	//std::cout<<k3_3<<" k33 "<<std::endl;
	//F= 	  F0 +   sPWM * k3_1;
	//idm = idm0 + sPWM * k3_2;
	//iqm = iqm0 + sPWM * k3_3;
	//std::cout<<F<<" F "<<idm<<" idm "<<iqm<<" iqm "<<std::endl;
	//float k4_1= M/T_ro_r*(idm/*+k3_2*sPWM*/)-1/T_ro_r*(F/*+k3_1*sPWM*/);
	//std::cout<<k4_1<<" k41 "<<std::endl;
	//float k4_2= 1/Tfe*(ids/*+sPWM*/)+1/Llr/Tfe*(F/*+k3_1*sPWM*/)+(/*??*/wm/*+sPWM*/)*(iqm/*+k3_3*sPWM*/)-Lr/Llr/Tfe*(idm/*+k3_2*sPWM*/);
	//std::cout<<k4_2<<" k42 "<<std::endl;
	//float k4_3= 1/Tfe*(iqs/*+sPWM*/)-(/*??*/wm/*+sPWM*/)*(idm/*+k3_2*sPWM*/)-Lr/Llr/Tfe*(iqm/*+k3_3*sPWM*/);
	//std::cout<<k4_3<<" k43 "<<std::endl;
	
	
	//F=	F0  +sPWM/6.0*(k1_1+2.0*k2_1+2.0*k3_1+k4_1);
	//idm=idm0+sPWM/6.0*(k1_2+2.0*k2_2+2.0*k3_2+k4_2);
	//iqm=iqm0+sPWM/6.0*(k1_3+2.0*k2_3+2.0*k3_3+k4_3);
	//std::cout<<F<<" F "<<idm<<" idm "<<iqm<<" iqm "<<std::endl;
	
	//end Runge-kutta
	
	//exponential method with losses:
	float a = -1/T_ro_r; float b = M/T_ro_r;
	float ex = exp(a*sPWM);
	
	F= b *1/a*(ex-1)*idm0+ex*F0;
//	std::cout<<F<<" F_exp "<<std::endl;
	 a = -Lr/Llr/Tfe;  b = 1;
	 ex = exp(a*sPWM);
	
	idm=  b *1/a*(ex-1)*(ids/Tfe+F0/Llr/Tfe+wm*iqm0)+ex*idm0;
	
//	std::cout<<idm<<" idm exp "<<std::endl;
	a = -Lr/Llr/Tfe; b = 1;
	 ex = exp(a*sPWM);
	
	iqm= b *1/a*(ex-1)*(iqs/Tfe-wm*idm0)+ex*iqm0;
	 
//	std::cout<<iqm<<" iqm ex "<<std::endl;
	
	
	OmegaSlip=(M*iqm/(T_ro_r*F));
	
	imr=F/M;
	
	
	
	//with first order exponential integrator:
	//float
	 a = -1.0/tRotor;
	  //float 
	  b = 1/tRotor;
	//float
	 ex = exp(a*sPWM);
	//imr=ex*imr+b *1/a*(ex-1)*(ids);
	
	//with simple euler, forward?
	//imr=imr+sPWM/tRotor*(ids-imr);

	//OmegaSlip=(iqs/(tRotor*/*(M*ids)*/imr));
	
	
	
	
	//TODO test with error
			//float rand_= rand()%10;rand_=rand_/300*2+1.0-0.1/3;//10/3 % error average
			//std::cout<<rand_<<std::endl;
			//imr*=rand_;		
	//if (reset_imr<(2*1/T))
	//			{reset_imr++;
		//			if(reset_imr>=1*1/T){
			//			reset_imr=0; 
				//		imr=10;
					//	fTorque<<"imr_RESET "<<imr<<std::endl;}}
	
	wm=Omega_r+OmegaSlip;//we, electric sincronous speed 

	theta_slip+=OmegaSlip*sPWM;//while(theta_slip>=PI){theta_slip=theta_slip-PI;};
	while (theta_slip >= (2*PI)||theta_slip<0)//TODO 0 e 2*PI || 0-PI 
	{       if( theta_slip >=( 2 * PI ))
				theta_slip = theta_slip - ( 2 * PI );
			else	theta_slip = theta_slip + ( 2 * PI );
	}
	theta_r += Omega_r * sPWM;
	while (theta_r >= ( 2 * PI )|| theta_r < 0 )//TODO 0 e 2*PI || 0-PI 
	{       if( theta_r >=( 2 * PI ))
				theta_r = theta_r - ( 2 * PI );
			else	theta_r = theta_r + ( 2 * PI );
	}
	
	rot_f_ang = theta_r + theta_slip;//Omega_r*sPWM ou melhor,  Teta_r 
	// Restrict rotor flux angle to a value between 0-2Pi radians
	while ( rot_f_ang >= ( 2 * PI ) || rot_f_ang < 0 )//TODO 0 e 2*PI || 0-PI 
	{       if( rot_f_ang >=( 2 * PI ))
				rot_f_ang = rot_f_ang - ( 2 * PI );
			else	rot_f_ang = rot_f_ang + ( 2 * PI );
	}

return rot_f_ang;
}
float RotFluxAng::get_wm(){return wm ;};
float RotFluxAng::get_imr(){return imr ;};//TODO(remove)nao sera necessario se nao voltage compensator 
float RotFluxAng::get_rfa(){return rot_f_ang ;};

//TODO JAN #else
FP_RotFluxAng::FP_RotFluxAng(/*float imr_*/)//TODO necess rfa?? para invocar construc? NEW
{
	
	//  Initialisation of slip frequency and angle
	FP_OmegaSlip = 0;
	FP_rot_f_ang=1;// FP_FROMFLT(0.0000000000001);
	FP_theta_r=0;
	FP_theta_slip=0;
	FP_imr=1;//TODO(remove)nao sera necessario se nao voltage compensator
	FP_wm=1;//FP_FROMFLT(0.0000000000000001);
};
void FP_RotFluxAng::FP_reset(){
	FP_OmegaSlip = 0;
	FP_rot_f_ang= 1;// FP_FROMFLT(0.0000000000001);
	FP_theta_r= 0;
	FP_theta_slip= 0;
	FP_imr=1;//TODO(remove)nao sera necessario se nao voltage compensator
	FP_wm=1;//FP_FROMFLT(0.0000000000000001);
};

s32fp FP_RotFluxAng::/*CalcSlipAngle*/FP_RotFluxAng_(s32fp FP_iqs, s32fp FP_ids, s32fp FP_sPWM, s32fp FP_tRotor,s32fp FP_Omega_r)//added Omega_r
{
	//fTorque<<"W_slip RFA theta_r Theta_slip imr wm "<<FLT_FROMFP(FP_OmegaSlip)<<" "<<FLT_FROMFP(FP_rot_f_ang)<<" "<<FLT_FROMFP(FP_theta_r)<<" "<<FLT_FROMFP(FP_theta_slip)<<" "<<FLT_FROMFP(FP_imr)<<" "<<FLT_FROMFP(FP_wm)<<std::endl;
	// Calculate current slip velocity
//		imr=imr+sPWM/tRotor*(ids-imr);
	FP_imr = FP_imr + ( FP_MUL( FP_DIV( FP_sPWM , FP_tRotor ),( FP_ids - FP_imr )));
		//OmegaSlip=(iqs/(tRotor*imr));
fTorque<<"FLT_FROMFP(iqs) "<<FLT_FROMFP(FP_iqs)<<"FLT_FROMFP(tRotor) " <<FLT_FROMFP(FP_tRotor)<<"FLT_FROMFP(imr) "<<FLT_FROMFP(FP_imr)<<std::endl;
	if ( FLT_FROMFP(FP_imr)< 0.001 ) FP_imr = FP_FROMFLT( 0.001 ); 
	fTorque<<"FLT_FROMFP(imr) "<<FLT_FROMFP(FP_imr)<<std::endl;

	FP_OmegaSlip = FP_DIV( FP_iqs , FP_MUL( FP_tRotor , FP_imr ));
	
	FP_wm = FP_Omega_r + FP_OmegaSlip ;//we, electric sincronous speed 

	FP_theta_slip += FP_MUL( FP_OmegaSlip , FP_sPWM );//while(theta_slip>=PI){theta_slip=theta_slip-PI;};
	while ( FP_theta_slip >= FP_2PI || FP_theta_slip < FP_FROMFLT( 0.0 ))//TODO 0 e 2*PI || 0-PI 
	{       if( FP_theta_slip >= FP_2PI )
				FP_theta_slip = FP_theta_slip - FP_2PI;
			else	FP_theta_slip = FP_theta_slip + FP_2PI;
	}
	FP_theta_r += FP_MUL( FP_Omega_r , FP_sPWM );
	while ( FP_theta_r >= FP_2PI || FP_theta_r < FP_FROMFLT( 0.0 ))//TODO 0 e 2*PI || 0-PI 
	{       if( FP_theta_r >= FP_2PI )
				FP_theta_r = FP_theta_r - FP_2PI;
			else	FP_theta_r = FP_theta_r + FP_2PI ;
	}
	
	FP_rot_f_ang = FP_theta_r + FP_theta_slip ;//Omega_r*sPWM ou melhor,  Teta_r 
	// Restrict rotor flux angle to a value between 0-2Pi radians
	while ( FP_rot_f_ang >= FP_2PI || FP_rot_f_ang < FP_FROMFLT( 0.0 ))//TODO 0 e 2*PI || 0-PI 
	{       if( FP_rot_f_ang >= FP_2PI )
				FP_rot_f_ang = FP_rot_f_ang - FP_2PI ;
			else	FP_rot_f_ang = FP_rot_f_ang + FP_2PI ;
	}
	//fTorque<<"W_slip RFA theta_r Theta_slip imr wm "<<FLT_FROMFP(FP_OmegaSlip)<<" "<<FLT_FROMFP(FP_rot_f_ang)<<" "<<FLT_FROMFP(FP_theta_r)<<" "<<FLT_FROMFP(FP_theta_slip)<<" "<<FLT_FROMFP(FP_imr)<<" "<<FLT_FROMFP(FP_wm)<<std::endl;

return FP_rot_f_ang;
}

s32fp FP_RotFluxAng::FP_get_wm(){return FP_wm;};
s32fp FP_RotFluxAng::FP_get_imr(){return FP_imr;};//TODO put FP//TODO(remove)nao sera necessario se nao voltage compensator 
s32fp FP_RotFluxAng::FP_get_rfa(){return FP_rot_f_ang;};

//TODO JAN #endif


/*float SlipAngle::CalcSlipAngle(float iqs, float ids, float sPWM, float tRotor)
{
	// Calculate current slip velocity
	OmegaSlip[1] = ((1 + tRotor*sPWM)/tRotor) + (iqs/ids);

	// Calculate accumulated slip angle using average value
	ThetaSlip[1] = (ThetaSlip[0] + ((OmegaSlip[0] +OmegaSlip[1])/2)*sPWM);

	// Restrict slip angle to a value between 0-Pi radians
	while (ThetaSlip[1] >= PI)
	{
		ThetaSlip[1] = ThetaSlip[1] - PI;
	}

	OmegaSlip[0] = OmegaSlip[1];
	ThetaSlip[0] = ThetaSlip[1];

	return ThetaSlip[1];
}*/
