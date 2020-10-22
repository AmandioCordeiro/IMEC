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

std::ofstream fTorque ("torque.txt");//TODO remove at end


//---------Constructor------------------------------------------------------------------------------------------
/*SlipAngle::SlipAngle()
{
	//  Initialisation of slip frequency and angle
	OmegaSlip[0] = 0;
	ThetaSlip[0] = 0;
}*/
FP_RotFluxAng::FP_RotFluxAng(/*float imr_*/)//TODO necess rfa?? para invocar construc? NEW
{
	//  Initialisation of slip frequency and angle
	OmegaSlip = 0.0;
	rot_f_ang= 0.0000000000001;
	theta_r=0.0;
	theta_slip=0.0;
	imr=0.0;//TODO(remove)nao sera necessario se nao voltage compensator
	wm=0.0000000000000001;
	//  Initialisation of slip frequency and angle
	FP_OmegaSlip = 0;
	FP_rot_f_ang=1;// FP_FROMFLT(0.0000000000001);
	FP_theta_r=0;
	FP_theta_slip=0;
	FP_imr=1;//TODO(remove)nao sera necessario se nao voltage compensator
	FP_wm=1;//FP_FROMFLT(0.0000000000000001);
};
void FP_RotFluxAng::reset(){
	OmegaSlip = 0.0;
	rot_f_ang= 0.0000000000001;
	theta_r=0.0;
	theta_slip=0.0;
	//imr=0.0;//TODO(remove)nao sera necessario se nao voltage compensator
	FP_wm=0.0000000000000001;
	FP_OmegaSlip = 0;
	FP_rot_f_ang= 1;// FP_FROMFLT(0.0000000000001);
	FP_theta_r= 0;
	FP_theta_slip= 0;
	FP_imr=1;//TODO(remove)nao sera necessario se nao voltage compensator
	FP_wm=1;//FP_FROMFLT(0.0000000000000001);
};
float FP_RotFluxAng::/*CalcSlipAngle*/RotFluxAng_(float iqs, float ids, float sPWM, float tRotor,float Omega_r)//added Omega_r
{
	// Calculate current slip velocity
	imr=imr+sPWM/tRotor*(ids-imr);
	OmegaSlip=(iqs/(tRotor*imr));
	
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

s32fp FP_RotFluxAng::/*CalcSlipAngle*/FP_RotFluxAng_(s32fp FP_iqs, s32fp FP_ids, s32fp FP_sPWM, s32fp FP_tRotor,s32fp FP_Omega_r)//added Omega_r
{
	//fTorque<<"W_slip RFA theta_r Theta_slip imr wm "<<FLT_FROMFP(FP_OmegaSlip)<<" "<<FLT_FROMFP(FP_rot_f_ang)<<" "<<FLT_FROMFP(FP_theta_r)<<" "<<FLT_FROMFP(FP_theta_slip)<<" "<<FLT_FROMFP(FP_imr)<<" "<<FLT_FROMFP(FP_wm)<<std::endl;
	// Calculate current slip velocity
//		imr=imr+sPWM/tRotor*(ids-imr);
	FP_imr = FP_imr + ( FP_MUL( FP_DIV( FP_sPWM , FP_tRotor ),( FP_ids - FP_imr )));
		//OmegaSlip=(iqs/(tRotor*imr));
//fTorque<<"FLT_FROMFP(iqs) "<<FLT_FROMFP(FP_iqs)<<"FLT_FROMFP(tRotor) " <<FLT_FROMFP(FP_tRotor)<<"FLT_FROMFP(imr) "<<FLT_FROMFP(FP_imr)<<std::endl;
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

float FP_RotFluxAng::get_wm(){return wm ;};
float FP_RotFluxAng::get_imr(){return imr ;};//TODO(remove)nao sera necessario se nao voltage compensator 
float FP_RotFluxAng::get_rfa(){return rot_f_ang ;};

s32fp FP_RotFluxAng::FP_get_wm(){return FP_wm ;};
s32fp FP_RotFluxAng::FP_get_imr(){return FP_imr ;};//TODO(remove)nao sera necessario se nao voltage compensator 
s32fp FP_RotFluxAng::FP_get_rfa(){return FP_rot_f_ang ;};

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
