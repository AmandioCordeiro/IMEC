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


//---------Constructor------------------------------------------------------------------------------------------
/*SlipAngle::SlipAngle()
{
	//  Initialisation of slip frequency and angle
	OmegaSlip[0] = 0;
	ThetaSlip[0] = 0;
}*/
RotFluxAng::RotFluxAng(/*float imr_*/)//TODO necess rfa?? para invocar construc? NEW
{
	//  Initialisation of slip frequency and angle
	OmegaSlip = 0.0;
	rot_f_ang= 0.0000000000001;
	theta_r=0.0;
	theta_slip=0.0;
	imr=0.0;//TODO(remove)nao sera necessario se nao voltage compensator
	wm=0.0000000000000001;
};
void RotFluxAng::reset(){
	OmegaSlip = 0.0;
	rot_f_ang= 0.0000000000001;
	theta_r=0.0;
	theta_slip=0.0;
	//imr=0.0;//TODO(remove)nao sera necessario se nao voltage compensator
	wm=0.0000000000000001;
};
double RotFluxAng::/*CalcSlipAngle*/RotFluxAng_(double iqs, double ids, double sPWM, double tRotor,double Omega_r)//added Omega_r
{
	// Calculate current slip velocity
	imr=imr+sPWM/tRotor*(ids-imr);
	OmegaSlip=(iqs/(tRotor*imr));
	
	wm=Omega_r+OmegaSlip;//we, electric sincronous speed 

	theta_slip+=OmegaSlip*sPWM;//while(theta_slip>=PI){theta_slip=theta_slip-PI;};
	while (theta_slip >= (2*PI)||theta_slip<0)//TODO 0 e 2*PI || 0-PI 
	{       if(theta_slip>=(2*PI))
				theta_slip = theta_slip - (2*PI);
			else	theta_slip=theta_slip+(2*PI);
	}
	theta_r+=Omega_r*sPWM;
	while (theta_r >= (2*PI)||theta_r<0)//TODO 0 e 2*PI || 0-PI 
	{       if(theta_r>=(2*PI))
				theta_r = theta_r - (2*PI);
			else	theta_r=theta_r+(2*PI);
	}
	
	rot_f_ang=theta_r+theta_slip;//Omega_r*sPWM ou melhor,  Teta_r 
	// Restrict rotor flux angle to a value between 0-2Pi radians
	while (rot_f_ang >= (2*PI)||rot_f_ang<0)//TODO 0 e 2*PI || 0-PI 
	{       if(rot_f_ang>=(2*PI))
				rot_f_ang = rot_f_ang - (2*PI);
			else	rot_f_ang=rot_f_ang+(2*PI);
	}

return rot_f_ang;
}
double RotFluxAng::get_wm(){return wm;};
double RotFluxAng::get_imr(){return imr;};//TODO(remove)nao sera necessario se nao voltage compensator 
double RotFluxAng::get_rfa(){return rot_f_ang;};

/*double SlipAngle::CalcSlipAngle(double iqs, double ids, double sPWM, double tRotor)
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
