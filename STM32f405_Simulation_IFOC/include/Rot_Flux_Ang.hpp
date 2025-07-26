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
//
// HISTORY:
//   Graham Osborne 21/9/2011 - First Cut in C (needs refinement!)
//   Bernard Mentink 28/3/2012 - ported to C++, included in Tumanko_QP project.
//
//------------------------------------------------------------------------------

#ifndef SLIPANGLE_HPP_
#define SLIPANGLE_HPP_


#include "my_fp_2.h"
//#include <fstream>//TODO remove at end
//#include "foc.hpp"
#include "variables.hpp"

//extern std::ofstream fTorque ;//TODO remove at end

#define FP_PI	FP_FROMFLT(3.14159265358979)
//#define FP_2PI (FP_PI<<1)//TODO substituir por valor ????

#define M 0.001500//? mutual inductance
//#define M_M (M*M)
//#define M_M__Rm (M*M/Rm)
//#define M_M_Llr_Llr__Rm__Lr__Lr (M*M*Llr*Llr/Rm/Lr/Lr)
//#define FP_M_M FP_FROMFLT(M*M)

#define PI	3.14159265358979


//TODO JAN#ifdef FLOATP
class RotFluxAng
{
   public:
    RotFluxAng(/*float imr_*//*rfa*/);
	float RotFluxAng_/*CalcSlipAngle*/(float iqs, float ids, float sPWM, float tRotor, float Omega_r);
	float	get_imr();
	float get_rfa();
	void reset();
	float get_wm();
	float get_theta_r();
    float get_OmegaSlip();
    float get_theta_slip();



   private:
	float OmegaSlip;
	float wm;
	float rot_f_ang;//ThetaSlip;
	float theta_slip;
	float theta_r;
	float imr;//float OmegaSlip[2], ThetaSlip[2];

	float idm, iqm, F;

};
//TODO JAN#else
class FP_RotFluxAng
{
   public:
    FP_RotFluxAng(/*float imr_*//*rfa*/);
	s32fp FP_RotFluxAng_/*CalcSlipAngle*/(s32fp FP_iqs, s32fp FP_ids, s32fp FP_sPWM, s32fp FP_tRotor, s32fp FP_Omega_r);
	s32fp /*CalcSlipAngle*/FP_RotFluxAng__(s32fp FP_iqs, s32fp FP_ids, s32fp FP_sPWM, s32fp FP_tRotor,s32fp FP_Omega_r);//added Omega_r

	s32fp FP_get_imr();
	s32fp FP_get_rfa();

	void FP_reset();
	s32fp FP_get_wm();



   private:
	//double idm, iqm, F;

	s32fp FP_OmegaSlip;
	s32fp FP_wm;
	s32fp FP_rot_f_ang;//ThetaSlip;
	s32fp FP_theta_slip;
	s32fp FP_theta_r;
	s32fp FP_imr;//float OmegaSlip[2], ThetaSlip[2];
	s32fp FP_idm, FP_iqm, FP_F;

};

#endif /* SLIPANGLE_HPP_ */
