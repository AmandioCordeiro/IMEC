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

#include "my_fp.h"
#include <fstream>//TODO remove at end

extern std::ofstream fTorque ;//TODO remove at end

#define PI	3.14159265358979
#define FP_PI	FP_FROMFLT(3.14159265358979)
#define FP_2PI (FP_PI<<1)//TODO substituir por valor ????
class FP_RotFluxAng
{
   public:
    FP_RotFluxAng(/*float imr_/*rfa*/);
	float RotFluxAng_/*CalcSlipAngle*/(float iqs, float ids, float sPWM, float tRotor, float Omega_r);
	s32fp FP_RotFluxAng_/*CalcSlipAngle*/(s32fp FP_iqs, s32fp FP_ids, s32fp FP_sPWM, s32fp FP_tRotor, s32fp FP_Omega_r);
	float	get_imr();
	s32fp FP_get_imr();
	float get_rfa();
	s32fp FP_get_rfa();
	void reset();
	void FP_reset();
	float get_wm();
	s32fp FP_get_wm();



   private:
	float OmegaSlip;
	float wm;
	float rot_f_ang;//ThetaSlip;
	float theta_slip;
	float theta_r;
	float imr;//float OmegaSlip[2], ThetaSlip[2];
	s32fp FP_OmegaSlip;
	s32fp FP_wm;
	s32fp FP_rot_f_ang;//ThetaSlip;
	s32fp FP_theta_slip;
	s32fp FP_theta_r;
	s32fp FP_imr;//float OmegaSlip[2], ThetaSlip[2];
};

//class RotFluxAng
//{
  // public:
 //   RotFluxAng(/*float imr_/*rfa*/);
//	s32fp RotFluxAng_/*CalcSlipAngle*/(s32fp iqs, s32fp ids, s32fp sPWM, s32fp tRotor, s32fp Omega_r);
//	s32fp	get_imr();
//	s32fp get_rfa();
//	void reset();
//s32fp get_wm();



  // private:
//	s32fp OmegaSlip;
//	s32fp wm;
//	s32fp rot_f_ang;//ThetaSlip;
//	s32fp theta_slip;
//	s32fp theta_r;
//	s32fp imr;//float OmegaSlip[2], ThetaSlip[2];
//};


#endif /* SLIPANGLE_HPP_ */
