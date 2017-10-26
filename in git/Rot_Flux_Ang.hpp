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


#define PI	3.14159265358979

class SlipAngle
{
   public:
      SlipAngle(float rfa);
	double RotFluxAng/*CalcSlipAngle*/(double iqs, double ids, double sPWM, double tRotor, double Omega_r);
	double	get_imr();
	double get_rfa();
double get_wm();



   private:
	double OmegaSlip;
	double wm;
	double rot_f_ang;//ThetaSlip;
	double theta_slip;
	double theta_r;
	double imr;//double OmegaSlip[2], ThetaSlip[2];
};


#endif /* SLIPANGLE_HPP_ */
