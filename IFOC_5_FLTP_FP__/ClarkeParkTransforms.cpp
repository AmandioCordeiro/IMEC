//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Graeme Bell <graemeb@users.sourceforge.net>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   This file provides standalone functions for implementing Clarke and Park
//   transforms. A highlevel implementation suitable for initial use & testing,
//   but may require subsequent optimisiation for the final hardware.
//
// HISTORY:
//   Graeme Bell 1/May/2001 - First Cut
//------------------------------------------------------------------------------

#ifndef MATHS_CLARKEPARKTRANSFORMS_H
#define MATHS_CLARKEPARKTRANSFORMS_H

//--- Includes -----------------------------------------------------------------
#include <cmath>
#include <math.h>
#include "my_fp.h"
#include "sine_core.h"
//#include "variables.hpp"
//#include "Rot_Flux_Ang.hpp"

//--- Defines ------------------------------------------------------------------
#define CONST_SQRT3_  1.732050807568877293527446315059
#define CONST_1_SQRT3 tReal(1/CONST_SQRT3_)
#define CONST_2_SQRT3 tReal(2/CONST_SQRT3_)
#define CONST_SQRT3_2 tReal(CONST_SQRT3_/2)

#define FP_CONST_SQRT3_  FP_FROMFLT(1.732050807568877293527446315059)
#define FP_CONST_1_SQRT3 s32fp(FP_DIV(FP_FROMFLT(1),FP_CONST_SQRT3_))
#define FP_CONST_2_SQRT3 s32fp(FP_DIV(FP_FROMFLT(2),FP_CONST_SQRT3_))
#define FP_CONST_SQRT3_2 s32fp(FP_DIV(FP_CONST_SQRT3_,FP_FROMFLT(2)))


#define PI	3.14159265358979

    
namespace Maths{

    //-------------------------------------------------------------------------
    // 
    
    // typedef fixed< int32_t, 16 > tReal
    typedef float tReal;
	//typedef s32fp FP_tReal;
	typedef int32_t FP_tReal;
	
    /** 
     * Simple structure for representing values in a three-phase system.
     */
    struct tThreePhase {
        tThreePhase( tReal ra, tReal rb, tReal rc ) : a(ra), b(rb), c(rc) {}
        
        tReal  a, b, c;
    };
    struct FP_tThreePhase {
        FP_tThreePhase( s32fp ra_, s32fp rb_, s32fp rc_ ) : a(ra_), b(rb_), c(rc_) {}
        
        s32fp  a, b, c;
    };
    /**
     * Simple structure for representing values in a two-phase orthogonal
     * system.
     */
    struct tTwoPhase { 
        tTwoPhase( tReal ralpha, tReal rbeta ) : alpha(ralpha), beta(rbeta) {}
        
        tReal  alpha, beta;
    };
    struct FP_tTwoPhase { 
        FP_tTwoPhase( s32fp ralpha_, s32fp rbeta_ ) : alpha(ralpha_), beta(rbeta_) {}
        
        s32fp  alpha, beta;
    };
    /**
     * Simple structure for representing values in a two-phase system with a 
     * rotating frame of reference.
     */
    struct tTwoPhaseDQ {
        tTwoPhaseDQ( tReal rd, tReal rq ) : d(rd), q(rq) {}
        
        tReal  d, q;
    };
    struct FP_tTwoPhaseDQ {
        FP_tTwoPhaseDQ( s32fp rd_, s32fp rq_ ) : d(rd_), q(rq_) {}
        
        s32fp  d, q;
    };
    /**
     * Simple structure for holding the sin & cos of an angle. This is to allow
     * the values to be calculated just once for the current rotor angle and
     * then be used in multiple transforms.
     */
    struct tCosSin {
        tCosSin( tReal radians ) : cos( std::cos(radians) ), sin( std::sin(radians) ) {}
        
        tReal  cos;
        tReal  sin;
    };
	struct FP_tCosSin {
        FP_tCosSin( s32fp radians_ ) : cos_( FP_FROMFLT(((float) (SineCore::Cosine((uint16_t)((FLT_FROMFP(radians_)/*>>CST_DIGITS*/)*(1<<16)/2/PI))))/(1<<15))/*std::cos(radians)*/ ), sin_( FP_FROMFLT(((float) (SineCore::Sine((uint16_t)((FLT_FROMFP(radians_)/*>>CST_DIGITS*/)*(1<<16)/2/PI))))/(1<<15)) /*std::sin(radians)*/ ) {}
        
        s32fp  cos_;
        s32fp  sin_;
    };
    //-------------------------------------------------------------------------
    // Park Transformations
    
    inline tTwoPhaseDQ Park( const tCosSin& angle, const tTwoPhase& in ) {
       //fTorque<<" in.alpha"<<in.alpha<<" in.beta "<<in.beta<<" angle.cos "<<angle.cos<<" angle.sin "<<angle.sin<<"Park "<<(tTwoPhaseDQ( 
             //in.alpha*angle.cos + in.beta*angle.sin, 
            //-in.alpha*angle.sin + in.beta*angle.cos ).q)<<std::endl;
        return tTwoPhaseDQ( 
             in.alpha*angle.cos + in.beta*angle.sin, 
            -in.alpha*angle.sin + in.beta*angle.cos );
    }
    
    inline FP_tTwoPhaseDQ FP_Park( const FP_tCosSin& angle_, const FP_tTwoPhase& in_ ) {
       // fTorque<<" in_.alpha"<<FLT_FROMFP(in_.alpha)<<" in_.beta_ "<<FLT_FROMFP(in_.beta)<<" angle_.cos_ "<<FLT_FROMFP(angle_.cos_)<<" angle_.sin_ "<<FLT_FROMFP(angle_.sin_)<<"FP_Park "<<(FLT_FROMFP(FP_tTwoPhaseDQ( 
         //   ( FP_MUL(in_.alpha,angle_.cos_) + FP_MUL(in_.beta,angle_.sin_)), 
           // (-FP_MUL((in_.alpha),angle_.sin_) + FP_MUL(in_.beta,angle_.cos_) )).q))<<std::endl;
        return FP_tTwoPhaseDQ( 
             (FP_MUL(in_.alpha,angle_.cos_) + FP_MUL(in_.beta,angle_.sin_)), 
            (-FP_MUL((in_.alpha),angle_.sin_) + FP_MUL(in_.beta,angle_.cos_) ));
    }
    inline tTwoPhaseDQ Park( tReal angle, const tTwoPhase& in ) {
       // fTorque<<" angle "<<angle<<std::endl;
        return Park( tCosSin(angle), in );
    }
    inline FP_tTwoPhaseDQ FP_Park( s32fp angle_, const FP_tTwoPhase& in_ ) {
      //  fTorque<<" angle_ "<<FLT_FROMFP(angle_)<<std::endl;
        return FP_Park( FP_tCosSin(angle_), in_ );
    }
    
    inline tTwoPhase InvPark( const tCosSin& angle, const tTwoPhaseDQ& in ) {
        return tTwoPhase(
             in.d*angle.cos - in.q*angle.sin, 
             in.d*angle.sin + in.q*angle.cos );//changed - by + in cos
    }
    inline FP_tTwoPhase FP_InvPark( const FP_tCosSin& angle_, const FP_tTwoPhaseDQ& in_ ) {
        //fp_stream<<"cos"<<FLT_FROMFP(angle.cos)<<std::endl;
            
        return FP_tTwoPhase(
             FP_MUL(in_.d,angle_.cos_) - FP_MUL(in_.q,angle_.sin_), 
             FP_MUL(in_.d,angle_.sin_) + FP_MUL(in_.q,angle_.cos_) );//changed - by + in cos
    }
    inline tTwoPhase InvPark( tReal angle, const tTwoPhaseDQ& in ) {
        return InvPark( tCosSin(angle), in );
    }
    inline FP_tTwoPhase FP_InvPark( s32fp angle_, const FP_tTwoPhaseDQ& in_ ) {
        return FP_InvPark( FP_tCosSin(angle_), in_ );
    }

    //-------------------------------------------------------------------------
    // Clarke Transformations
    
    inline tTwoPhase Clarke( const tThreePhase& in ) {
        // note: this assumes in.a+in.b+in.c == 0
        
       // fTorque<<"Clarke "<<tTwoPhase( in.a, in.a*CONST_1_SQRT3 + in.b*CONST_2_SQRT3 ).beta;
   
        
        return tTwoPhase( in.a, in.a*CONST_1_SQRT3 + in.b*CONST_2_SQRT3 );
    }
    inline FP_tTwoPhase FP_Clarke( const FP_tThreePhase& in_ ) {
        // note: this assumes in.a+in.b+in.c == 0
        //fTorque<<"FP_Clarke "<<FLT_FROMFP(FP_tTwoPhase( in_.a, (FP_MUL(in_.a,FP_CONST_1_SQRT3) + FP_MUL(in_.b,FP_CONST_2_SQRT3)) ).beta);
        return FP_tTwoPhase( in_.a, (FP_MUL(in_.a,FP_CONST_1_SQRT3) + FP_MUL(in_.b,FP_CONST_2_SQRT3)) );
    }
    inline tThreePhase InvClarke( const tTwoPhase& in ) {
        tReal r1 = -in.alpha/2;
        tReal r2 = in.beta*CONST_SQRT3_2;
        return tThreePhase( in.alpha, r1 + r2, r1 - r2 );
    }
	inline FP_tThreePhase FP_InvClarke( const FP_tTwoPhase& in_ ) {
        s32fp r1_ = -FP_DIV(in_.alpha,FP_FROMFLT(2));
        s32fp r2_ = FP_MUL(in_.beta,FP_CONST_SQRT3_2);
        return FP_tThreePhase( in_.alpha, r1_ + r2_, r1_ - r2_ );
    }
    //-------------------------------------------------------------------------
    // Combined Clarke & Park Transformations
    
    inline tTwoPhaseDQ ClarkePark( tReal angle, const tThreePhase& in ) {
        return Park( angle, Clarke( in ) );
    }
    inline FP_tTwoPhaseDQ FP_ClarkePark( s32fp angle_, const FP_tThreePhase& in_ ) {
        return FP_Park( angle_, FP_Clarke( in_ ) );
    }
    inline tThreePhase InvClarkePark( tReal angle, const tTwoPhaseDQ& in ) {
        return InvClarke( InvPark( angle, in ) );
    }
	inline FP_tThreePhase FP_InvClarkePark( s32fp angle_, const FP_tTwoPhaseDQ& in_ ) {
        return FP_InvClarke( FP_InvPark( angle_, in_ ) );
    }
//-----------------------------------------------------------------------------
    
}  // namespace Maths

#endif  // inclusion guard
