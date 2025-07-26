float Lls= 0.000140;
float Ls = ( Lls + M ) ;
s32fp FP_Ls = FP_FROMFLT ( Ls );
s32fp FP_Idn = FP_FROMFLT( Idn );
s32fp FP_Rs = FP_FROMFLT( Rs );
float ro = ( 1.0 - ( M * M ) / ( Ls * Lr )) ;
float KT = ( 3.0 / 2.0 * np * M * M / Lr ) ; 
s32fp FP_KT = FP_FROMFLT( KT ) ; 
float TM1 = ( KT * Idn * sqrt ( Imax * Imax - Idn * Idn ) ) ;// Maximum torque to apply in first zone
s32fp FP_TM1 = FP_FROMFLT( TM1 ); 
s32fp FP_ro = FP_FROMFLT ( ro ) ;
float Ls_ro = ( Ls * ro ) ;
s32fp FP_INV__ro_ro = FP_FROMFLT ( 1 / ( ro * ro )) ;
s32fp FP_ro_ro = FP_FROMFLT ( ro * ro ) ;
s32fp FP_KT__2__ro = FP_FROMFLT ( KT / 2.0 / ro) ;
float ro_ro_Ls_Ls_Ls_Ls__KT__KT = ( ro * ro * Ls * Ls * Ls * Ls / KT / KT ) ;
s32fp FP_Imax = FP_FROMFLT ( Imax ) ;
s32fp FP_Imax_Imax = FP_FROMFLT( Imax * Imax ) ; //TODO remove
s32fp FP_Imax_Imax_ro_ro = FP_FROMFLT( Imax * Imax * ro * ro ) ; // TODO !! was without FP_
float Imax_Imax = ( Imax * Imax ) ; 
float Imax_Imax_Imax_Imax = ( Imax * Imax * Imax * Imax ) ; 
float Imax_Imax__2 = ( Imax_Imax / 2.0 ) ; 

float four__KT_KT = ( 4.0 / KT / KT ) ; 
s32fp FP_CONST_Tm2 = FP_FROMFLT( KT / ( 1.0 - ro * ro ) ) ; 
float T_LOAD_1_sec = 0.135 / T ;

float kwn = ( Ls*sqrt(Idn*Idn+ro*ro*(Imax*Imax -Idn*Idn)));
float kwc = (Imax*Ls) * sqrt( ( ro*ro+1.0 ) / ( 2.0*ro*ro ) );

float Imax_Imax_ro_ro = Imax*Imax*ro*ro;
float ro_ro = ro * ro ;
float KT__KT = KT / KT ;
float ro_ro_Ls_Ls_Ls_Ls = ro * ro * Ls * Ls * Ls * Ls ;
