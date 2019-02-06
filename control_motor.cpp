
#include "control_motor.hpp"

//double Tr_calc_gl=Trrr;
double m_Tr_calc_gl=Trrr;//TODO insert at the end Tr motor value

double M=0.001500;//0.050382;/*0.51233;*///0.0117//0.069312//mutual inductance
double Ls=(0.000140+M);//0.051195;/*(0.00845+M);*///0.014//0.07132

double Lr=(0.000140+M);//0.052571;/*(0.00515+M);*///0.014//0.07132
#define Lps Ls-M*M/Lr
double J=(0.071);//TODO:?? 1pv5135-4ws14: 0.071
double ro=(1.0-(M*M)/(Ls*Lr));//0.999863//1-M*M/(Ls*Lr)//alterar nome

double VDC=300.0;//need actualization in pratice in function get_VDC( -Ifoc
double Vmax=VDC/sqrt(3.0);
/*#define*/double Wn = (Vmax/(Ls*sqrt(Idn*Idn+ro*ro*(Imax*Imax-Idn*Idn))));
/*#define*/double Wc = 0.98*Vmax/(Imax*Ls)*sqrt((ro*ro+1.0)/(2.0*ro*ro));//TODO acrescentei 0.9??
/*#define*/double Kt=(3.0/2.0*np*M*M/Lr);//TODO atencao pois M aparece como nome de 2 variaveis,repetido

/*#define*/double Rm=650.0; //TODO, nao funcionou c define nao sei pk, talvez dpois d inserir .0 ja deia, como esta agora
/*#define*/double Llr=(Lr-M);//0.0021891 //TODO verificar Llr=Lr-M;
#define CLUTCH_TIME 0.5//0.13//0.5
int clutch_time=-1;
double roLs=(ro*Ls);//0.013998//ro*Ls

tTwoPhase v_(0.0,0.0);
double velocidade=0;//vel. clutch
double T = 0.000125;//0.0001-10khz//TODO: PWM periode 0.000125-8khz  
#define T_lag (0.0000000000000000000001)//0.000000000001*0.02 TODO:?ver tempo de computacao desde que lidos os valores até disparos dos igbts
double vel_p=20.0;////20.0;//6;//1000;//1;//1//9/ /1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO: speed controller gain 
double vel_i=0.0006;//0.00006;//TODO: speed controller integral gain 
double torque_control_p=/*to use current controller */128/*estava este val em torq_control:0.33,mas experimentei c 1.4;*//* e deu bons result.*//*3.6*0.124*/;//alterei*0.5 TODO: torque controller gain
double torque_control_i=0.000008/*alterei0.0009(0.001) 0.008*/;//TODO: torque controller integral gain
double current_control_x_p=0.54;//0.5 a experiencia tava 0.6
double current_control_x_i=0.00001;

double IDC2_3=0.0;
double IDC=0.0;//TODO: remove at end. to calc medium value of IDC 
double vaa,vbb,vcc;
//double VDQ_alpha_esc,VDQ_beta_esc;

double iaa_p,vaa_p,cos_phi,two_phi=0.0,desc_p=0.0;
ofstream sfData_va_ia ("Data_va_ia.dat");
double Tm1;

tTwoPhaseDQ VDQ_ant(0.0,0.0);//torque and magnetization current value at exit controllers
tTwoPhaseDQ VDQ(0.0,0.0);
double const_VDQ_d=0.0;
double const_VDQ_q=0.0;
tTwoPhaseDQ VDQ_ant_1(0.0,0.0);
tThreePhase v(0.0,0.0,0.0);
double T0,T1,T2;//2-errado pois T1+T2 estava a dar > k T. 1-T0 indica o tempo por periodo k esta desligado
int a1,b1,c1,a2,b2,c2;//interruptores da ponte trifásica, 1- first time T1, 2- second time T2
double pwm_a=0.0,pwm_b=0.0,pwm_c=0.0;//pwm_a- racio time of T that superior leg of a is open, inferior leg complementar

ofstream T1T2 ("T1T2.txt");

float w_ref=0.0;
double b, end_f;
double vel_cel = 0.04;

float Tm=0.0;

double torque_clutch=0.0;

double Vds_T_1=0.0;
double Vds_T=0.0;
double Vqs_T_1=0.0;
double Vqs_T=0.0;
double angle_get_wm_T_1=0.0;
double angle_get_wm_T=0.0;
double wr_T_1=0.0;
double wr_T=0.0;
//speed up code
#define T_LOAD_1_sec 0.135/T


double dy_nt_(double /*&*/y1, double /*&*/y_1){//y1 significa y[n+1] 
	return ((y1-y_1)/(2*T));};

double d2y_nt_(double /*&*/y1, double /*&*/y,double /*&*/y_1){//derived can only be obtained  when n>=? 
	return ((y1-2*y+y_1)/(T*T));};


float max_mod_v_ref=0.0;
float ang=0.0;
void calc_max_mod_v_ref(tTwoPhase v_bi){
ang=atan2(v_bi.beta,v_bi.alpha);

	//cout<<"angulo"<<ang<<endl;
	T1T2<<"angulo: "<<ang<<endl;

	double ang_u = 0.0;
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
	//else if (ang>= 5*PI/6 && ang<PI))
			ang_u = ang - 5*PI/6;
	
	
	else if (ang>=-PI && ang<(-2*PI/3))
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
					
	max_mod_v_ref = VDC/(cos(ang_u)*CONST_SQRT3_);//TODO uncoment in the end cos for max 
T1T2 << "max_mod_v_ref "<< max_mod_v_ref<<endl;
};

////////To implement SVPWM:
void svpwm(control_loops & control_){//tTwoPhase- bifasico, tendo como referencia o estator
	tTwoPhase v=control_.get_V();
	double RotorFluxAngle_=control_.get_rot_fl_an();
	T1T2 <<"valpha"<<v.alpha<<"vbeta"<<v.beta<<endl<<"VDQ.q_i:"<<VDQ.q-const_VDQ_q<<endl;
	calc_max_mod_v_ref(v);
	bool none=false;
	tTwoPhaseDQ VDQ_t(VDQ);
	
	//analitico
	if (0==1 && (VDQ.d*VDQ.d+VDQ.q*VDQ.q) > max_mod_v_ref*max_mod_v_ref ){
		VDQ.q=(VDQ.d*(sin(RotorFluxAngle_)+CONST_SQRT3_*cos(RotorFluxAngle_))+const_VDQ_q*(cos(RotorFluxAngle_)+CONST_SQRT3_*sin(RotorFluxAngle_))-2.0/CONST_SQRT3_*VDC)/(-cos(RotorFluxAngle_)+CONST_SQRT3_*sin(RotorFluxAngle_));
		VDQ_t.q=VDQ.q+const_VDQ_q;
		v=InvPark((RotorFluxAngle_),(VDQ_t));//TODO remove embedded
		T1T2 <<"try1"<<"VDQ.q"<<VDQ.q<<endl;
		T1T2<<(-2.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q)<<endl<<(-1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q)<<endl<<v.beta<<endl;
		if (!(-2.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q<VDQ.q) 
			|| !(-1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q>VDQ.q)
			|| !(v.beta>0))				
				{VDQ.q=((-VDQ.d*sin(RotorFluxAngle_)/cos(RotorFluxAngle_))-const_VDQ_q+VDC/CONST_SQRT3_/cos(RotorFluxAngle_));	
				
				VDQ_t.q=VDQ.q+const_VDQ_q;
				v=InvPark((RotorFluxAngle_),(VDQ_t));//TODO remove embedded
				T1T2 <<"try2"<<endl;
				if (!(1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q>VDQ.q) 
					|| !(-1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q<VDQ.q)
					|| !(v.beta>0)){
						VDQ.q=(VDQ.d*(sin(RotorFluxAngle_)-CONST_SQRT3_*cos(RotorFluxAngle_))+const_VDQ_q*(cos(RotorFluxAngle_)-CONST_SQRT3_*sin(RotorFluxAngle_))-2.0/CONST_SQRT3_*VDC)/(-cos(RotorFluxAngle_)-CONST_SQRT3_*sin(RotorFluxAngle_));
						VDQ_t.q=VDQ.q+const_VDQ_q;
						v=InvPark((RotorFluxAngle_),(VDQ_t));//TODO remove embedded
						T1T2 <<"try3"<<endl;
									
					if (!(1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q<VDQ.q) 
						|| !(2.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q>VDQ.q)
						|| !(v.beta>0)){
							VDQ.q=(VDQ.d*(sin(RotorFluxAngle_)+CONST_SQRT3_*cos(RotorFluxAngle_))+const_VDQ_q*(cos(RotorFluxAngle_)+CONST_SQRT3_*sin(RotorFluxAngle_))-2.0/CONST_SQRT3_*VDC)/(-cos(RotorFluxAngle_)+CONST_SQRT3_*sin(RotorFluxAngle_));
							VDQ_t.q=VDQ.q+const_VDQ_q;
							v=InvPark((RotorFluxAngle_),(VDQ_t));//TODO remove embedded
							T1T2 <<"try4"<<endl;
							
						if (!(1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q<VDQ.q) 
							|| !(2.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q>VDQ.q)
							|| !(v.beta<0)){
								VDQ.q=((-VDQ.d*sin(RotorFluxAngle_)/cos(RotorFluxAngle_))-const_VDQ_q-VDC/CONST_SQRT3_/cos(RotorFluxAngle_));
								VDQ_t.q=VDQ.q+const_VDQ_q;
								v=InvPark((RotorFluxAngle_),(VDQ_t));//TODO remove embedded
								T1T2 <<"try5"<<endl;
								
								if (!(1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q>VDQ.q) 
									|| !(-1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q<VDQ.q)
									|| !(v.beta<0)){
										VDQ.q=(VDQ.d*(sin(RotorFluxAngle_)-CONST_SQRT3_*cos(RotorFluxAngle_))+const_VDQ_q*(cos(RotorFluxAngle_)-CONST_SQRT3_*sin(RotorFluxAngle_))+2.0/CONST_SQRT3_*VDC)/(-cos(RotorFluxAngle_)-CONST_SQRT3_*sin(RotorFluxAngle_));
										VDQ_t.q=VDQ.q+const_VDQ_q;
										v=InvPark((RotorFluxAngle_),(VDQ_t));//TODO remove embedded
										T1T2 <<"try6"<<endl;
								
										if (!(-2.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q<VDQ.q) 
											|| !(-1.0/3.0*VDC/sin(RotorFluxAngle_)+VDQ.d*cos(RotorFluxAngle_)/sin(RotorFluxAngle_)-const_VDQ_q>VDQ.q)
											|| !(v.beta<0)){
												none=true;
												T1T2<<"none found"<<endl;}
									}
								}
							}
						}
					}			
		T1T2<<"VDQ.q:"<<VDQ.q<<" VDQ_ant.q:"<<VDQ_ant.q<<endl;
		if (none){VDQ.q=VDQ_ant.q+const_VDQ_q;}
		else VDQ.q+=const_VDQ_q;
		T1T2<<"VDQ_ant.q+const_VDQ_q"<<VDQ_ant.q+const_VDQ_q<<"VDQ.q"<<VDQ.q<<"VDQ_ant.d+const_VDQ_d"<<VDQ_ant.d+const_VDQ_d<<"VDQ.d"<<VDQ.d<<endl;
		v=InvPark(RotorFluxAngle_,VDQ);//TODO remove embedded
		T1T2<<"novo vbeta_alpha "<<v.beta<<" "<<v.alpha<<"rfa"<<RotorFluxAngle_<<endl;										
		calc_max_mod_v_ref(v);					
		
	}
	if (0==1 && (VDQ.d*VDQ.d+VDQ.q*VDQ.q) > max_mod_v_ref*max_mod_v_ref ){
		VDQ.q=VDQ_ant.q+const_VDQ_q;
	
		calc_max_mod_v_ref(InvPark((RotorFluxAngle_),VDQ));
		v=InvPark((RotorFluxAngle_),VDQ);//TODO remove embedded
	}		
	if (1==1&& ((VDQ.d*VDQ.d+VDQ.q*VDQ.q)) > max_mod_v_ref*max_mod_v_ref ){
		T1T2 <<"exceeded by:"<<sqrt(VDQ.d*VDQ.d+VDQ.q*VDQ.q)<<endl;
		if (v.alpha>2.0/3.0*VDC || v.alpha<-2.0/3.0*VDC){	
		
			v.beta = sin(ang)*max_mod_v_ref;
			v.alpha = cos(ang)*max_mod_v_ref;
		}
		else if (v.alpha<=2.0/3.0*VDC && v.alpha>VDC/3.0)
			{
				if (v.beta>0)v.beta=(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC-v.alpha)/tan(PI/2.0-PI/3.0);
			}
			else if(v.alpha>-VDC/3.0 && v.alpha<=VDC/3.0){
					if (v.beta>0)v.beta=VDC/CONST_SQRT3_;else /*if(v.beta<0)*/ v.beta=-VDC/CONST_SQRT3_;
					}
				 else if(v.alpha<=-VDC/3.0 && v.alpha>=-2.0/3.0*VDC){
						if (v.beta>0)v.beta=(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);else v.beta=-(2.0/3.0*VDC+v.alpha)/tan(PI/2.0-PI/3.0);
						}
		calc_max_mod_v_ref(v);	  
		if (((v.beta*v.beta+v.alpha*v.alpha)) > max_mod_v_ref*max_mod_v_ref ){
			v.beta = sin(ang)*max_mod_v_ref;
			v.alpha = cos(ang)*max_mod_v_ref;
		
		}
	}
	
	
	if (ang>=0&&ang<(PI/3)){
			T1=3.0/2.0*T/VDC*(v.alpha-v.beta/CONST_SQRT3_);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(2*VDC/3,-VDC/3,-VDC/3)).alpha)*T;
			a1=1;b1=0;c1=0;	//ozz 
			T2=CONST_SQRT3_*T/VDC*v.beta;//T2=3*T/(2*VDC)*(v.alpha-1/CONST_SQRT3_*v.beta);//T2=abs(v.beta)/abs((Clarke(tThreePhase(VDC/3,VDC/3,-2*VDC/3)).beta)*T);
			a2=1;b2=1;c2=0	;/*v_ooz*/
			
			if ((T1+T2)<T) pwm_a=(T1+T2)/T; else pwm_a=1;
			if ((T1+T2)<T) pwm_b=T2/T; else pwm_b=T2/(T1+T2);
			pwm_c=0;
	}
	else if(ang>=(PI/3)&&ang<(2.0*PI/3)){
			T1=3.0*T/2.0/VDC*(v.alpha+1/CONST_SQRT3_*v.beta);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(VDC/3,VDC/3,-2*VDC/3)).alpha)*T;
			a1=1;b1=1;c1=0;//ooz 
			T2=3.0*T/VDC/2.0*(v.beta/CONST_SQRT3_-v.alpha);
			a2=0;b2=1;c2=0;/*v_zoz*/
			
			if ((T1+T2)<T) pwm_a=T1/T; else pwm_a=T1/(T1+T2);
			if ((T1+T2)<T) pwm_b=(T1+T2)/T; else pwm_b=1;
			pwm_c=0;
	}
	else if(ang>=(2.0*PI/3)&&ang<(PI)){
			T1=T*CONST_SQRT3_/VDC*v.beta;//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-VDC/3,2*VDC/3,-VDC/3)).alpha)*T;
			a1=0;b1=1;c1=0;//zoz 
			T2=CONST_SQRT3_/2.0*T/VDC*(-v.beta-CONST_SQRT3_*v.alpha);//T2=abs(v.beta)/abs(Clarke(tThreePhase(-2*VDC/3,VDC/3,VDC/3)).beta)*T/*v_zoo*/;
			a2=0;b2=1;c2=1;
			
			pwm_a=0;
			if ((T1+T2)<T) pwm_b=(T1+T2)/T; else pwm_b=1;
			if ((T1+T2)<T) pwm_c=T2/T; else pwm_c=T2/(T1+T2);
	}
	else if(ang>=(-PI)&&ang<(-2.0*PI/3)){
			T1=3.0*T/2.0/VDC*(-v.alpha+v.beta/CONST_SQRT3_);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-2*VDC/3,VDC/3,VDC/3)).alpha)*T;
			a1=0;b1=1;c1=1;//zoo 
			T2=-CONST_SQRT3_*T/VDC*v.beta;//T2=abs(v.beta)/abs(Clarke(tThreePhase(-VDC/3,-VDC/3,2*VDC/3)).beta)*T/*v_zzo*/;
			a2=0;b2=0;c2=1;
			
			pwm_a=0;
			if ((T1+T2)<T) pwm_b=T1/T; else pwm_b=T1/(T1+T2);
			if ((T1+T2)<T) pwm_c=(T1+T2)/T; else pwm_c=1;			
	}
	else if(ang>=(-2*PI/3)&&ang<(-PI/3)){
			T1=CONST_SQRT3_*T/2.0/VDC*(-v.alpha*CONST_SQRT3_-/*2*/v.beta);//T1=abs(v.alpha)/abs(Clarke(tThreePhase(-VDC/3,-VDC/3,2*VDC/3)).alpha)*T;
			a1=0;b1=0;c1=1;//zzo 
			T2=CONST_SQRT3_*T/2.0/VDC*(-v.beta+v.alpha*CONST_SQRT3_);//T2=abs(v.beta)/abs(Clarke(tThreePhase(VDC/3,-2*VDC/3,VDC/3)).beta)*T/*v_ozo*/;
			a2=1;b2=0;c2=1;
			
			if ((T1+T2)<T) pwm_a=T2/T; else pwm_a=T2/(T1+T2);
			pwm_b=0;
			if ((T1+T2)<T) pwm_c=(T1+T2)/T; else pwm_c=1;
	}
	else /*if(ang>=(-PI/3)&&ang<(0))*/{
			T1=-1.0*T/VDC*CONST_SQRT3_*v.beta;//T1=abs(v.alpha)/abs(Clarke(tThreePhase(VDC/3,-2*VDC/3,VDC/3)).alpha)*T;
			a1=1;b1=0;c1=1;//ozo 
			T2=3.0*T/2.0/VDC*(v.beta/CONST_SQRT3_+v.alpha);//T2=abs(v.beta)/abs(Clarke(tThreePhase(2*VDC/3,-VDC/3,-VDC/3)).beta)*T/*v_ozz*/;
			a2=1;b2=0;c2=0;
			
			if ((T1+T2)<T) pwm_a=(T1+T2)/T; else pwm_a=1;
			pwm_b=0;
			if ((T1+T2)<T) pwm_c=T1/T; else pwm_c=T1/(T1+T2);		
	};
	 	
			
	 T0=T-(T1+T2);//T0 time of period that are off 1, 1, 1 or 0, 0, 0
	//doc.: para cada periodo T: T1 de tempo com os respectivos a1, b1, c1; T2 de tempo com os respectivos a2, b2, c2. O interruptor correspondente há outra parte da "perna" será o complementar;
	//TIMERS
	 //a1,T1; a2,T2 	;a0,T0;   a off
	 //b1,T1; b2,T2 	;b0,T0;   and b off
	 //c1,T1 ;c2,T2 	;c0,T0;   and c off
	//OR: PWMs
	//ofstream T1T2 ("T1T2.txt",ios::app); 
	//cout<<" T1:"<<T1<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<T2<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<T0<<endl;
	//cout<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;
	T1T2<<" T1:"<<T1<<" a1:"<<a1<<" b1:"<<b1<<" c1:"<<c1<<"   T2:"<<T2<<" a2:"<<a2<<" b2:"<<b2<<" c2:"<<c2<<"    T0:"<<T0<<endl;
	T1T2<<" pwm_a:"<<pwm_a<<" pwm_b:"<<pwm_b<<" pwm_c:"<<pwm_c<<endl;
v_=v;//TODO remove in embedded

};

////!!!!!!!!!!!!!!double dy_nt(double &y1, double &y_1);//y1 means y[n+1] 
	//return dynt=(y1-y_1)/(2*per);

////!!!!!!!-tirar isto final-double d2y_nt(double &y1, double &y,double &y_1);//derived can only be obtained  when n>=? 
//extern double ro;//0.999863//1-M*M/(Ls*Lr)//alterar nome
//extern double B;//M/(ro*Ls*Lr)
//extern double roLs;//0.013998//ro*Ls
//extern double MB;//0.69851//B*M
//extern double one_p_MB;//1.69851//1+MrB
double Lsro=(ro*Ls);//0.00081261
double Lrro=(ro*Lr);//0.0021891
#define R 0.34 //whell radius
#define MASSA (3500.0)//massa veiculo
/*#define*/double T_G_R = (4.313/*2.33*//*1.436 1 0.789*/*4.1/*1.9*/); //total gear racio. (gear_box*final_gear*"reduction_low_ratio")
double T_G_R_ant=T_G_R;	
double gradiente=0;//-0.25;//0.15;
#define G 9.81
#define C_D 0.5
#define RO_AIR 1.25
#define C_R 0.01
#define A_F 2
double sum_power_out=0.001;
double sum_power_in=0.000001;
double Torq_net=0.0;
double sum_power_trac =0.0;
double sum_power_battery = 0.0;
bool e=true;
int n_rtc=0;
double T_G_R_b_clutch=T_G_R_ant;//thing its not needed
//double Vf=0.0;
double Te=0.0;//torque motor
double J2=0.0;//inercia vehicle??
double w_ref_ant=0.0;

using namespace std;
using namespace Maths;

//tried runge-kutta method but doesnt work: f(x)=f(t) = (por ex.) Vds(t),necessitaria-se de f(x+h/2)=f(t+T/2), (update.: experimentei o seguinte e nao funcionou)uma solucao seria h=2*T, guardando os valores de T-1, T; ex. Vds_T_1=Vds_T . torque generated frame synchronous rotor
double motor::torque_z(double Vds,double Vqs,double angle_get_wm){
			cout<<"vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
	double Lls=Ls-M;
		if (angle_get_wm==0.0)angle_get_wm=0.00000000001;
		double	s_2=(angle_get_wm-wr*np)/angle_get_wm*(angle_get_wm-wr*np)/angle_get_wm;
		double Rfe=Rm/(s_2+1);//325;		
		//usado no euler:
			
		double fds_=1/(1/T+Rs/Lls)*(Vds+fds/T+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		double fqs_=1/(1/T+Rs/Lls)*(Vqs+fqs/T-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		double fdr_=1/(1/T+Rr/Llr)*(fdr/T+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm);
		double fqr_=1/(1/T+Rr/Llr)*(fqr/T-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm);
		double Idm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm/*-np*wr*/)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+Iqm/T-(angle_get_wm/*-np*wr*/)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;

		//double fds_k1=T/(1/T+Rs/Lls)*(Vds+fds/T+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		//double fqs_k1=T/(1/T+Rs/Lls)*(Vqs+fqs/T-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		//double fdr_k1=T/(1/T+Rr/Llr)*(fdr/T+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm);
		//double fqr_k1=T/(1/T+Rr/Llr)*(fqr/T-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm);
		//double Idm_k1=T/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm/*-np*wr*/)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		//double Iqm_k1=T/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+Iqm/T-(angle_get_wm/*-np*wr*/)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;

		//double fds_k2=T/(1/T+Rs/Lls)*((Vds+T/2)+(fds+fds_k1/2)/T+/*np*wr*/angle_get_wm*(fqs+fqs_k1/2)+Rs*M/Lls*(Idm+Idm_k1/2));
		//double fqs_k2=T/(1/T+Rs/Lls)*((Vqs+T/2)+(fqs+fqs_k1/2)/T-/*np*wr*/angle_get_wm*(fds+fds_k1/2)+Rs*M/Lls*(Iqm+Iqm_k1/2));
		//double fdr_k2=T/(1/T+Rr/Llr)*((fdr+fdr_k1/2)/T+(angle_get_wm-np*wr)*(fqr+fqr_k1/2)+M*Rr/Llr*(Idm+Idm_k1/2));
		//double fqr_k2=T/(1/T+Rr/Llr)*((fqr+fqr_k1/2)/T-(angle_get_wm-np*wr)*(fdr+fdr_k1/2)+M*Rr/Llr*(Iqm+Iqm_k1/2));
		//double Idm_k2=T/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*(fds+fds_k1/2)+Rfe/(M*Llr)*(fdr+fdr_k1/2)+(Idm+Idm_k1/2)/T+(angle_get_wm/*-np*wr*/)*(Iqm+Iqm_k1/2));//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		//double Iqm_k2=T/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*(fqs+fqs_k1/2)+Rfe/(M*Llr)*(fqr+fqr_k1/2)+(Iqm+Iqm_k1/2)/T-(angle_get_wm/*-np*wr*/)*(Idm+Idm_k1/2));//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;

		//_______________
		
		
		double fds_k1=2.0*T*(Vds_T_1-fds*Rs/Lls+/*np*wr*/angle_get_wm_T_1*fqs+Rs*M/Lls*Idm);
		double fqs_k1=2.0*T*(Vqs_T_1-fqs*Rs/Lls-/*np*wr*/angle_get_wm_T_1*fds+Rs*M/Lls*Iqm);
		double fdr_k1=2.0*T*(-fdr*Rr/Llr+(angle_get_wm_T_1-np*wr_T_1)*fqr+M*Rr/Llr*Idm);
		double fqr_k1=2.0*T*(-fqr*Rr/Llr-(angle_get_wm_T_1-np*wr_T_1)*fdr+M*Rr/Llr*Iqm);
		double Idm_k1=2.0*T*(Rfe/(Lls*M)*fds-Idm*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*fdr+(angle_get_wm_T_1/*-np*wr*/)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k1=2.0*T*(Rfe/(Lls*M)*fqs-Iqm*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*fqr-(angle_get_wm_T_1/*-np*wr*/)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
		
		double fds_k2=2.0*T*((Vds_T)-(fds+fds_k1/2)*Rs/Lls+/*np*wr*/(angle_get_wm_T)*(fqs+fqs_k1/2)+Rs*M/Lls*(Idm+Idm_k1/2));
		double fqs_k2=2.0*T*((Vqs_T)-(fqs+fqs_k1/2)*Rs/Lls-/*np*wr*/(angle_get_wm_T)*(fds+fds_k1/2)+Rs*M/Lls*(Iqm+Iqm_k1/2));
		double fdr_k2=2.0*T*(-(fdr+fdr_k1/2)*Rr/Llr+((angle_get_wm_T)-(np*wr_T))*(fqr+fqr_k1/2)+M*Rr/Llr*(Idm+Idm_k1/2));
		double fqr_k2=2.0*T*(-(fqr+fqr_k1/2)*Rr/Llr-((angle_get_wm_T)-(np*wr_T))*(fdr+fdr_k1/2)+M*Rr/Llr*(Iqm+Iqm_k1/2));
		double Idm_k2=2.0*T*(Rfe/(Lls*M)*(fds+fds_k1/2)-(Idm+Idm_k1/2)*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*(fdr+fdr_k1/2)+((angle_get_wm_T)/*-np*wr*/)*(Iqm+Iqm_k1/2));//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k2=2.0*T*(Rfe/(Lls*M)*(fqs+fqs_k1/2)-(Iqm+Iqm_k1/2)*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*(fqr+fqr_k1/2)-((angle_get_wm_T)/*-np*wr*/)*(Idm+Idm_k1/2));//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
		
		double fds_k3=2.0*T*((Vds_T)-(fds+fds_k2/2)*Rs/Lls+/*np*wr*/(angle_get_wm_T)*(fqs+fqs_k2/2)+Rs*M/Lls*(Idm+Idm_k2/2));
		double fqs_k3=2.0*T*((Vqs_T)-(fqs+fqs_k2/2)*Rs/Lls-/*np*wr*/(angle_get_wm_T)*(fds+fds_k2/2)+Rs*M/Lls*(Iqm+Iqm_k2/2));
		double fdr_k3=2.0*T*(-(fdr+fdr_k2/2)*Rr/Llr+((angle_get_wm_T)-(np*wr_T))*(fqr+fqr_k2/2)+M*Rr/Llr*(Idm+Idm_k2/2));
		double fqr_k3=2.0*T*(-(fqr+fqr_k2/2)*Rr/Llr-((angle_get_wm_T)-(np*wr_T))*(fdr+fdr_k2/2)+M*Rr/Llr*(Iqm+Iqm_k2/2));
		double Idm_k3=2.0*T*(Rfe/(Lls*M)*(fds+fds_k2/2)-(Idm+Idm_k2/2)*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*(fdr+fdr_k2/2)+((angle_get_wm_T)/*-np*wr*/)*(Iqm+Iqm_k2/2));//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k3=2.0*T*(Rfe/(Lls*M)*(fqs+fqs_k2/2)-(Iqm+Iqm_k2/2)*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*(fqr+fqr_k2/2)-((angle_get_wm_T)/*-np*wr*/)*(Idm+Idm_k2/2));		
		
		double fds_k4=2.0*T*((Vds/*+T*/)-(fds+fds_k3)*Rs/Lls+/*np*wr*/(angle_get_wm/*+T*/)*(fqs+fqs_k3)+Rs*M/Lls*(Idm+Idm_k3));
		double fqs_k4=2.0*T*((Vqs/*+T*/)-(fqs+fqs_k3)*Rs/Lls-/*np*wr*/(angle_get_wm/*+T*/)*(fds+fds_k3)+Rs*M/Lls*(Iqm+Iqm_k3));
		double fdr_k4=2.0*T*(-(fdr+fdr_k3)*Rr/Llr+((angle_get_wm/*+T*/)-(np*wr/*+T*/))*(fqr+fqr_k3)+M*Rr/Llr*(Idm+Idm_k3));
		double fqr_k4=2.0*T*(-(fqr+fqr_k3)*Rr/Llr-((angle_get_wm/*+T*/)-(np*wr/*+T*/))*(fdr+fdr_k3)+M*Rr/Llr*(Iqm+Iqm_k3));
		double Idm_k4=2.0*T*(Rfe/(Lls*M)*(fds+fds_k3)-(Idm+Idm_k3)*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*(fdr+fdr_k3)+((angle_get_wm/*+T*/)/*-np*wr*/)*(Iqm+Iqm_k3));//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k4=2.0*T*(Rfe/(Lls*M)*(fqs+fqs_k3)-(Iqm+Iqm_k3)*Rfe*(1.0/Lls+1.0/Llr+1.0/M)+Rfe/(M*Llr)*(fqr+fqr_k3)-((angle_get_wm/*+T*/)/*-np*wr*/)*(Idm+Idm_k3));		
		
		
		double fds_1=fds+1.0/6.0*(fds_k1+2.0*fds_k2+2.0*fds_k3+fds_k4);
		double fqs_1=fqs+1.0/6.0*(fqs_k1+2.0*fqs_k2+2.0*fqs_k3+fqs_k4);
		double fdr_1=fdr+1.0/6.0*(fdr_k1+2.0*fdr_k2+2.0*fdr_k3+fdr_k4);
		double fqr_1=fqr+1.0/6.0*(fqr_k1+2.0*fqr_k2+2.0*fqr_k3+fqr_k4);
		double Idm_1=Idm+1.0/6.0*(Idm_k1+2.0*Idm_k2+2.0*Idm_k3+Idm_k4);
		double Iqm_1=Iqm+1.0/6.0*(Iqm_k1+2.0*Iqm_k2+2.0*Iqm_k3+Iqm_k4);
		

		fds=fds_1;
		fqs=fqs_1;
		Idm=Idm_1;
		Iqm=Iqm_1;
		fdr=fdr_1;
		fqr=fqr_1;
		
		iqs=(fqs-M*Iqm)/Lls;
		ids=(fds-M*Idm)/Lls;
		
		Vds_T_1=Vds_T;
		Vds_T=Vds;
		Vqs_T_1=Vqs_T;
		Vqs_T=Vqs;
		angle_get_wm_T_1=angle_get_wm_T;
		angle_get_wm_T=angle_get_wm;
		wr_T_1=wr_T;
		wr_T=wr;
				
		////idr=(fdr-M*Idm)/Llr;
		////iqr=-M*Iqm/Llr;
		
		if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_out += abs(IDC*VDC) ; else sum_power_out += (t*wr);
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_in += abs(t*wr); else sum_power_in += (IDC*VDC);
	sum_power_trac += t*wr;
	sum_power_battery += IDC*VDC/10000/3600;fTorque<<" battery energy : "<<sum_power_battery<<" (wh)"<<endl;
	if (n*T==195)fTorque<<" efficiency end cycle: "<<sum_power_out/sum_power_in<<endl;
	fTorque<<FIXED_FLOAT(n*T)<<" sec.; Torque_generated (previous): "<<t/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl<<" power (Torq*wr) (previous): "<<(t/*np*/*wr)<<" power (Vdc*Idc) (previous): "<<(IDC*VDC)<<endl;if (IDC<0) fTorque<<" efficiency (instant. generat.) (previous) :"<<(IDC*VDC)/(t/*np*/*wr);else fTorque<<" efficiency (instant. motor)(previous):"<<(t/*np*/*wr)/(IDC*VDC);fTorque<<" machine medium efficiency: "<<sum_power_out/sum_power_in/*<<" ef: "<<sum_power_trac/sum_power_battery*/<<endl<<"We- sincronous speed in ang. electric: "<<angle_get_wm<<endl<<"(angle_get_wm-wr*np) \"slip angle:\" "<<(angle_get_wm-wr*np)<<endl;//<<"torq_l:"<<torq_L<<endl;	
	t=(3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds)));
	return /*M/Llr*fdr*Iqm*/t;
};

double motor::torque_g(double Vds,double Vqs,double angle_get_wm){//torque generated, frame generic . used #######"backward euler method"######
			cout<<"Vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
	double Lls=Ls-M;
		if (angle_get_wm==0.0)angle_get_wm=0.00000000001;
		double	s_2=(angle_get_wm-wr*np)/angle_get_wm*(angle_get_wm-wr*np)/angle_get_wm;
		double Rfe=Rm/(s_2+1);		
		//usado (fds_n+1-fds_n)/T=fds_n+1 +(...), e nao: (fds_n+1/T-fds_n)/T=fds_n+(...)
		double fds_=1/(1/T+Rs/Lls)*(Vds+fds/T+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		double fqs_=1/(1/T+Rs/Lls)*(Vqs+fqs/T-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		double fdr_=1/(1/T+Rr/Llr)*(fdr/T+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm);
		double fqr_=1/(1/T+Rr/Llr)*(fqr/T-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm);
		double Idm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm/*-np*wr*/)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+Iqm/T-(angle_get_wm/*-np*wr*/)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;

		
		fds=fds_;
		fqs=fqs_;
		fdr=fdr_;
		fqr=fqr_;
		Idm=Idm_;
		Iqm=Iqm_;
		
		ids=(fds-M*Idm)/Lls;
		iqs=(fqs-M*Iqm)/Lls;
		
		
	
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_out += abs(IDC*VDC) ; else sum_power_out += (t*wr);
	if (/*(t*wr)<0 &&*/ (IDC/*VDC*/)<0) sum_power_in += abs(t*wr); else sum_power_in += (IDC*VDC);
	sum_power_trac += t*wr;
	sum_power_battery += IDC*VDC/10000/3600;fTorque<<"battery energy : "<<sum_power_battery<<" wh"<<endl;
	if (n*T==195)fTorque<<"efficiency end cycle "<<sum_power_out/sum_power_in<<endl;
	fTorque<<FIXED_FLOAT(n*T)<<" sec., Torque_generated (previous): "<<t/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl<<"power (Torque_generated*rotor_velocity)(previous):"<<(t/*np*/*wr)<<"; power (Vdc*Idc)(previous):"<<(IDC*VDC)<<endl;if (IDC<0) fTorque<<"efficiency (instant. generat.)(previous) :"<<(IDC*VDC)/(t/*np*/*wr);else fTorque<<" efficiency (instant. motor)(previous):"<<(t/*np*/*wr)/(IDC*VDC);fTorque<<" machine medium efficiency: "<<sum_power_out/sum_power_in/*<<" ef: "<<sum_power_trac/sum_power_battery*/<<endl<<"We- sincronous speed in ang. electric: "<<angle_get_wm<<endl<<"(angle_get_wm-wr*np) \"slip angle:\" "<<(angle_get_wm-wr*np)<<endl;//<<"torq_l:"<<torq_L<<endl;	
	t=(3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds)));
	return t;
};


long motor::get_n(){return n;};
double motor::get_torq_g(){return torq_g;};
double motor::get_m_Tr_calc_gl(){return m_Tr_calc_gl;};

void motor::set_theta_r(double t){theta_r=t;};

motor::motor():idf(0.0),iqf(0.0),RotorFluxAngle(0.0),Idm(0.0),Iqm(0.0),d_Idm(0.0),d_Iqm(0.0),angle(0.0),abc_current(0.0,0.0,0.0){/*Tr_calc=0.17703,*//*TR=0.252308*//*0.17703*//*1.12252 Lr/Rr*/;Rs=0.012/*0.10941*//*0.258*//*0.435*/;/*alfa=(Rs/roLs+M*M/(roLs*Lr*TR));*/n=0;ids=0.0;iqs=0.0;idr=0.0;iqr=0.0;wr=0.00000001;Rr=0.0065/*3.9*/;fds=0.0;fqs=0.0;fdr=0.0;fqr=0.0;torq_L=0;theta_r=0.0;torq_g=0.00000001;};
motor::~motor(){};
double motor::torque(double vas,double vbs,double vcs){
		return torque_g(ClarkePark(RotorFluxAngle/*np*theta_r*/,tThreePhase(vas,vbs,vcs)).d/*alpha*/,ClarkePark(RotorFluxAngle/*np*theta_r*/,tThreePhase(vas,vbs,vcs)).q/*beta*/,angle.get_wm());
};

void control_loops::clutch(){//
	double J1=J;
	//double J2=MASSA*R*R/T_G_R/T_G_R;
	//double Te=torque(va,vb,vc);
	double T_l=get_torq_L();
	double n1=get_wr();
	//Torq_net=Te-T_l;
	double n2=velocidade;//??
	double Wfc=n2-n1;
	
	double Ru=1.0/(R2-R1)*(us/2.0*R2*R2+(ud-us)*(R2+1.0)/(fi*Wfc)*log(cosh(fi*R2*Wfc))-(us/2.0*R1*R1+(ud-us)*(R1+1.0)/(fi*Wfc)*log(cosh(fi*R1*Wfc))));
	double Xto=XTO_MAX/(CLUTCH_TIME/T)*clutch_time;
	double Ffc=F_MAX;
	if (Xto>0.0 && Xto<=XTO_CNT)
		{Ffc=0.0;}
	else if(Xto>XTO_CNT && Xto<=XTO_CLS)
			Ffc=F_MAX*(1.0-sqrt(1.0-pow((Xto-XTO_CNT)/(XTO_CLS-XTO_CNT),2)));		 		
	double Tfc=-N_P*Ru*Ffc/T_G_R;
	
	n1=n1+(Te-Tfc)/J1*T;
	n2=n2+(Tfc-T_l)/J2*T;
	
	velocidade=n2;
	wr=n1;
		fVel<<"Wr(rotor velocity): "<<n1<</*" a: "<<a<<*/" torque_clutch: "<<Tfc<<endl;
}
double motor::get_wr(double va,double vb,double vc/*double torq_g,double torq_L*/){
		//double a=atan(gradiente);//TODO estava comentado este e alinha a seguir --load in ifoc_2_sem_gui
		//torq_L=(C_D*1/2*RO_AIR*A_F*wr/T_G_R*R*wr/T_G_R*R+C_R*MASSA*G*cos(a)+MASSA*G*sin(a))*R/T_G_R;
		Te=torque(va,vb,vc);
		if (e==true && velocidade*R/T_G_R*3.6 < 26){
				T_G_R = (4.313 /*2.33 1.436 1 0.789*/*4.1/*1.9*/);
			if (T_G_R_ant!=T_G_R){
				T_G_R_b_clutch=T_G_R_ant;
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//jan 2019 ;w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
				
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
				//angle.reset();
				}

			}
		if (e==true && velocidade*R/T_G_R*3.6 > 30 && velocidade*R/T_G_R*3.6 < 66 )
			{T_G_R = (/*4.313*/ 2.33/*1.436 1 0.789*/*4.1/*1.9*/);
			if (T_G_R_ant!=T_G_R){
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//jan 2019 ;w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
				
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
				//angle.reset();
				}
			}
		if (e==true && velocidade*R/T_G_R*3.6 > 70 )
			{T_G_R = (/*4.313 2.33*/1.436/* 1 0.789*/*4.1/*1.9*/);
			if (T_G_R_ant!=T_G_R){
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//jan 2019 ;w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
			
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
				//angle.reset();
				}
			}
				
		J2=MASSA*R*R/T_G_R/T_G_R;
		if(e==true){//if driving cycle:
			if (clutch_time>0){
				this->clutch();
				//torq_L-=torque_clutch;
				if (clutch_time==1)w_ref=w_ref_ant;	
				fwref<<FIXED_FLOAT(get_n()*T)<<"clutch_time: "<<clutch_time<<endl;
						
				clutch_time--;
				}
			//if (clutch_time==0){
				//torq_L-=torque_clutch;
				//clutch_time=clutch_time-3;
			//	}
			else if (velocidade>0.0 && (velocidade>wr*1.05 || velocidade<wr*0.95))
					this->clutch();
					else if (velocidade<0.0 && (velocidade<wr*1.05 || velocidade>wr*0.95))
					this->clutch();
				else{
					Torq_net=Te-torq_L;
					wr=wr+(Torq_net)*T/(J+J2/*R*R*MASSA/(T_G_R*T_G_R)*/);
					velocidade=wr;
					}
		}
		else{//if vheicle mode or by load??not sure//T_G_R=1.436*4.1;//TODONEw, test load 3rth
			if (T_G_R_ant!=T_G_R){
				//wr = wr *T_G_R/T_G_R_ant;
				w_ref_ant=w_ref;
				w_ref=0;//w_ref *T_G_R/T_G_R_ant;/*end_f*1000/3600*T_G_R/R*/;
			////	clutch_time=CLUTCH_TIME*10000;
				clutch_time=CLUTCH_TIME*1/T;
				velocidade=velocidade*T_G_R/T_G_R_ant;
				T_G_R_ant=T_G_R;
				n_rtc=0;
			////	//angle.reset();
			}
			if (clutch_time>0){
				this->clutch();
				//torq_L-=torque_clutch;
				clutch_time--;	
				if (clutch_time==1)w_ref=w_ref_ant;	
				fwref<<FIXED_FLOAT(get_n()*T)<<"clutch_time: "<<clutch_time<<endl;
					
				}
			//if (clutch_time==0){
				//torq_L-=torque_clutch;
				//clutch_time-=3;
			//	}
			else {
				if (T_G_R==0){
					wr=wr+(Te/*torque(va,vb,vc)-torq_L*/)*T/J;
					velocidade+=torq_L/J2*T;
					}
				else if (velocidade>0.0 && (velocidade>wr*1.05 || velocidade<wr*0.95))
					this->clutch();
					else if (velocidade<0.0 && (velocidade<wr*1.05 || velocidade>wr*0.95))
					this->clutch();
					else{
						Torq_net=Te/*torque(va,vb,vc)*/-torq_L;
						wr=wr+(Torq_net)*T/(J+J2/*R*R*MASSA/(T_G_R*T_G_R)*/);
						velocidade=wr;
					}
			}	
			}
		return wr;
	};
	
//----------------	the most simple of doing the following, but i dont know if is the more accurate, is measure only IDC, the current flowing from batteries
	double motor::get_ias(){//TODO in real read from sensors
		abc_current.a=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).a);
		return abc_current.a;
	};
	double motor::get_ibs(){//TODO in real read from sensors
		abc_current.b=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).b);
		return abc_current.b;
	};
	double motor::get_ics(){//TODO in real read from sensors or if system simetric and equilibrated: 
		//abc_current.c=-abc_current.a-abc_current.b; but is ??better get abc_current.c from measure
		abc_current.c=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).c);
		return abc_current.c;
	};
//-----------------	
	
	void motor::set_torq_L(double torque){torq_L=torque;};//TODO remove in real
	
	//void motor::set_Rs(double rr){Rs=rr;};
	double motor::get_torq_L(){return torq_L;};//TODO remove in real
	
	double motor::get_wr(){return wr;};//TODO implement in REAL

	//double motor::get_Rs(){return Rs;};




	double control_loops::get_rot_fl_an(){return RotorFluxAngle;};

	control_loops::control_loops():/*v_(0.0,0.0),*//*IDQq_p(0.0),stc_p(0.0),*/ abc_voltage_svpwm(0.0,0.0,0.0),VDQ_rfa(0.0,0.0),/*sobre_Tr_comp(0.0),*/wmr__1(0.0),imrref(0.0),abc_voltage(0.0,0.0,0.0),IDQ(0.0001,0.0001)/*,VDQ(0.0,0.0)*/,/*VDQ_ant(0.0,0.0),*//*rot_fl_an(0.0),*/Theta_r(0.0),w_ref(0.0),V(0.0,0.0),IDQ_rotor(0.0,0.0),VDQ_rotor(0.0,0.0),IDQ_d_1(0.0),IDQ_d_(0.0),IDQ_d1(0.0),IDQ_d_p(0.0),IDQ_d_pp(0.0),IDQ_q_1(0.0),IDQ_q_(0.0),IDQ_q1(0.0),IDQ_q_p(0.0),IDQ_q_pp(0.0),wmr_(0.0),wmr1(0.0),wmr_p(0.0){
			/*Tr_calc_gl=Trrr/*0.17703*//*;m_Tr_calc_gl=Trrr*//*0.17703*/;//TODO insert at the end Tr motor value
			//torque_control_Min_pid_res=-150;//?
			//torque_control_Max_pid_res=150;
			vel_Min_pid_res = -400.0;//600 //-300.0//-170.0				//Min value PI out
			vel_Max_pid_res = 400.0;
			current_control_y_Min_pid_res=-10.0;//(-(2/3)*300);//100?
			current_control_y_Max_pid_res=10.0;//((2/3)*300);// 400//100?
			current_control_x_Min_pid_res=-100;//-VDC/CONST_SQRT3_/*TODO-250*/;//TODO???-400.0;//(-(2/3)*300);// (-400)//100?
			current_control_x_Max_pid_res=100;//VDC/CONST_SQRT3_/*TODO250*/;//TODO??400.0;//((2/3)*300);//400//100?
			vel.tune_pid(vel_p,vel_i,vel_d);/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*/
			torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);
			current_control_y.tune_pid(current_control_y_p,current_control_y_i,current_control_y_d);
			current_control_x.tune_pid(current_control_x_p,current_control_x_i,current_control_x_d);
			//flux_control.tune_pid(flux_control_p,flux_control_i,flux_control_d);
			//for (int i=0;i<9;i++)vec_MediaTr_[i]= m_Tr_calc_gl;
	};
	
	void control_loops::vel_tune_pid(){vel.tune_pid(vel_p,vel_i,vel_d);};
	void control_loops::torque_control_tune_pid(){torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);};
	control_loops::~control_loops(){/*delete ConstTr;*/};
	void control_loops::set_w_ref(double wreff) {w_ref=wreff;};
	//void control_loops::set_imrref(double imrreff){imrref=imrreff;};
	double control_loops::get_w_ref(){return w_ref;};

void control_loops::set_Theta_r(){//set variable theta_r
		Theta_r=Theta_r+this->get_wr()*T;
		while (Theta_r>=(2*PI)||Theta_r<0)
		       {if(Theta_r>=(2*PI)){Theta_r=Theta_r-(2*PI);}
				else{Theta_r=Theta_r+(2*PI);};};
		motor::set_theta_r(Theta_r);
		
};
void control_loops::get_VDC(float vdc){VDC=vdc;Vmax=VDC/sqrt(3);};//TODO adapt in real


	//get_VDQ +++++++++++++++++++++++++++
tTwoPhase control_loops::get_V(/*const*/ ){
		
			//float torque_control_Min_pid_res=-150;//?
			//float torque_control_Max_pid_res=150;
			current_control_y_Min_pid_res=-6.45;//6.9;//8.5//(-2.0)/3*VDC;
			current_control_y_Max_pid_res=6.45;//(2.0/3*VDC);
			current_control_y.act_min_max(current_control_y_Min_pid_res,current_control_y_Max_pid_res);
			current_control_x_Min_pid_res=-100.0;//-VDC/CONST_SQRT3_/*TODO-300*/;/*imr -255.0*/;//((-2.0)/3*VDC);
			current_control_x_Max_pid_res=100.0;//VDC/CONST_SQRT3_/*TODO300*/;//255.0;//(2.0/3*VDC);
			current_control_x.act_min_max(current_control_x_Min_pid_res,current_control_x_Max_pid_res);
			//TODO if (n>=("maximum of long")){n=0;}//n nao é necessario na realidade
			if(n==0){
//PIDs----------------------
				//TODO remove fImr<<"Wn: "<<Wn<<"Wc: "<<Wc<<" Tm1: "<<Tm1<<endl;		
			 vel.init_pid(this->get_wr(),w_ref,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
				vel.calc_pid();

			 torque_control.init_pid(IDQ.q*3.0/2.0*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
				torque_control.calc_pid();

			current_control_y.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
				current_control_y.calc_pid();
			
			//flux_control.init_pid(angle.get_imr(),imrref,flux_control_Min_pid_res,flux_control_Max_pid_res ,flux_control_cel);
			//	flux_control.calc_pid();
			current_control_x.init_pid(IDQ.d,/*flux_control.get_pid_result(),TODO*/Idn,current_control_x_Min_pid_res,current_control_x_Max_pid_res ,current_control_x_cel);
				current_control_x.calc_pid();
			
					VDQ.q=/*torque_control.get_pid_result();*/current_control_y.get_pid_result();//cout<<"VDQ.q:"<<VDQ.q<<endl;//se necessario reduzir codigo podesse eliminar vdq?

// isd.init_pid(IDQ.d,(90/*-5*w_ref*w_ref*w_ref+56*w_ref*w_ref-209*w_ref+300*/)/*TODO:(30%nominal)*/,current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
//	current_control_y.calc_pid();
					VDQ.d=current_control_x.get_pid_result();//cout<<"VDQ.d:"<<VDQ.d<<endl;
				//reset?integral

//+Vdecouple-------------			
			IDQ_d1=IDQ.d;
			
			IDQ_q1=IDQ.q;
			
			wmr1=angle.get_wm();
			//**********
				fTorque<<"power iaa*vaa+ibb*vbb+icc*vcc: "<<abc_current.a*vaa+/*InvClarkePark(RotorFluxAngle,IDQ)*/abc_current.b*vbb+abc_current.c*vcc<<endl;
			//**********
			//TODO colocar n++ aqui?? no final
			}
		else{
//----------------------------------------
			VDQ_ant_1=VDQ;
			IDQ=ClarkePark(RotorFluxAngle,abc_current);//TODO???rfa==0? ??abc_current ta bem?

			//IDQ_rotor=(ClarkePark((np*Theta_r),abc_current));
			
			//the following is used to calc rotor time constant (adapt value at change).
			abc_voltage_svpwm.a=vaa;abc_voltage_svpwm.b=vbb;abc_voltage_svpwm.c=vcc;
			VDQ_rfa=ClarkePark(RotorFluxAngle,abc_voltage_svpwm);
			//VDQ_rotor=(ClarkePark((np*Theta_r),abc_voltage_svpwm));//TODO:ERROR because in real abc_voltage is scaled by svpwm. voltage,done?, sensor?				
			
////---------------------------	
			float IDQq=IDQ.q;float IDQd=IDQ.d;	
		//********** used in simulation to calc cos_phi
				fTorque<<"power iaa*vaa+ibb*vbb+icc*vcc: "<<abc_current.a*vaa+/*InvClarkePark(RotorFluxAngle,IDQ)*/abc_current.b*vbb+abc_current.c*vcc<<endl;
				//fTorque<<"vaa: "<<vaa<<" iaa: "<<abc_current.a<<endl;
			sfData_va_ia<<" "<<FIXED_FLOAT(n*T)<<" "<<vaa<<" "<<abc_current.a<<" "<<abc_current.b<<" "<<abc_current.c<<endl;
			if ( vaa_p >0 && vaa_p*vaa < 0 ) {cos_phi=n*T;fTorque<<"vaa pass by zero"<<endl;}else {fTorque<<"vaa_p"<<vaa_p<<"vaa"<<vaa<<endl;}
			if ( iaa_p >0 && iaa_p*abc_current.a < 0 ) {/*if (desc_p==0.0){desc_p=n*T;} else {*/two_phi=n*T-desc_p;desc_p=n*T;cos_phi=n*T-cos_phi;double cos_phi_a=cos_phi/two_phi*2*PI;
				fTorque<<"two_phi"<<two_phi<<"cos_phi_t "<<cos_phi<<" phi "<<cos_phi_a<<" cos_phi: "<<cos(cos_phi_a)<<endl;} else {fTorque<<"iaa_p"<<iaa_p<<"iaa"<<abc_current.a<<endl;}
		
		//**********
			int sinal_=1;
			
			//to get Rotor flux angle 
			RotorFluxAngle=angle.RotFluxAng_(IDQq,IDQd,T,m_Tr_calc_gl,(this->get_wr()*np));			
			//if (n==50000)RotorFluxAngle=0.0;
			RotorFluxAngle=RotorFluxAngle;//TODO:-PI/2 remove? in simulation doesn't work but in real i think it's needed??
			
//PIDs------------------------------------------- To get Voltage direct and quadracture before decoupling: 

			vel.set_acceleration_limit(vel_cel=ACCEL*1000/3600*T_G_R/R*T);

	/*here*/double Wm = angle.get_wm();//Wm == we, utilizado nos papers, vel sincrona. aten.- electric speed- ..*number pole pairs	
			double Rd_we=Rs+Wm*Wm*M*M/Rm;
			double Rq_we=Rs+/*Rrr */1.0/(m_Tr_calc_gl*Lr)*M*M/*/Lr/Lr*/+Wm/*np*/*Wm/*np*/*M*M*Llr*Llr/Rm/Lr/Lr;//TODO ?substituir online Rs pelo calculado na constante de tempo d rotor 
			float IDQ_d_lma=0.0;
			
			vel.set_process_point(this->get_wr());	//fIDQ<<" get_wr(): "<<this->get_wr()<<" w_ref: "<<w_ref<<endl;		
			vel.set_setpoint(w_ref);
			//vel.calc_pid();

			torque_control.set_process_point(IDQ.q*Kt/*3/2*np*M*M/Lr*/*angle.get_imr());//TODO isto ta bem?
			fTorque<<"torque_process_point: "<< (IDQ.q*Kt/*3/2*np*M*M/Lr*/*angle.get_imr())<<endl;
			//torque_control.set_setpoint(vel.get_pid_result());   								
			
			
			if(Wm < Wn) 
				{
				//max torque limit region:
				if (n>T_LOAD_1_sec) Tm1=TM1;else Tm1=LOAD_1_sec;//TODO remove T in the final and calc, in real maybe this is limited by batteries//TODO maybe limite set speed 
				
						//if(n*T<10)Tm1=200;else Tm1=TM1; //TODO remove at end or adapt
					 Tm=Tm1;
					 vel.act_min_max(-Tm1,Tm1);
					 vel.calc_pid();		
					 fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;
					 
				fTorque<<"<Wn: "<<Wn<<endl;
				fTorque<<"Tm1: "<<Tm1<<endl;
				
				if(abs(IDQ.q)<=sqrt(Rd_we/Rq_we)*Idn)//ok
					{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}//OK absolute value of iqs
				else{IDQ_d_lma=Idn;};
				if (isnanf(IDQ_d_lma)){IDQ_d__lma<<"is not a number IDQ_d_lam 1"<<endl;}
				if ((IDQ_d_lma<Idmin) || isnanf(IDQ_d_lma))IDQ_d_lma=Idmin;
				}
			else {
				//max current(power) limit region:
				if (Wm < Wc ){					
					float Tm2=Kt*sqrt(pow(Vmax/(Wm*Ls),2.0)-Imax*Imax*ro*ro)*sqrt(Imax*Imax-pow(Vmax/(Wm/*r()*np*/*Ls),2.0))/(1.0-ro*ro);
					Tm=Tm2;
					vel.act_min_max(-Tm2,Tm2);
					vel.calc_pid();		
					fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;
					fTorque<<"<Wc: "<<Wc<<endl;
					fTorque<<"Tm2: "<<Tm2<<endl;
					
					if(abs(IDQ.q)<=Vmax/(Wm/*r()*np*/*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {IDQ_d_lma=sqrt(Imax*Imax/2.0-sqrt(Imax*Imax*Imax*Imax-4.0*vel.get_pid_result()*vel.get_pid_result()/Kt/Kt)/2);};
					if (isnanf(IDQ_d_lma)){IDQ_d__lma<<"is not a number IDQ_d_lam 2"<<endl;}
					if ((IDQ_d_lma<Idmin) || isnanf(IDQ_d_lma))IDQ_d_lma=Idmin;
					}
				else {
					//max Power-speed(voltage) limit region:
					float Tm3=0.9/*cag*/*Kt*(pow((Vmax/(Wm/*r()*np*/*Ls)),2.0)/(2.0*ro));
					Tm=Tm3;
					vel.act_min_max(-Tm3,Tm3);
					vel.calc_pid();		
					fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;
					fTorque<<">Wc: "<<Wc<<endl;
					fTorque<<"Tm3: "<<Tm3<<endl;
					
					if(abs(IDQ.q)<=Vmax/(Wm/*r()*np*/*Ls*ro)/sqrt(pow(ro,-2.0)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {
						//double Te=vel.get_pid_result();
						//IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm/*r()*np*/,4.0)*ro*ro*Ls*Ls*Ls*Ls*Te*Te/Kt/Kt))/(2.0*pow(Wm/*r()*np*/*Ls,2.0)));
						IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4.0)-4.0*pow(Wm,4.0)*ro*ro*Ls*Ls*Ls*Ls*vel.get_pid_result()*vel.get_pid_result()/Kt/Kt))/(2.0*pow(Wm*Ls,2.0)));
						
						};
				};
				if (isnanf(IDQ_d_lma)){/*IDQ_d_lma=Idn;*/IDQ_d__lma<<"is not a number IDQ_d_lam 3"<<endl;}
				if ((IDQ_d_lma<Idmin) || isnanf(IDQ_d_lma))IDQ_d_lma=Idmin;
				};
			
			
		
			fTorque << "tc_ref:  "<< vel.get_pid_result() <<endl;
			torque_control.set_setpoint(vel.get_pid_result());
			torque_control.calc_pid();
			fTorque<<"torque current error: "<<torque_control.get_current_error()<<"torque pid result: "<<torque_control.get_pid_result()<<endl;
			
			float iqmax=Tm/Kt/angle.get_imr()/*0.97cag*/;
			
			if (IDQ.q>iqmax)current_control_y.set_process_point(iqmax);
			else
			  current_control_y.set_process_point(IDQ.q);
			current_control_y.set_setpoint(torque_control.get_pid_result());
			current_control_y.calc_pid();
			
			IDQ_d__lma<<FIXED_FLOAT(n*T)<<"sec.; IDQ_D_lma: "<<IDQ_d_lma<<endl<<"IDQ_d: "<<IDQ.d<<endl;
			current_control_x.set_process_point(/*angle.get_imr()*/IDQ.d);// i changed: put imr instead IDQd BUT not best
			current_control_x.set_setpoint(IDQ_d_lma/*flux_control.get_pid_result()*/);//TODO 0.9
			current_control_x.calc_pid();
		
			VDQ.d=current_control_x.get_pid_result();//because tension is not proporcional of currents, used in controll as currents but after decoupling, tension 											   
			
			//float iqmax=Tm/Kt/angle.get_imr()/* *0.99caganco*/;
			cout<<"iqmax: "<<iqmax;
			VDQ.q=current_control_y.get_pid_result()/*torque_control.get_pid_result()*/;			
			if (VDQ.q>0 && VDQ.q>iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/))VDQ.q=iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/);
			else if (VDQ.q<0 && VDQ.q<-iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/))VDQ.q=-iqmax*(Rs*1.1/*+IDQ_q_p*Lps*/);
			VDQ_ant=VDQ;
//--------------Rotor Time Constant value adjust:			
						
			
			fIDQ<<FIXED_FLOAT(n*T)<<" sec.;"<<" VDQ_rfa.d: "<<VDQ_rfa.d<<endl<<" IDQd: "<<IDQd<<" IDQq: "<<IDQq<<endl/*<<"(IDQd^2+IDQq^2)^(1/2): "<<sqrt(IDQd*IDQd+IDQq*IDQq)*//*IDQd+IDQq*/<<"IDC:"<<IDC/*<<"IDC2_3:"<<IDC2_3*/<<endl;
						
			
			//cout<<"VDQ.q:"<<VDQ.q<<endl;
			fIDQ<<"VDQ.d :"<<VDQ.d<<endl<<"VDQ.q: "<<VDQ.q<<endl;
			
			//if (n>200 && n<100000){Rr*=1.0000022/*1/1.0000002*/;};//TODO remove at end, this is to simulate variations of Tr, the real value influences the gain error
			
			//if (n==220){Rr*=1.3;};
			if (n_rtc<90000)n_rtc++;
			if (n_rtc>50000 /*&&the following is because it dont work at 0 power: (IDQq < -100 || IDQq > 100 )*/) {//steady state TODO				n
				
				double error=this->VDQ_rfa.d-(Rs*IDQd-angle.get_wm()*((Ls*Lr-M*M)/Lr)*IDQq);
				//s/*+*/=error;//dont work:integral controller, work proporcional only but need tune de following numeric value 
				//double pi_contr=(s)*/*the following value is the gain, adjusted from the rate of change of Rr(Tr=Lr/Rr)*/0.000004/*0.000025*/;
				if ((IDQq<-10.5*3/*10:maybe more secure 11.5*/ && wr<-/*30*/2 ) || (IDQq>10.5*3 && wr >2) /*10//-44//-40*//*&& VDQ.q>0*/ /*&& (IDQq <= -100 || IDQq >= 100 )*/){
				////if (torque_control.get_pid_result()<0){
				  sinal_=/*-*/1;}
				  else if((IDQq>10.5*3 && wr<-2) || (IDQq<-10.5*3 && wr >2 )){sinal_=-1;} 
						/*else if (IDQq<-10.5 && wr >2 ){sinal_=-1;}*//*else if (IDQq>10.5 && wr >2){sinal_=1;}*/else{sinal_=0;}
				//if (wr<-30){sinal_*=-1;}else if (wr>30){sinal_=1;} else {sinal_=0;};
					//TODO vel velref idqq
				m_Tr_calc_gl=((m_Tr_calc_gl)+sinal_*error*0.000002/*0.000004*//*0.000025*//*pi_contr*/);
	
				fIDQ<<" Tr estimated: "<<m_Tr_calc_gl/*Tr___*/<<" error: "<<error<<endl;
				}; 
			fIDQ<<" TR of motor: "<<Lr/Rr<<endl;
			//////stc_p=VDQ.q;

	//part of, used in function decouple			
	//+Vdecouple-------------To get Voltage direct and quadracture after decoupling	
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

			const_VDQ_d=/*VDQ.d*//*-wmr_1*Lsro*IDQ.q*//*(this is part of usx, the PI output)+Rs*IDQ_d1*/+(/*(idem previous)Lsro+*/T_lag*Rs)*IDQ_d_p+T_lag*Lsro*IDQ_d_pp-wmr1*(Lsro-T_lag*Rs)*IDQ_q1+wmr1*wmr1*(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))/*TODO:modulo imr*/-T_lag*Lsro*IDQ_q1*wmr_p;				
			const_VDQ_q=/*VDQ.q*//*+wmr_1*Lsro*IDQ_d_p*T_lag*//*(this is part of usy, the PI output)+Lsro*IDQ_q_p*//*-wmr1*T_lag*Lsro*IDQ_d_p*//*(ideyntastic_check_on_open = 1 previous)+Rs*IDQ_q1*/+T_lag*Rs*IDQ_q_p+T_lag*Lsro*IDQ_q_pp+wmr1*(Lsro-T_lag*Rs)*IDQ_d1+wmr1*wmr1*T_lag*Lsro*IDQ_q1+(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))*wmr_p+(Ls-Lsro)*wmr1*abs(angle.get_imr());//angle.wmr1*Lsro*IDQ.d+(Ls-Lsro)*wmr1*angle.get_imr();	
			/*here*/
			VDQ.d+=const_VDQ_d;
			VDQ.q+=const_VDQ_q;
//-------------			
			//cout<<"VDQ.d +decoupling:"<<VDQ.d<<endl;
			;fIDQ<<"VDQ.d +decoupling: "<<VDQ.d<<endl;
			//cout<<"VDQ.q +decoupling:"<<VDQ.q<<endl;
			;fIDQ<<"VDQ.q +decoupling: "<<VDQ.q<<endl;
			//VDQ.d=VDQ.d-angle.get_wm()*Lsro*IDQ.q;cout<<"VDQD+"<<VDQ.d<<endl;//Rs*IDQ.d-np*2/3*motor::wr*(Lsro+Lrro)*IDQ.q;				
			//VDQ.q=VDQ.q+angle.get_wm()*Lsro*IDQ.d+(Ls-Lsro)*angle.get_wm()*angle.get_imr();

fImr<<FIXED_FLOAT(n*T)<<"sec.; imr (rotor magnetization current): "<<angle.get_imr()/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;	
		
		};
		n++;//TODO:2-?only is necessary in simulation,can be changed in embedded by flag 1-se n proximo do limite- n=5001; 
		//timer_Tr++;//to count the periods between Tr read
		V=InvPark((RotorFluxAngle),VDQ);
		T1T2<<"get v alpha beta "<<V.alpha<<" "<<V.beta<<"rfa"<<RotorFluxAngle<<endl;
		//abc_voltage=InvClarke(V);//TODO ??remove in real
		return V;//voltage to be applied
	};

