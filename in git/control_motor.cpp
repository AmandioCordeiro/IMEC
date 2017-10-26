
#include "control_motor.hpp"

#define T  0.0001
#define T_lag (0.0001*0.2)//TODO:ver tempo de computacao desde que lidos os valores de corrente até fim de ??decoupling?

/*mutable*/// std::mutex m_Mutex;

extern double Tr_calc_gl;
extern double m_Tr_calc_gl;
extern double dy_nt(double /*&*/y1, double /*&*/y_1);
extern double d2y_nt(double /*&*/y1, double /*&*/y,double /*&*/y_1);


extern double M;//0.050382//0.0117//0.069312//mutual inductance
extern double Ls;//0.051195//0.014//0.07132


extern double Lr;//0.052571//0.014//0.07132
extern double J;//TODO:??

//!!!!!!!!!!!!!!double dy_nt(double &y1, double &y_1);//y1 means y[n+1] 
	//return dynt=(y1-y_1)/(2*per);

//!!!!!!!-tirar isto final-double d2y_nt(double &y1, double &y,double &y_1);//derived can only be obtained  when n>=? 
extern double ro;//0.999863//1-*M/(Ls*Lr)//alterar nome
extern double B;//M/(ro*Ls*Lr)
extern double roLs;//0.013998//ro*Ls
extern double MB;//0.69851//B*M
extern double one_p_MB;//1.69851//1+MrB
double Lsro=(ro*Ls);//0.00081261
double Lrro=(ro*Lr);//0.0021891

bool fim_ciclo=false;

using namespace std;
using namespace Maths;

double motor::torque(double Vds,double Vqs){//torque generated
		//	cout<<"vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
		ids=ids+(Vds/roLs-alfa*ids+B/TR*fdr+np*wr*B*fqr+np*wr*iqs)*T;//ids=Lr/(Lr*Ls-M*M)*fds-M/(Ls*Lr-M*M)*fdr;
		iqs=iqs+(Vqs/roLs-alfa*iqs+B/TR*fqr-np*wr*B*fdr-np*wr*ids)*T;//idr=-M/(Ls*Lr-M*M)*fds+Ls/(Ls*Lr-M*M)*fdr;
		//iqs=Lr/(Lr*Ls-M*M)*fqs-M/(Ls*Lr-M*M)*fqr;//iqr=-M/(Ls*Lr-M*M)*fqs+Ls/(Ls*Lr-M*M)*fqr;//double torq_g=(-3/2*P/2*M/(Lr*Ls-M*M)*(fds*fqr-fqs*fdr));//fds=fds+(Vds+2*PI*fhz*fqs-Rs*ids)*T;cout<<"fds:"<<fds<<endl;
		//fqs=fqs+(Vqs-2*PI*fhz*fds-Rs*iqs)*T;cout<<"fqs:"<<fqs<<endl;
		fdr=fdr+(M/TR*ids-fdr/TR)*T;//fdr=fdr+((2*PI*fhz-P/2*wr)*fqr-Rr*idr)*T;cout<<"fdr:"<<fdr<<endl;
		fqr=fqr+(M*iqs/TR-fqr/TR)*T;//fqr=fqr+(-(2*PI*fhz-P/2*wr)*fdr-Rr*iqr)*T;cout<<"fqr:"<<fqr<<endl;
		torq_g=M*np/Lr*(iqs*fdr-ids*fqr);/*cout<<"ids"<<ids<<endl<<"iqs"<<iqs<<endl;*/fTorque<<n*T<<" sec. Torque_gerado:"<<torq_g/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl;//<<"torq_l:"<<torq_L<<endl;	
	return torq_g;
};
//!!!!!!!!!!!!!errado? pois conversao em relacao estator e em relacao angulo rotorico, to do in another version the introduction of pwm: qd->abc->pwm, pwm->dq
//!!(NAO) vou introduzir voltage compensation pois isso seria se nao houvesse correccao do Tr(isto em relacao ti),mas depois introduzi o q aparece no livro
//public:
long motor::get_n(){return n;};
double motor::get_torq_g(){return torq_g;};
double motor::get_m_Tr_calc_gl(){return m_Tr_calc_gl;};

void motor::set_theta_r(double t){theta_r=t;};

motor::motor(){/*Tr_calc=0.17703,*/TR=/*1.12252*/0.17703/*Lr/Rr*/;Rs=/*0.258*/0.10941/*0.435*/;alfa=(Rs/roLs+M*M/(roLs*Lr*TR));n=0;ids=0;iqs=0;idr=0;iqr=0;wr=0;/*Rr=3.9;*/fds=0;fqs=0;fdr=0;fqr=0;};
motor::~motor(){};
double motor::torque(double vas,double vbs,double vcs){
		return torque(ClarkePark(/*2*PI*fhz*/np*theta_r,tThreePhase(vas,vbs,vcs)).d,ClarkePark(/*2*PI*fhz*/np*theta_r,tThreePhase(vas,vbs,vcs)).q);
};
double motor::get_wr(double va,double vb,double vc/*double torq_g,double torq_L*/){
		//??if torq_g>torq_L
		wr=wr+((torque(va,vb,vc)-torq_L)*T/J);
		return wr;
	};
	double motor::get_ias(){
		return (InvClarkePark(np*theta_r/*2*PI*fhz*n()*T*/,tTwoPhaseDQ (ids,iqs)).a);
	};
	double motor::get_ibs(){
		return (InvClarkePark(np*theta_r/*2*PI*fhz*n()*T*/,tTwoPhaseDQ (ids,iqs)).b);
	};
	double motor::get_ics(){
		return (InvClarkePark(np*theta_r/*2*PI*fhz*n()*T*/,tTwoPhaseDQ (ids,iqs)).c);
	};
	void motor::set_torq_L(double torque){torq_L=torque;};
	void motor::set_Rs(double rr){Rs=rr;};
	double motor::get_torq_L(){return torq_L;};
	double motor::get_wr(){return wr;};
//tTwoPhaseDQ get_Vdq(){return tTwoPhaseDQ(Vds,Vqs);};
/*	tThreePhase get_abc_voltage(){//tudo:get or find abc_voltage;when torqL>torq_g
		return tThreePhase (bi2tri_as(Vds),bi2tri_bs(Vds,Vqs),bi2tri_cs(Vds,Vqs)) ;
};*/
	double motor::get_Rs(){return Rs;};

//class control_loops: public motor{
//public:
	double control_loops::get_rot_fl_an(){return rot_fl_an;};

	control_loops::control_loops():imrref(0.0),angle(0.0),abc_current(0.0,0.0,0.0),abc_voltage(0.0,0.0,0.0),IDQ(0.0001,0.0001),VDQ(0.0,0.0),rot_fl_an(0.0),Theta_r(0.0),w_ref(0.0),V(0.0,0.0),IDQ_rotor(0.0,0.0),VDQ_rotor(0.0,0.0),IDQ_d_1(0.0),IDQ_d_(0.0),IDQ_d1(0.0),IDQ_d_p(0.0),IDQ_d_pp(0.0),IDQ_q_1(0.0),IDQ_q_(0.0),IDQ_q1(0.0),IDQ_q_p(0.0),IDQ_q_pp(0.0),wmr_(0.0),wmr_1(0.0),wmr_p(0.0){
			Tr_calc_gl=0.17703;m_Tr_calc_gl=0.17703;//TODO insert at the end Tr motor valuet1=nullptr;fim_ciclo=false;
			vel.tune_pid(vel_p,vel_i,vel_d);/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*/
			torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);
			current_control_y.tune_pid(current_control_y_p,current_control_y_i,current_control_y_d);
			current_control_x.tune_pid(current_control_x_p,current_control_x_i,current_control_x_d);
			//flux_control.tune_pid(flux_control_p,flux_control_i,flux_control_d);
			
	};
	control_loops::~control_loops(){/*delete ConstTr;*/};
	void control_loops::set_w_ref(double wreff) {w_ref=wreff;};
	void control_loops::set_imrref(double imrreff){imrref=imrreff;};
	double control_loops::get_w_ref(){return w_ref;};

void control_loops::set_Theta_r(){
		Theta_r=Theta_r+this->get_wr()*T;
		while (Theta_r>=(2*PI)||Theta_r<0)//TODO 0 e 2*PI || -2*PI e 2PI {
		       {if(Theta_r>=(2*PI)){Theta_r=Theta_r-(2*PI);}
				else{Theta_r=Theta_r+(2*PI);};};
		motor::set_theta_r(Theta_r);//float Theta_r_np=Theta_r*np;while(Theta_r_np>=2*PI){Theta_r_np=Theta_r_np-2*PI;};
		//cout<<"Theta_r*np"<<(Theta_r_np)<<endl;//adapt after 2PI; electrical?
}

double control_loops::IDQ_rotor_d(){/*double temp=*/return IDQ_rotor.d;/*return temp;*/}
double control_loops::IDQ_rotor_q(){/*double temp=*/return IDQ_rotor.q;/*return temp;*/}
double control_loops::VDQ_rotor_d(){/*double temp=*/return VDQ_rotor.d;/*return temp;*/}
double control_loops::VDQ_rotor_q(){/*double temp=*/return VDQ_rotor.q;/*return temp;*/}

	//get_VDQ +++++++++++++++++++++++++++
tTwoPhase control_loops::get_V(/*const*/ ){
			float torque_control_Min_pid_res=-150;//?
			float torque_control_Max_pid_res=150;
			if(n==0){
//PIDs----------------------
						
			 vel.init_pid(this->get_wr(),w_ref,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
				vel.calc_pid();

			 torque_control.init_pid(IDQ.q*3/2*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
				torque_control.calc_pid();

			current_control_y.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
				current_control_y.calc_pid();
			
			//flux_control.init_pid(angle.get_imr(),imrref,flux_control_Min_pid_res,flux_control_Max_pid_res ,flux_control_cel);
			//	flux_control.calc_pid();
			current_control_x.init_pid(IDQ.d,/*flux_control.get_pid_result()*/20,current_control_x_Min_pid_res,current_control_x_Max_pid_res ,current_control_x_cel);
				current_control_x.calc_pid();
		//cout<<"imrref_"<<imrref<<endl;
					VDQ.q=current_control_y.get_pid_result();//cout<<"VDQ.q:"<<VDQ.q<<endl;//se necessario reduzir codigo podesse eliminar vdq?

// isd.init_pid(IDQ.d,(90/*-5*w_ref*w_ref*w_ref+56*w_ref*w_ref-209*w_ref+300*/)/*TODO:(30%nominal)*/,current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
//	current_control_y.calc_pid();
					VDQ.d=current_control_x.get_pid_result();//cout<<"VDQ.d:"<<VDQ.d<<endl;
				//reset?integral

//+Vdecouple-------------todo:srea necessario com a Tr corregida			
			IDQ_d1=IDQ.d;
			
			IDQ_q1=IDQ.q;
			
			wmr_1=angle.get_wm();
			
			}
		else{
//----------------------------------------
			abc_current.a=this->get_ias();/*cout<<"ia"<<abc_current.a<<endl;*/abc_current.b=this->get_ibs();abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor, and is better get abc_current.c from measure
			IDQ=ClarkePark(rot_fl_an,abc_current);//TODO???rfa==0?

			IDQ_rotor=(ClarkePark((np*Theta_r),abc_current));
			VDQ_rotor=(ClarkePark((np*Theta_r),abc_voltage));//TODO: voltage,done?, sensor?				
			//signal_Tr;//start_thread_Tr();//TODO??
			
			
////---------------------------	
			float IDQq=IDQ.q;float IDQd=IDQ.d;										fIDQ<<n*T<<" sec. IDQ_q: "/*<<IDQd<<" "*/<<IDQq/*<<" IDQd "<<"IDQq"*/<<endl/*<<"motor::TR"<<motor::TR<<endl<<"motor::wr"<<motor::wr*/;
			rot_fl_an=angle.RotFluxAng(IDQq,IDQd,T,m_Tr_calc_gl/*motor::TR*/,(this->get_wr()*np));			/*cout<<"rfa:"<<rot_fl_an<<endl;*///TODO nofinal retirar T chamando-o d define
			rot_fl_an=rot_fl_an*0.97;//TODO: remover?
//PIDs-------------------------------------------
			float Rd_we=Rs+get_wr()*np*get_wr()*np*M*M/Rm;
			float Rq_we=Rs+/*Rrr */1/m_Tr_calc_gl*Lr*M*M/Lr/Lr+get_wr()*np*get_wr()*np*M*M*Llr*Llr/Rm/Lr/Lr;//TODO ?substituir online Rs pelo calculado na constante de tempo d rotor 
			float IDQ_d_lma=0;
			
			
			
			vel.set_process_point(this->get_wr());			
			vel.set_setpoint(w_ref);
			vel.calc_pid();

			torque_control.set_process_point(IDQ.q*Kt/*3/2*np*M*M/Lr*/*angle.get_imr());
			torque_control.set_setpoint(vel.get_pid_result());   								// cout<<"wel.get_pid_result:"<<vel.get_pid_result()<<endl;
			if (get_wr()*np<Wn){
				torque_control_Min_pid_res=-Tm1;//?
				torque_control_Max_pid_res=Tm1;
				if(IDQ.q<=pow(Rd_we/Rq_we,1/2)*Idn)
					{IDQ_d_lma=pow(Rq_we/Rd_we,1/2)*abs(IDQ.q);}
				else{IDQ_d_lma=Idn;}
				}
			else if (get_wr()*np<Wc){
					float Tm2=Kt*pow((pow(Vmax/(get_wr()*np*Ls),2)-Imax*Imax*ro*ro),1/2)*pow(Imax*Imax-pow(Vmax/(get_wr()*np*Ls),2),1/2)/(1-ro*ro);
					torque_control_Min_pid_res=-Tm2;//?
					torque_control_Max_pid_res=Tm2;
					if(IDQ.q<=Vmax/(get_wr()*np*Ls*ro)/pow(pow(ro,-2)+Rd_we/Rq_we,1/2)*pow(Rd_we/Rq_we,1/2))
						{IDQ_d_lma=pow(Rq_we/Rd_we,1/2)*abs(IDQ.q);}
					else {IDQ_d_lma=pow(Imax*Imax/2-pow((Imax*Imax*Imax*Imax-4*vel.get_pid_result()*vel.get_pid_result()/Kt/Kt),1/2)/2,1/2);}
					}
			else {
				float Tm3=Kt*pow(Vmax/get_wr()*np*Ls,2)/(2*ro);
				torque_control_Min_pid_res=-Tm3;//?
				torque_control_Max_pid_res=Tm3;
				if(IDQ.q<=Vmax/(get_wr()*np*Ls*ro)/pow(pow(ro,-2)+Rd_we/Rq_we,1/2)*pow(Rd_we/Rq_we,1/2))//Tp3=Tp2,?usa-se iqs<= ou Tp 
						{IDQ_d_lma=pow(Rq_we/Rd_we,1/2)*abs(IDQ.q);}
					else {IDQ_d_lma=pow((Vmax*Vmax+pow(pow(Vmax,4)-4*pow(get_wr()*np,4)*ro*ro*Ls*Ls*Ls*Ls*vel.get_pid_result()*vel.get_pid_result()/Kt/Kt,1/2))/(2*pow(get_wr()*np*Ls,2)),1/2);//TODO
					}
			}
			torque_control.calc_pid();

			current_control_y.set_process_point(IDQ.q);
			current_control_y.set_setpoint(torque_control.get_pid_result());
			current_control_y.calc_pid();
			
			////flux_control.set_process_point(angle.get_imr());
			////flux_control.set_setpoint(imrref);//TODO
			////flux_control.calc_pid();
			IDQ_d__lma<<n*T<<"sec. IDQ_D_lma:"<<IDQ_d_lma<<endl;
			current_control_x.set_process_point(IDQ.d);
			current_control_x.set_setpoint(IDQ_d_lma/*flux_control.get_pid_result()*/);
			current_control_x.calc_pid();
		
			VDQ.d=current_control_x.get_pid_result();											   cout<<"VDQ.d:"<<VDQ.d<<endl;
			VDQ.q=current_control_y.get_pid_result(); 											//  cout<<"VDQ.q:"<<VDQ.q<<endl;

//+Vdecouple-------------	
			IDQ_d_1=IDQ_d_;
			IDQ_d_=IDQ_d1;
			IDQ_d1=IDQ.d;
			
			IDQ_d_p=dy_nt(	IDQ_d1, IDQ_d_);
			IDQ_d_pp=d2y_nt(IDQ_d1,IDQ_d_ ,IDQ_d_1);
			
			IDQ_q_1=IDQ_q_;
			IDQ_q_=IDQ_q1;
			IDQ_q1=IDQ.q;
			
			IDQ_q_p=dy_nt(	IDQ_q1, IDQ_q_);
			IDQ_q_pp=d2y_nt(IDQ_q1,IDQ_q_ ,IDQ_q_1);
			
			wmr_=wmr_1;
			wmr_1=angle.get_wm();
			
			wmr_p=dy_nt(wmr_1, wmr_);
			
			VDQ.d=VDQ.d/*-wmr_1*Lsro*IDQ.q*/+T_lag*Rs*IDQ_d_p+T_lag*Lsro*IDQ_d_pp-wmr_1*(Lsro-T_lag*Rs)*IDQ_q1+wmr_1*wmr_1*T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr())/*TODO:modulo imr*/-T_lag*Lsro*IDQ_q1*wmr_p;				
			VDQ.q=VDQ.q+T_lag*Rs*IDQ_q_p+T_lag*Lsro*IDQ_q_pp+wmr_1*(Lsro-T_lag*Rs)*IDQ_d1+wmr_1*wmr_1*T_lag*Lsro*IDQ_q1+(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))*wmr_p+(Ls-Lsro)*wmr_1*abs(angle.get_imr());//angle.wmr_1*Lsro*IDQ.d+(Ls-Lsro)*wmr_1*angle.get_imr();
			
			//VDQ.d=VDQ.d-angle.get_wm()*Lsro*IDQ.q;cout<<"VDQD+"<<VDQ.d<<endl;//Rs*IDQ.d-np*2/3*motor::wr*(Lsro+Lrro)*IDQ.q;				
			//VDQ.q=VDQ.q+angle.get_wm()*Lsro*IDQ.d+(Ls-Lsro)*angle.get_wm()*angle.get_imr();
//cout <<"VDQ.q+com"<<VDQ.q<<endl;
fImr<<n*T<<"sec. imr:"<<angle.get_imr()/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;	
		//TODO:ver codigo de test PID em q referem-se ao erro e pid_set_integral
		};
		//cout<<"n:"<<n<<endl;
		++n;//TODO: se n proximo do limite- n=5001; 
		V=InvPark((rot_fl_an),VDQ);
		abc_voltage=InvClarke(V);//TODO remove in real
		return V;
	};



