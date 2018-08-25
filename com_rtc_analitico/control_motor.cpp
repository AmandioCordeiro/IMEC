
#include "control_motor.hpp"

tTwoPhase v_(0.0,0.0);
double velocidade=0;

extern double VDC, Vmax;
double T = 0.0001;//0.0001
#define T_lag (0.000000001)//0.0001*0.02 TODO:?ver tempo de computacao desde que lidos os valores até disparos dos igbts
double vel_p=20.0;//6;//1000;//1;//1//9//1.1//(3)//6.0*0.9//(2/*4.5*/)//TODO// 13marco
double vel_i=0.0006;//0.00006;
double torque_control_p=3.6;
double torque_control_i=0.008;

/*mutable*/// std::mutex m_Mutex;
double IDC=0.0;

double vaa,vbb,vcc;
double VDQ_alpha_esc,VDQ_beta_esc;

double dy_nt_(double /*&*/y1, double /*&*/y_1){//y1 significa y[n+1] 
	return ((y1-y_1)/(2*T));};

double d2y_nt_(double /*&*/y1, double /*&*/y,double /*&*/y_1){//derived can only be obtained  when n>=? 
	return ((y1-2*y+y_1)/(T*T));};

extern long timer_Tr;

//extern double dy_nt(double /*&*/y1, double /*&*/y_1);
//extern double d2y_nt(double /*&*/y1, double /*&*/y,double /*&*/y_1);


//extern double M;//0.050382//0.0117//0.069312//mutual inductance
extern double Ls;//0.051195//0.014//0.07132

extern double Lr;//0.052571//0.014//0.07132
extern double J;//TODO:??

//!!!!!!!!!!!!!!double dy_nt(double &y1, double &y_1);//y1 means y[n+1] 
	//return dynt=(y1-y_1)/(2*per);

//!!!!!!!-tirar isto final-double d2y_nt(double &y1, double &y,double &y_1);//derived can only be obtained  when n>=? 
extern double ro;//0.999863//1-M*M/(Ls*Lr)//alterar nome
extern double B;//M/(ro*Ls*Lr)
extern double roLs;//0.013998//ro*Ls
extern double MB;//0.69851//B*M
extern double one_p_MB;//1.69851//1+MrB
double Lsro=(ro*Ls);//0.00081261
double Lrro=(ro*Lr);//0.0021891

//bool fim_ciclo=false;

using namespace std;
using namespace Maths;


double motor::torque_z(double Vds,double Vqs,double angle_get_wm){//doesnt work, torque generated frame synchronous rotor
			cout<<"vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
	double Lls=Ls-M;
		if (angle_get_wm==0.0)angle_get_wm=0.00000000001;
		double	s_2=(angle_get_wm-wr*np)/angle_get_wm*(angle_get_wm-wr*np)/angle_get_wm;
		double Rfe=325;//Rm/(s_2+1);		
		
		double fds_k1=(Vds-Rs/Lls*fds+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		double fqs_k1=(Vqs-Rs/Lls*fqs-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		double fdr_k1=-Rr/Llr*fdr+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm;
		double fqr_k1=-Rr/Llr*fqr-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm;
		double Idm_k1=Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Idm+(angle_get_wm-np*wr)*Iqm;//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k1=Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Iqm-(angle_get_wm-np*wr)*Idm;//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
		//double fdr_k1=(-fdr+M*Idm)*Rr/Llr;
		
		double fds_k2=(Vds-Rs/Lls*fds_k1/2*T+/*np*wr*/angle_get_wm*fqs_k1/2*T+Rs*M/Lls*Idm_k1/2*T);
		double fqs_k2=(Vqs-Rs/Lls*fqs_k1/2*T-/*np*wr*/angle_get_wm*fds_k1/2*T+Rs*M/Lls*Iqm_k1/2*T);
		double fdr_k2=-Rr/Llr*fdr_k1/2*T+(angle_get_wm-np*wr)*fqr_k1/2*T+M*Rr/Llr*Idm_k1/2*T;
		double fqr_k2=-Rr/Llr*fqr_k1/2*T-(angle_get_wm-np*wr)*fdr_k1/2*T+M*Rr/Llr*Iqm_k1/2*T;
		double Idm_k2=Rfe/(Lls*M)*fds_k1/2*T+Rfe/(M*Llr)*fdr_k1/2*T+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Idm_k1/2*T+(angle_get_wm-np*wr)*Iqm_k1/2*T;//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k2=Rfe/(Lls*M)*fqs_k1/2*T+Rfe/(M*Llr)*fqr_k1/2*T+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Iqm_k1/2*T-(angle_get_wm-np*wr)*Idm_k1/2*T;//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
		
		double fds_k3=(Vds-Rs/Lls*fds_k2/2*T+/*np*wr*/angle_get_wm*fqs_k2/2*T+Rs*M/Lls*Idm_k2/2*T);
		double fqs_k3=(Vqs-Rs/Lls*fqs_k2/2*T-/*np*wr*/angle_get_wm*fds_k2/2*T+Rs*M/Lls*Iqm_k2/2*T);
		double fdr_k3=-Rr/Llr*fdr_k2/2*T+(angle_get_wm-np*wr)*fqr_k2/2*T+M*Rr/Llr*Idm_k2/2*T;
		double fqr_k3=-Rr/Llr*fqr_k2/2*T-(angle_get_wm-np*wr)*fdr_k2/2*T+M*Rr/Llr*Iqm_k2/2*T;
		double Idm_k3=Rfe/(Lls*M)*fds_k2/2*T+Rfe/(M*Llr)*fdr_k2/2*T+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Idm_k2/2*T+(angle_get_wm-np*wr)*Iqm_k2/2*T;//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k3=Rfe/(Lls*M)*fqs_k2/2*T+Rfe/(M*Llr)*fqr_k2/2*T+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Iqm_k2/2*T-(angle_get_wm-np*wr)*Idm_k2/2*T;//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
		
		double fds_k4=(Vds-Rs/Lls*fds_k3*T+/*np*wr*/angle_get_wm*fqs_k3*T+Rs*M/Lls*Idm_k3*T);
		double fqs_k4=(Vqs-Rs/Lls*fqs_k3*T-/*np*wr*/angle_get_wm*fds_k3*T+Rs*M/Lls*Iqm_k3*T);
		double fdr_k4=-Rr/Llr*fdr_k3*T+(angle_get_wm-np*wr)*fqr_k3*T+M*Rr/Llr*Idm_k3*T;
		double fqr_k4=-Rr/Llr*fqr_k3*T-(angle_get_wm-np*wr)*fdr_k3*T+M*Rr/Llr*Iqm_k3*T;
		double Idm_k4=Rfe/(Lls*M)*fds_k3*T+Rfe/(M*Llr)*fdr_k3*T+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Idm_k3*T+(angle_get_wm-np*wr)*Iqm_k3*T;//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_k4=Rfe/(Lls*M)*fqs_k3*T+Rfe/(M*Llr)*fqr_k3*T+(-Rfe/Lls-Rfe/Llr-Rfe/M)*Iqm_k3*T-(angle_get_wm-np*wr)*Idm_k3*T;//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;
		
		double fds_1=fds+T/6*(fds_k1+2*fds_k2+2*fds_k3+fds_k4);
		double fqs_1=fqs+T/6*(fqs_k1+2*fqs_k2+2*fqs_k3+fqs_k4);
		double fdr_1=fdr+T/6*(fdr_k1+2*fdr_k2+2*fdr_k3+fdr_k4);
		double fqr_1=fqr+T/6*(fqr_k1+2*fqr_k2+2*fqr_k3+fqr_k4);
		double Idm_1=Idm+T/6*(Idm_k1+2*Idm_k2+2*Idm_k3+Idm_k4);
		double Iqm_1=Iqm+T/6*(Iqm_k1+2*Iqm_k2+2*Iqm_k3+Iqm_k4);
		
//		double iiqf=iiqs+iiqr-Iqm;
		iqf=iqs+iqr-Iqm;
//		double iidf=iids+iidr-Idm;
		idf=ids+idr-Idm;		
	
		//Idm=Idm+(Rfe*idf+angle_get_wm*M*Iqm)*T/M;
		//Iqm=Iqm+(Rfe*iqf-angle_get_wm*M*Idm)*T/M;
		
		fds=fds_1;
		fqs=fqs_1;
		Idm=Idm_1;
		Iqm=Iqm_1;
		fdr=fdr_1;
		fqr=fqr_1;
		
		iqs=(fqs-M*Iqm)/Lls;
		ids=(fds-M*Idm)/Lls;
				
		idr=(fdr-M*Idm)/Llr;
		iqr=-M*Iqm/Llr;
		
		torq_g=Kt*ids*iqs;
		//double ttorq_g=3.0/2.0*M*np/Lr*(iqs*fdr-ids*fqr);
		double torq_ggg=3.0/2.0*M*np/Lr*(fdr*(Iqm-(fqr-M*Iqm)/Llr)-fqr*(Idm-(fdr-M*Idm)/Llr));//double torq_gggg=M*np/Lr*(fdr*(iiqs-iiqf)-fqr*(iids-iidf));
		double torq_gggg=3.0/2.0*M*np/Lr*(fdr*(fqs-M*Iqm)/Lls-fqr*(fds-M*Idm)/Lls);
		double torq_ggggg=3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds));
		//double torq_gggg=M/Llr*(Llr*idr+M*Idm)*Iqm;
		//double torq_gggg=3.0/2.0*M*np/Lr*(fdr*Iqm-fqr*Idm);
		fTorque<<n*T<<" sec. Torque_gerado:"<<torq_ggggg<<" g :"<<torq_g<<" ids "<<ids <<" iqs "<<iqs<<"   ggggg  "<< torq_gggg/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl;
	return /*M/Llr*fdr*Iqm*/torq_ggggg;
};

double motor::torque_g(double Vds,double Vqs,double angle_get_wm){//torque generated, frame generic . used backward euler method
		//	cout<<"vds:"<<Vds<<endl<<"Vqs:"<<Vqs<<endl;
	double Lls=Ls-M;
		if (angle_get_wm==0.0)angle_get_wm=0.00000000001;
		double	s_2=(angle_get_wm-wr*np)/angle_get_wm*(angle_get_wm-wr*np)/angle_get_wm;
		double Rfe=Rm/(s_2+1);		
		
		double fds_=T*Lls/(Lls+Rs*T)*(Vds+fds/T+/*np*wr*/angle_get_wm*fqs+Rs*M/Lls*Idm);
		double fqs_=T*Lls/(Lls+Rs*T)*(Vqs+fqs/T-/*np*wr*/angle_get_wm*fds+Rs*M/Lls*Iqm);
		double fdr_=T*Llr/(Llr+Rr*T)*(fdr/T+(angle_get_wm-np*wr)*fqr+M*Rr/Llr*Idm);
		double fqr_=T*Llr/(Llr+Rr*T)*(fqr/T-(angle_get_wm-np*wr)*fdr+M*Rr/Llr*Iqm);
		double Idm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fds+Rfe/(M*Llr)*fdr+Idm/T+(angle_get_wm-np*wr)*Iqm);//(-Lr/Llr*Idm+ids+fdr/Llr+angle_get_wm/*np*wr*/*M/Rfe*Iqm)/M*Rfe;
		double Iqm_=1/(1/T+Rfe/Lls+Rfe/Llr+Rfe/M)*(Rfe/(Lls*M)*fqs+Rfe/(M*Llr)*fqr+Iqm/T-(angle_get_wm-np*wr)*Idm);//(-Lr/Llr*Iqm+iqs-angle_get_wm/*np*wr*/*M/Rfe*Idm)/M*Rfe;

		fds=fds_;
		fqs=fqs_;
		fdr=fdr_;
		fqr=fqr_;
		Idm=Idm_;
		Iqm=Iqm_;
		ids=(fds-M*Idm)/Lls;
		iqs=(fqs-M*Iqm)/Lls;
		//ids=ids+(Vds/roLs-alfa*ids+B/TR*fdr+np*wr*B*fqr+np*wr*iqs)*T;
		
		//iqs=iqs+(Vqs/roLs-alfa*iqs+B/TR*fqr-np*wr*B*fdr-np*wr*ids)*T;
		
		
//		fds=fds+(Vds+angle_get_wm*fqs-Rs*ids)*T;cout<<"fds:"<<fds<<endl;
//		fqs=fqs+(Vqs-angle_get_wm*fds-Rs*iqs)*T;cout<<"fqs:"<<fqs<<endl;
//		//fdr=fdr+(M/TR*ids-fdr/TR)*T;
//		fdr=fdr+((angle_get_wm-np*wr)*fqr-Rr*idr)*T;cout<<"fdr:"<<fdr<<endl;
//		//fqr=fqr+(M*iqs/TR-fqr/TR)*T;
//		fqr=fqr+(-(angle_get_wm-np*wr)*fdr-Rr*iqr)*T;cout<<"fqr:"<<fqr<<endl;
//		
//		//idr=-M/(Ls*Lr-M*M)*fds+Ls/(Ls*Lr-M*M)*fdr;
//		idr=fds/(M-Lr*Ls/M)-Ls*fdr/(M*M-Lr*Ls);
//		//iqr=-M/(Ls*Lr-M*M)*fqs+Ls/(Ls*Lr-M*M)*fqr;//double torq_g=(-3/2*P/2*M/(Lr*Ls-M*M)*(fds*fqr-fqs*fdr));
//		iqr=fqs/(M-Lr*Ls/M)-Ls*fqr/(M*M-Lr*Ls);
//		
//		//ids=Lr/(Lr*Ls-M*M)*fds-M/(Ls*Lr-M*M)*fdr;
//		ids=(fdr-Lr*idr)/M;
//		//iqs=Lr/(Lr*Ls-M*M)*fqs-M/(Ls*Lr-M*M)*fqr;
//		iqs=(fqr-Lr*iqr)/M;
//		
//		//torq_g=M*np/Lr*(iqs*fdr-ids*fqr);
//		torq_g=3.0/*2.0*/*np*(iqs*fds-ids*fqs);//TODO sobre 2?? acho que n
//		/*cout<<"ids"<<ids<<endl<<"iqs"<<iqs<<endl;*/fTorque<<n*T<<" sec. Torque_gerado:"<<torq_g/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl;//<<"torq_l:"<<torq_L<<endl;	
	double t=(3.0/2.0*M*np/Lr/roLs*(fdr*(fqs)-fqr*(fds)));
	cout<<"torq: "<<t<<endl<<"fqr: "<<fqr<<endl;
	fTorque<<n*T<<" sec. Torque_gerado:"<<t/*<<"torq_i:"<<(3/2*P/2*(fds*iqs-fqs*ids))*/<<endl<<"potencia (Torq*wr): "<<(t/*np*/*wr)<<" corrente(Vdc*Idc) "<<(IDC*VDC)<<endl<<"We: "<<angle_get_wm<<endl<<"(angle_get_wm-wr*np)"<<(angle_get_wm-wr*np)<<endl;//<<"torq_l:"<<torq_L<<endl;	
	return t;
};

//!!!!!!!!!!!!!errado? pois conversao em relacao estator e em relacao angulo rotorico, to do in another version the introduction of pwm: qd->abc->pwm, pwm->dq
//!!(NAO) vou introduzir voltage compensation pois isso seria se nao houvesse correccao do Tr(isto em relacao ti),mas depois introduzi o q aparece no livro
//public:
long motor::get_n(){return n;};
double motor::get_torq_g(){return torq_g;};
double motor::get_m_Tr_calc_gl(){return m_Tr_calc_gl;};

void motor::set_theta_r(double t){theta_r=t;};

motor::motor():idf(0.0),iqf(0.0),RotorFluxAngle(0.0),Idm(0.0),Iqm(0.0),d_Idm(0.0),d_Iqm(0.0),angle(0.0),abc_current(0.0,0.0,0.0){/*Tr_calc=0.17703,*/TR=0.252308/*0.17703*//*1.12252 Lr/Rr*/;Rs=0.012/*0.10941*//*0.258*//*0.435*/;alfa=(Rs/roLs+M*M/(roLs*Lr*TR));n=0;ids=0.0;iqs=0.0;idr=0.0;iqr=0.0;wr=0.00000001;Rr=0.0065/*3.9*/;fds=0.0;fqs=0.0;fdr=0.0;fqr=0.0;torq_L=0;theta_r=0.0;torq_g=0.00000001;};
motor::~motor(){};
double motor::torque(double vas,double vbs,double vcs){
		return torque_g(ClarkePark(RotorFluxAngle/*np*theta_r*/,tThreePhase(vas,vbs,vcs)).d/*alpha*/,ClarkePark(RotorFluxAngle/*np*theta_r*/,tThreePhase(vas,vbs,vcs)).q/*beta*/,angle.get_wm());
};
double motor::get_wr(double va,double vb,double vc/*double torq_g,double torq_L*/){
		//??if torq_g>torq_L
		wr=wr+((torque(va,vb,vc)-torq_L)*T/J);
		return wr;
	};
	double motor::get_ias(){//TODO in real read from sensors
		abc_current.a=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).a);
		return abc_current.a;
	};
	double motor::get_ibs(){//TODO in real read from sensors
		abc_current.b=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).b);
		return abc_current.b;
	};
	double motor::get_ics(){//TODO in real read from sensors
		//abc_current.c=-abc_current.a-abc_current.b; but is better get abc_current.c from measure
		abc_current.c=(InvClarkePark(RotorFluxAngle/*np*theta_r*/,tTwoPhaseDQ(ids,iqs)).c);
		return abc_current.c;
	};
	void motor::set_torq_L(double torque){torq_L=torque;};
	void motor::set_Rs(double rr){Rs=rr;};
	double motor::get_torq_L(){return torq_L;};
	double motor::get_wr(){return wr;};//TODO implement in REAL
//tTwoPhaseDQ get_Vdq(){return tTwoPhaseDQ(Vds,Vqs);};
/*	tThreePhase get_abc_voltage(){//tudo:get or find abc_voltage;when torqL>torq_g
		return tThreePhase (bi2tri_as(Vds),bi2tri_bs(Vds,Vqs),bi2tri_cs(Vds,Vqs)) ;
};*/
	double motor::get_Rs(){return Rs;};

//class control_loops: public motor{
//public:
	double control_loops::get_rot_fl_an(){return RotorFluxAngle;};

	control_loops::control_loops():/*v_(0.0,0.0),*/IDQq_p(0.0),stc_p(0.0), abc_voltage_svpwm(0.0,0.0,0.0),VDQ_rfa(0.0,0.0),sobre_Tr_comp(0.0),wmr__1(0.0),imrref(0.0),abc_voltage(0.0,0.0,0.0),IDQ(0.0001,0.0001),VDQ(0.0,0.0),/*rot_fl_an(0.0),*/Theta_r(0.0),w_ref(0.0),V(0.0,0.0),IDQ_rotor(0.0,0.0),VDQ_rotor(0.0,0.0),IDQ_d_1(0.0),IDQ_d_(0.0),IDQ_d1(0.0),IDQ_d_p(0.0),IDQ_d_pp(0.0),IDQ_q_1(0.0),IDQ_q_(0.0),IDQ_q1(0.0),IDQ_q_p(0.0),IDQ_q_pp(0.0),wmr_(0.0),wmr1(0.0),wmr_p(0.0){
			Tr_calc_gl=Trrr/*0.17703*/;m_Tr_calc_gl=Trrr/*0.17703*/;//TODO insert at the end Tr motor value
			//torque_control_Min_pid_res=-150;//?
			//torque_control_Max_pid_res=150;
			vel_Min_pid_res = -1000.0;//600 //-300.0//-170.0				//Min value PI out
			vel_Max_pid_res = 1000.0;
			current_control_y_Min_pid_res=-400.0;//(-(2/3)*300);//100?
			current_control_y_Max_pid_res=400.0;//((2/3)*300);// 400//100?
			current_control_x_Min_pid_res=-300.0;//(-(2/3)*300);// (-400)//100?
			current_control_x_Max_pid_res=300.0;//((2/3)*300);//400//100?
			vel.tune_pid(vel_p,vel_i,vel_d);/*vector <Rs_Tr> ConstTr(100,Rs_Tr());*/
			torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);
			current_control_y.tune_pid(current_control_y_p,current_control_y_i,current_control_y_d);
			current_control_x.tune_pid(current_control_x_p,current_control_x_i,current_control_x_d);
			//flux_control.tune_pid(flux_control_p,flux_control_i,flux_control_d);
			for (int i=0;i<9;i++)vec_MediaTr_[i]= m_Tr_calc_gl;
	};
	
	void control_loops::vel_tune_pid(){vel.tune_pid(vel_p,vel_i,vel_d);};
	void control_loops::torque_control_tune_pid(){torque_control.tune_pid(torque_control_p,torque_control_i,torque_control_d);};
	control_loops::~control_loops(){/*delete ConstTr;*/};
	void control_loops::set_w_ref(double wreff) {w_ref=wreff;};
	void control_loops::set_imrref(double imrreff){imrref=imrreff;};
	double control_loops::get_w_ref(){return w_ref;};

void control_loops::set_Theta_r(){//set variable theta_r
		Theta_r=Theta_r+this->get_wr()*T;
		while (Theta_r>=(2*PI)||Theta_r<0)//TODO 0 e 2*PI || -2*PI e 2PI {
		       {if(Theta_r>=(2*PI)){Theta_r=Theta_r-(2*PI);}
				else{Theta_r=Theta_r+(2*PI);};};
		motor::set_theta_r(Theta_r);
		
};
void control_loops::get_VDC(float vdc){VDC=vdc;Vmax=VDC/sqrt(3);};//TODO adapt in real

double control_loops::IDQ_rotor_d(){return IDQ_rotor.d;};
double control_loops::IDQ_rotor_q(){return IDQ_rotor.q;};
double control_loops::VDQ_rotor_d(){return VDQ_rotor.d;};
double control_loops::VDQ_rotor_q(){return VDQ_rotor.q;};

	//get_VDQ +++++++++++++++++++++++++++
tTwoPhase control_loops::get_V(/*const*/ ){
			//float torque_control_Min_pid_res=-150;//?
			//float torque_control_Max_pid_res=150;
			current_control_y_Min_pid_res=-400.0;//(-2.0)/3*VDC;
			current_control_y_Max_pid_res=400.0;//(2.0/3*VDC);
			current_control_y.act_min_max(current_control_y_Min_pid_res,current_control_y_Max_pid_res);
			current_control_x_Min_pid_res=-400.0;//((-2.0)/3*VDC);
			current_control_x_Max_pid_res=400.0;//(2.0/3*VDC);
			current_control_x.act_min_max(current_control_x_Min_pid_res,current_control_x_Max_pid_res);
			//TODO if (n>=("maximum of long")){n=0;}//n nao é necessario na realidade
			if(n==0){
//PIDs----------------------
				//TODO remove fImr<<"Wn: "<<Wn<<"Wc: "<<Wc<<" Tm1: "<<Tm1<<endl;		
			 vel.init_pid(this->get_wr(),w_ref,vel_Min_pid_res,vel_Max_pid_res ,vel_cel);//insirir val em tune...
				vel.calc_pid();

			 torque_control.init_pid(IDQ.q*3/2*np*M*M/Lr*angle.get_imr()/*IDQ.q*/,vel.get_pid_result(),torque_control_Min_pid_res,torque_control_Max_pid_res ,torque_control_cel);
				torque_control.calc_pid();

			current_control_y.init_pid(IDQ.q,torque_control.get_pid_result(),current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
				current_control_y.calc_pid();
			
			//flux_control.init_pid(angle.get_imr(),imrref,flux_control_Min_pid_res,flux_control_Max_pid_res ,flux_control_cel);
			//	flux_control.calc_pid();
			current_control_x.init_pid(IDQ.d,/*flux_control.get_pid_result(),TODO*/100,current_control_x_Min_pid_res,current_control_x_Max_pid_res ,current_control_x_cel);
				current_control_x.calc_pid();
		//cout<<"imrref_"<<imrref<<endl;
			
					VDQ.q=torque_control.get_pid_result();//current_control_y.get_pid_result();//cout<<"VDQ.q:"<<VDQ.q<<endl;//se necessario reduzir codigo podesse eliminar vdq?

// isd.init_pid(IDQ.d,(90/*-5*w_ref*w_ref*w_ref+56*w_ref*w_ref-209*w_ref+300*/)/*TODO:(30%nominal)*/,current_control_y_Min_pid_res,current_control_y_Max_pid_res ,current_control_y_cel);
//	current_control_y.calc_pid();
					VDQ.d=current_control_x.get_pid_result();//cout<<"VDQ.d:"<<VDQ.d<<endl;
				//reset?integral

//+Vdecouple-------------TODO:srea necessario com a Tr corregida			
			IDQ_d1=IDQ.d;
			
			IDQ_q1=IDQ.q;
			
			wmr1=angle.get_wm();
			//TODO colocar n++ aqui?? no final
			}
		else{
//----------------------------------------
			
			IDQ=ClarkePark(RotorFluxAngle,abc_current);//TODO???rfa==0? ??abc_current ta bem?

			IDQ_rotor=(ClarkePark((np*Theta_r),abc_current));
			
			abc_voltage_svpwm.a=vaa;abc_voltage_svpwm.b=vbb;abc_voltage_svpwm.c=vcc;
			VDQ_rfa=ClarkePark(RotorFluxAngle,abc_voltage_svpwm);
			VDQ_rotor=(ClarkePark((np*Theta_r),abc_voltage_svpwm));//TODO:ERROR because in real abc_voltage is scaled by svpwm. voltage,done?, sensor?				
			
////---------------------------	
			float IDQq=IDQ.q;float IDQd=IDQ.d;	
			
			int sinal_=1;
			double incr_decr=IDQq_p-IDQ.q;
			IDQq_p=IDQ.q;
			
			fIDQ<<n*T<<" sec. IDQ_q: "/*<<IDQd<<" "*/<<IDQq/*<<" IDQd "<<"IDQq"*/<<endl/*<<"motor::TR"<<motor::TR<<endl<<"motor::wr"<<motor::wr*/;
			RotorFluxAngle=angle.RotFluxAng_(IDQq,IDQd,T,m_Tr_calc_gl,(this->get_wr()*np));			
			//if (n==50000)RotorFluxAngle=0.0;
			RotorFluxAngle=RotorFluxAngle*0.9;//TODO:+PI/2 remove? in simulation doesn't work but in real i think it's needed??
			
//PIDs-------------------------------------------
			double Wm = angle.get_wm();//Wm == we, utilizado nos papers
			double Rd_we=Rs+Wm*Wm*M*M/Rm;
			double Rq_we=Rs+/*Rrr */1.0/(m_Tr_calc_gl*Lr)*M*M/*/Lr/Lr*/+Wm/*np*/*Wm/*np*/*M*M*Llr*Llr/Rm/Lr/Lr;//TODO ?substituir online Rs pelo calculado na constante de tempo d rotor 
			float IDQ_d_lma=0.0;
			
			vel.set_process_point(this->get_wr());	//fIDQ<<" get_wr(): "<<this->get_wr()<<" w_ref: "<<w_ref<<endl;		
			vel.set_setpoint(w_ref);
			//vel.calc_pid();

			torque_control.set_process_point(IDQ.q*Kt/*3/2*np*M*M/Lr*/*angle.get_imr());//TODO isto ta bem?
			fTorque<<"torque_process_point "<< (IDQ.q*Kt/*3/2*np*M*M/Lr*/*angle.get_imr())<<endl;
			//torque_control.set_setpoint(vel.get_pid_result());   								
			
			if(Wm < Wn) 
				{
				//torque_control_Min_pid_res=-Tm1;//?
				//torque_control_Max_pid_res=Tm1;
				//torque_control.act_min_max(torque_control_Min_pid_res,torque_control_Max_pid_res); 
				//if (vel.get_pid_result() > Tm1)
				  //torque_control.set_setpoint(Tm1);
				  //{vel_Max_pid_res = Tm1; vel.calc_pid();}
				//else if (vel.get_pid_result() < -Tm1)
				  //{*/vel_Min_pid_res = -Tm1; /*vel.calc_pid();*/}
				//else	
				  //{  
					 vel.act_min_max(-Tm1*1.06,Tm1*1.06);
					 vel.calc_pid();		
					 fTorque<<"vel current error: "<<vel.get_current_error()<<"vel pid result: "<<vel.get_pid_result()<<endl;
					 //torque_control.set_setpoint(vel.get_pid_result()); 
					 //}
				fTorque<<"Tm1: "<<Tm1<<endl;
				
				if(abs(IDQ.q)<=sqrt(Rd_we/Rq_we)*Idn)//ok
					{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}//OK absolute value of iqs
				else{IDQ_d_lma=Idn;};
				if ((IDQ_d_lma<Idmin) || isnanf(IDQ_d_lma))IDQ_d_lma=Idmin;
				}
			else {if (Wm < Wc ){
					float Tm2=Kt*sqrt(pow(Vmax/(Wm*Ls),2)-Imax*Imax*ro*ro)*sqrt(Imax*Imax-pow(Vmax/(Wm/*r()*np*/*Ls),2))/(1-ro*ro);
					//torque_control_Min_pid_res=-Tm2;//?
					//torque_control_Max_pid_res=Tm2;
					//torque_control.act_min_max(torque_control_Min_pid_res,torque_control_Max_pid_res);
					//if (vel.get_pid_result() > Tm2)
				    //  torque_control.set_setpoint(Tm2);
				    //else	
				    //  torque_control.set_setpoint(vel.get_pid_result());
					vel.act_min_max(-Tm2*1.06,Tm2*1.06);//TODO
					vel.calc_pid();		
					//torque_control.set_setpoint(vel.get_pid_result());
					fTorque<<"Tm2: "<<Tm2<<endl;
					
					if(abs(IDQ.q)<=Vmax/(Wm/*r()*np*/*Ls*ro)/sqrt(pow(ro,-2)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {IDQ_d_lma=sqrt(Imax*Imax/2-sqrt(Imax*Imax*Imax*Imax-4.0*vel.get_pid_result()*vel.get_pid_result()/Kt/Kt)/2);};
					}
				else {
					float Tm3=Kt*(pow((Vmax/Wm/*r()*np*/*Ls),2)/(2.0*ro));
					//torque_control_Min_pid_res=-Tm3;//?
					//torque_control_Max_pid_res=Tm3;
					//torque_control.act_min_max(torque_control_Min_pid_res,torque_control_Max_pid_res);
					//if (vel.get_pid_result() > Tm3)
				    //  torque_control.set_setpoint(Tm3);
				    //else	
				    //  torque_control.set_setpoint(vel.get_pid_result());
					vel.act_min_max(-Tm3*1.06,Tm3*1.06);
					vel.calc_pid();		
					//torque_control.set_setpoint(vel.get_pid_result());
					fTorque<<"Tm3: "<<Tm3<<endl;
					
					if(abs(IDQ.q)<=Vmax/(Wm/*r()*np*/*Ls*ro)/sqrt(pow(ro,-2)+Rd_we/Rq_we)*sqrt(Rd_we/Rq_we))//Tp3=Tp2,?use iqs<= or Tp 
						{IDQ_d_lma=sqrt(Rq_we/Rd_we)*abs(IDQ.q);}
					else {double Te=vel.get_pid_result();IDQ_d_lma=sqrt((Vmax*Vmax+sqrt(pow(Vmax,4)-4.0*pow(Wm/*r()*np*/,4)*ro*ro*Ls*Ls*Ls*Ls*Te*Te/Kt/Kt))/(2*pow(Wm/*r()*np*/*Ls,2)));//TODO
					};
				};
				if ((IDQ_d_lma<Idmin) || isnanf(IDQ_d_lma))IDQ_d_lma=Idmin;
				};//TODO ?flux min?
			
			
		
			fTorque << "tc_ref:  "<< vel.get_pid_result() <<endl;
			torque_control.set_setpoint(vel.get_pid_result());
			torque_control.calc_pid();
			fTorque<<"torque current error: "<<torque_control.get_current_error()<<"torque pid result: "<<torque_control.get_pid_result()<<endl;
			
			current_control_y.set_process_point(IDQ.q);
			current_control_y.set_setpoint(torque_control.get_pid_result());
			current_control_y.calc_pid();
			
			////flux_control.set_process_point(angle.get_imr());
			////flux_control.set_setpoint(imrref);//TODO
			////flux_control.calc_pid();
			IDQ_d__lma<<n*T<<"sec. IDQ_D_lma:"<<IDQ_d_lma<<endl<<"IDQ_d: "<<IDQ.d<<endl;
			current_control_x.set_process_point(/*angle.get_imr()*/IDQ.d);// i changed: put imr instead IDQd BUT not best
			current_control_x.set_setpoint(IDQ_d_lma*0.9/*flux_control.get_pid_result()*/);//TODO 0.9
			current_control_x.calc_pid();
		
			//double VDQd_p=VDQ.d;
			VDQ.d=current_control_x.get_pid_result();											   
			//VDQ.q=current_control_y.get_pid_result(); //							
			//double VDQq_p=VDQ.q;
			VDQ.q=torque_control.get_pid_result();			
//--------------			
						
			tTwoPhase VDQ_alpha_beta(VDQ_alpha_esc,VDQ_beta_esc);
			double MM=Park(RotorFluxAngle,VDQ_alpha_beta).d;//VDQd_p-VDQ.d;
			//VDQd_p=VDQ.d;
			
			double N=Park(RotorFluxAngle,VDQ_alpha_beta).q;//VDQq_p-VDQ.q;
			//VDQq_p=VDQ.q;
			
			double d_sobre_Tr=IDQd/IDQq*(MM*IDQq-N*IDQd)/(ro*Ls*IDQq*IDQq+Ls*IDQd*IDQd);
			fIDQ <<"MM "<<MM<<" N "<<N<<"d_sobre Tr: "<< d_sobre_Tr<<" m_Tr_calc_gl: "<<m_Tr_calc_gl<<endl;
			
			//bool flag_=false;
			//if (IDQq<0){VDQ_rfa.d*=-1;IDQd*=-1;IDQq*=-1;flag_=true;}
			
			fIDQ<<" VDQ_rfa.d: "<<VDQ_rfa.d<<endl<<" IDQd: "<<IDQd<<" IDQq: "<<IDQq<<endl;
						
			
			cout<<"VDQ.q:"<<VDQ.q<<endl;;fIDQ<<"VDQ.d:"<<VDQ.d<<endl<<"VDQ.q:"<<VDQ.q<<endl;
			
						//if ( (stc_p-VDQ.q>0 && incr_decr>0) || (stc_p-VDQ.q<0 && incr_decr<0) ) sinal_*=-1;				
			//if (IDQq>=-30 && IDQq<=30)sinal_=0.0;
			fIDQ<<" incre_decres_stc:"<<stc_p-VDQ.q<<" incre_decreas_IDQ "<< incr_decr<<endl;
			//if (n>200 && n<100000){Rr*=1.0000022/*1/1.0000002*/;};//TODO remove at end, this is to simulate variations of Tr, the real value influences the gain error
			
			//if (n==220){Rr*=1.3;};
			if (n>25000 /*&&the following is because it dont work at 0 power: (IDQq < -100 || IDQq > 100 )*/) {//steady state TODO				
				
				double error=this->VDQ_rfa.d-(Rs*IDQd-angle.get_wm()*((Ls*Lr-M*M)/Lr)*IDQq);
				//if (flag_){VDQ_rfa.d*=-1;IDQd*=-1;IDQq*=-1;}
					//m_Tr_calc_gl*=(1+0.000005);
					sobre_Tr_comp/*+*/=d_sobre_Tr;
					fIDQ<<" sobre_Tr_comp "<<sobre_Tr_comp<<endl;
					double sobre_Tr=1/m_Tr_calc_gl-sobre_Tr_comp*0.000001;
					double Tr__=1/sobre_Tr;//63 bem depois divergiu
					fIDQ<<" Tr__ "<<Tr__<<" m_Tr_calc_gl "<<m_Tr_calc_gl<<endl;
				s/*+*/=error;//dont work:integral controller, work proporcional only but need tune de following numeric value 
				double pi_contr=(s)*/*the following value is the gain, adjusted from the rate of change of Rr(Tr=Lr/Rr)*/0.000002/*0.000025*/;
				if (IDQq<-/*10:maybe more secure 11.5*/10.5 && (velocidade<-/*30*/2 ) /*10//-44//-40*//*&& VDQ.q>0*/ /*&& (IDQq <= -100 || IDQq >= 100 )*/){
				////if (torque_control.get_pid_result()<0){
				  sinal_=/*-*/1;}
				  else if(IDQq>10.5 && (velocidade<-2 )){sinal_=-1;} 
						else if (IDQq<-10.5 && velocidade >2 ){sinal_=-1;}else if (IDQq>10.5 && velocidade >2){sinal_=1;}else{sinal_=0;}
				//if (velocidade<-30){sinal_*=-1;}else if (velocidade>30){sinal_=1;} else {sinal_=0;};
					//TODO vel velref idqq
				/*m_Tr_calc_gl*/double Tr___=((m_Tr_calc_gl)+sinal_*pi_contr);
			////		for (int i=0;i<8;i++){
				////	 vec_MediaTr_[i]=vec_MediaTr_[i+1];}
					////vec_MediaTr_[8]=Tr___;
					////double fff=0.0;				
					////for (int i=0;i<9;i++){
					 ////fff+=vec_MediaTr_[i];fIDQ<<" fff "<<fff<<endl;}
					////m_Tr_calc_gl=fff/9;
				m_Tr_calc_gl=Tr___;
				fIDQ<<" Tr___ "<<Tr___<<" error: "<<error<<endl;
				}; 
			fIDQ<<" TR do motor: "<<Lr/Rr<<endl;
			stc_p=VDQ.q;

//+Vdecouple-------------	
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
		
			VDQ.d=VDQ.d/*-wmr_1*Lsro*IDQ.q*//*(this is part of usx, the PI output)+Rs*IDQ_d1*/+(/*(idem previous)Lsro+*/T_lag*Rs)*IDQ_d_p+T_lag*Lsro*IDQ_d_pp-wmr1*(Lsro-T_lag*Rs)*IDQ_q1+wmr1*wmr1*(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))/*TODO:modulo imr*/-T_lag*Lsro*IDQ_q1*wmr_p;				
			VDQ.q=VDQ.q/*+wmr_1*Lsro*IDQ_d_p*T_lag*//*(this is part of usy, the PI output)+Lsro*IDQ_q_p*//*-wmr1*T_lag*Lsro*IDQ_d_p*//*(ideyntastic_check_on_open = 1 previous)+Rs*IDQ_q1*/+T_lag*Rs*IDQ_q_p+T_lag*Lsro*IDQ_q_pp+wmr1*(Lsro-T_lag*Rs)*IDQ_d1+wmr1*wmr1*T_lag*Lsro*IDQ_q1+(T_lag*Lsro*IDQ_d1+T_lag*(Ls-Lsro)*abs(angle.get_imr()))*wmr_p+(Ls-Lsro)*wmr1*abs(angle.get_imr());//angle.wmr1*Lsro*IDQ.d+(Ls-Lsro)*wmr1*angle.get_imr();	
//-------------			
				
			
//-------------			
			cout<<"VDQ.d +decoupling:"<<VDQ.d<<endl;;fIDQ<<"VDQ.d +decoupling:"<<VDQ.d<<endl;
			cout<<"VDQ.q +decoupling:"<<VDQ.q<<endl;;fIDQ<<"VDQ.q +decoupling:"<<VDQ.q<<endl;
			//VDQ.d=VDQ.d-angle.get_wm()*Lsro*IDQ.q;cout<<"VDQD+"<<VDQ.d<<endl;//Rs*IDQ.d-np*2/3*motor::wr*(Lsro+Lrro)*IDQ.q;				
			//VDQ.q=VDQ.q+angle.get_wm()*Lsro*IDQ.d+(Ls-Lsro)*angle.get_wm()*angle.get_imr();
//cout <<"VDQ.q+com"<<VDQ.q<<endl;
fImr<<n*T<<"sec. imr:"<<angle.get_imr()/*<<"TTTTTTTTTTTEEEEEEE"<<(3/2*np*M*M/Lr*IDQq*IDQd)*/<<endl;	
		//TODO:ver codigo de test PID em q referem-se ao erro e pid_set_integral
		};
		n++;//TODO:2-?only is necessary in simulation,can be changed in embedded by flag 1-se n proximo do limite- n=5001; 
		timer_Tr++;//to count the periods between Tr read
		V=InvPark((RotorFluxAngle),VDQ);
		//abc_voltage=InvClarke(V);//TODO ??remove in real
		return V;
	};

