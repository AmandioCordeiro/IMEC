//#include <stdio.h>
//#include <iostream>
//#include <math.h>
//#include <tgmath.h>
//#include <complex> 
//#include <vector>
//#include "filter_rtc.hpp"
//#include "bissection_exclusion4.hpp"
#include "Rotor_Time_Const.hpp"
//#include "control_motor.cpp"
ofstream hessian ("hessian.txt");

using namespace std;
//*******
//filter

//*********
//mutex  coutt;
double Tr_calc_gl;
double m_Tr_calc_gl;//TODO insert at the end Tr motor value
//extern ofstream Tr_s ;
extern ofstream PEI_txt;

double M=/*0.51233;*/0.050382;//0.0117//0.069312//mutual inductance
double Ls=/*(0.00845+M);*/0.051195;//0.014//0.07132


double Lr=/*(0.00515+M);*/0.052571;//0.014//0.07132
double J=(0.089);//TODO:??

//!!!!!!!!!!!!!!double dy_nt(double &y1, double &y_1);//y1 means y[n+1] 
	//return dynt=(y1-y_1)/(2*per);

//!!!!!!!-tirar isto final-double d2y_nt(double &y1, double &y,double &y_1);//derived can only be obtained  when n>=? 
double ro=(1-(M*M)/(Ls*Lr));//0.999863//1-*M/(Ls*Lr)//alterar nome
double B=(M/(ro*Ls*Lr));//M/(ro*Ls*Lr)
double roLs=(ro*Ls);//0.013998//ro*Ls
double MB=(B*M);//0.69851//B*M
double one_p_MB=(1+MB);//1.69851//1+MrB

double dy_nt(double /*&*/y1, double /*&*/y_1){//y1 significa y[n+1] 
	return ((y1-y_1)/(2*per));};

double d2y_nt(double /*&*/y1, double /*&*/y,double /*&*/y_1){//derived can only be obtained  when n>=? 
	return ((y1-2*y+y_1)/(per*per));};

//class Y_W_{
//protected:
//	double Y11,Y21,W11,W12,W13,W14,W15,W16,W17,W18,W21,W22,W23,W24,W25,W26,W27,W28;
//	double Ry;double Rwy[7];double Rw[7][7]; 
//	double isx,isx_1,isx_2,isy,isy_1,isy_2,usx,usx_2,usy,usy_2,w,w_2;
//	double usx_temp,usy_temp,wr_temp;
//	int n;

	void Y_W_::upd_Y_and_W(/*double &isx,double &isx_1,double &isx_2,double &isy,double &isy_1,double &isy_2,double &usx,double &usx_2,double &usy,double &usy_2,double &w,double &w_2*/){//wr

		double d2isx=d2y_nt(isx,isx_1,isx_2);//cout<<"d2isx"<<d2isx<<endl;//y_nt=f.filtred_isx_1();d2ysx_nt
		double d2isy=d2y_nt(isy,isy_1,isy_2);//cout<<"d2isy"<<d2isy<<endl;
		double disy=dy_nt(isy,isy_2);//cout<<"disy"<<disy<<endl;
		double disx=dy_nt(isx,isx_2);//cout<<"disx"<<disx<<endl;
		double dusx=dy_nt(usx,usx_2);//cout<<"dusx"<<dusx<<endl;
		double dusy=dy_nt(usy,usy_2);//cout<<"dusy"<<dusy<<endl;
		double dw=dy_nt(w,w_2);//cout<<"dw"<<dw<<endl;

		Y11=d2isx-np*isy*dw-np*w*disy-np*np*w*w*MB*isx-dusx/(roLs);//cout<<"y11"<<Y11<<endl;
		Y21=d2isy+np*isx*dw+np*w*disx-np*np*w*w*MB*isy-dusy/(roLs);//cout<<"y21"<<Y21<<endl;

		W11=-disx;//cout<<"w11"<<W11<<endl;
		W12=-disx+isy*np*w+np*w*MB*isy+usx/roLs;//cout<<"w12"<<W11<<endl;
		W13=MB*isx;//cout<<"w13"<<W11<<endl;
		W14=-isx;//cout<<"w14"<<W14<<endl;
		W15=np*disy*dw+np*np*(w*isx*dw-w*w*disx)+np*np*np*w*w*w*isy*one_p_MB+(np*np*w*w*usx-np*usy*dw)/(roLs);//cout<<"w15"<<W15<<endl;
		W16=np*isy*dw-np*np*w*w*isx;//cout<<"w16"<<W16<<endl;
		W17=np*np*(isx*w*dw-w*w*disx);//cout<<"w17"<<W17<<endl;
		W18=np*np*(w*disx*dw-w*w*d2isx)+disy*np*np*np*w*w*w-np*np/roLs*(w*usx*dw-w*w*dusx);//cout<<"w18"<<W18<<endl;

		W21=-disy;//cout<<"w21"<<W21<<endl;
		W22=-disy-isx*np*w-np*w*MB*isx+usy/roLs;//cout<<"w22"<<W22<<endl;
		W23=MB*isy;//cout<<"w23"<<W23<<endl;
		W24=-isy;//cout<<"w24"<<W24<<endl;
		W25=-np*disx*dw+np*np*(w*isy*dw-w*w*disy)-np*np*np*w*w*w*isx*one_p_MB+(np*np*w*w*usy+np*usx*dw)/(roLs);//cout<<"w25"<<W25<<endl;
		W26=-np*isx*dw-np*np*w*w*isy;//cout<<"w26"<<W26<<endl;
		W27=np*np*(isy*w*dw-w*w*disy);//cout<<"w27"<<W27<<endl;
		W28=np*np*(w*disy*dw-w*w*d2isy)-disx*np*np*np*w*w*w-np*np/roLs*(w*usy*dw-w*w*dusy);//cout<<"w28"<<W28<<endl;
	};
	void Y_W_::upd_Ry_(){
			//std::lock_guard<std::mutex> lockk(coutt);
			Ry=Ry+Y11*Y11+Y21*Y21;//cout<<"Ry:"<<Ry<<endl; 

			Rwy[0]=Rwy[0]+W11*Y11+W21*Y21;//cout<<"Rwy0"<<Rwy[0]<<endl;//indice 0 corresponde a +1 na matriz
			Rwy[1]=Rwy[1]+W12*Y11+W22*Y21;
			Rwy[2]=Rwy[2]+W13*Y11+W23*Y21;
			Rwy[3]=Rwy[3]+W14*Y11+W24*Y21;
			Rwy[4]=Rwy[4]+W15*Y11+W25*Y21;
			Rwy[5]=Rwy[5]+W16*Y11+W26*Y21;//cout<<"Rwy5"<<Rwy[5]<<endl;
			Rwy[6]=Rwy[6]+W17*Y11+W27*Y21;
			Rwy[7]=Rwy[7]+W18*Y11+W28*Y21;//for(int i=0;i<8;++i)cout<<"Rwy"<<Rwy[i]<<endl;


			Rw[0][0]=Rw[0][0]+W11*W11+W21*W21;
			Rw[1][0]=Rw[1][0]+W12*W11+W22*W21;
			Rw[2][0]=Rw[2][0]+W13*W11+W23*W21;//cout<<"Rw20"<<Rw[2][0]<<endl;
			Rw[3][0]=Rw[3][0]+W14*W11+W24*W21;
			Rw[4][0]=Rw[4][0]+W15*W11+W25*W21;//cout<<"Rw40"<<Rw[4][0]<<endl;
			Rw[5][0]=Rw[5][0]+W16*W11+W26*W21;
			Rw[6][0]=Rw[6][0]+W17*W11+W27*W21;
			Rw[7][0]=Rw[7][0]+W18*W11+W28*W21;//for(int i=0;i<8;++i)cout<<"Rw...0"<<Rw[i][0]<<endl;

			Rw[0][1]=Rw[0][1]+W11*W12+W21*W22;
			Rw[1][1]=Rw[1][1]+W12*W12+W22*W22;
			Rw[2][1]=Rw[2][1]+W13*W12+W23*W22;
			Rw[3][1]=Rw[3][1]+W14*W12+W24*W22;//cout<<"Rw31"<<Rw[3][1]<<endl;
			Rw[4][1]=Rw[4][1]+W15*W12+W25*W22;
			Rw[5][1]=Rw[5][1]+W16*W12+W26*W22;//cout<<"Rw51"<<Rw[5][1]<<endl;
			Rw[6][1]=Rw[6][1]+W17*W12+W27*W22;//cout<<"Rw61"<<Rw[6][1]<<endl;
			Rw[7][1]=Rw[7][1]+W18*W12+W28*W22;//for(int i=0;i<8;++i)cout<<"Rw...1"<<Rw[i][1]<<endl;

			Rw[0][2]=Rw[0][2]+W11*W13+W21*W23;//cout<<"Rw02"<<Rw[0][2]<<endl;
			Rw[1][2]=Rw[1][2]+W12*W13+W22*W23;
			Rw[2][2]=Rw[2][2]+W13*W13+W23*W23;
			Rw[3][2]=Rw[3][2]+W14*W13+W24*W23;//cout<<"Rw32"<<Rw[3][2]<<endl;
			Rw[4][2]=Rw[4][2]+W15*W13+W25*W23;
			Rw[5][2]=Rw[5][2]+W16*W13+W26*W23;
			Rw[6][2]=Rw[6][2]+W17*W13+W27*W23;//cout<<"Rw62"<<Rw[6][2]<<endl;
			Rw[7][2]=Rw[7][2]+W18*W13+W28*W23;//for(int i=0;i<8;++i)cout<<"Rw...2"<<Rw[i][2]<<endl;

			Rw[0][3]=Rw[0][3]+W11*W14+W21*W24;
			Rw[1][3]=Rw[1][3]+W12*W14+W22*W24;//cout<<"Rw13"<<Rw[1][3]<<endl;
			Rw[2][3]=Rw[2][3]+W13*W14+W23*W24;//cout<<"Rw23"<<Rw[2][3]<<endl;
			Rw[3][3]=Rw[3][3]+W14*W14+W24*W24;
			Rw[4][3]=Rw[4][3]+W15*W14+W25*W24;//cout<<"Rw43"<<Rw[4][3]<<endl;
			Rw[5][3]=Rw[5][3]+W16*W14+W26*W24;
			Rw[6][3]=Rw[6][3]+W17*W14+W27*W24;
			Rw[7][3]=Rw[7][3]+W18*W14+W28*W24;//for(int i=0;i<8;++i)cout<<"Rw...3"<<Rw[i][3]<<endl;//cout<<"Rw73"<<Rw[7][3]<<endl;

			Rw[0][4]=Rw[0][4]+W11*W15+W21*W25;//cout<<"Rw04"<<Rw[0][4]<<endl;
			Rw[1][4]=Rw[1][4]+W12*W15+W22*W25;
			Rw[2][4]=Rw[2][4]+W13*W15+W23*W25;
			Rw[3][4]=Rw[3][4]+W14*W15+W24*W25;//cout<<"Rw34"<<Rw[3][4]<<endl;
			Rw[4][4]=Rw[4][4]+W15*W15+W25*W25;
			Rw[5][4]=Rw[5][4]+W16*W15+W26*W25;
			Rw[6][4]=Rw[6][4]+W17*W15+W27*W25;
			Rw[7][4]=Rw[7][4]+W18*W15+W28*W25;//for(int i=0;i<8;++i)cout<<"Rw...4"<<Rw[i][4]<<endl;

			Rw[0][5]=Rw[0][5]+W11*W16+W21*W26;
			Rw[1][5]=Rw[1][5]+W12*W16+W22*W26;//cout<<"Rw15"<<Rw[1][5]<<endl;
			Rw[2][5]=Rw[2][5]+W13*W16+W23*W26;
			Rw[3][5]=Rw[3][5]+W14*W16+W24*W26;
			Rw[4][5]=Rw[4][5]+W15*W16+W25*W26;
			Rw[5][5]=Rw[5][5]+W16*W16+W26*W26;
			Rw[6][5]=Rw[6][5]+W17*W16+W27*W26;
			Rw[7][5]=Rw[7][5]+W18*W16+W28*W26;//for(int i=0;i<8;++i)cout<<"Rw...5"<<Rw[i][5]<<endl;

			Rw[0][6]=Rw[0][6]+W11*W17+W21*W27;
			Rw[1][6]=Rw[1][6]+W12*W17+W22*W27;//cout<<"Rw16"<<Rw[1][6]<<endl;
			Rw[2][6]=Rw[2][6]+W13*W17+W23*W27;//cout<<"Rw26"<<Rw[2][6]<<endl;
			Rw[3][6]=Rw[3][6]+W14*W17+W24*W27;
			Rw[4][6]=Rw[4][6]+W15*W17+W25*W27;
			Rw[5][6]=Rw[5][6]+W16*W17+W26*W27;
			Rw[6][6]=Rw[6][6]+W17*W17+W27*W27;
			Rw[7][6]=Rw[7][6]+W18*W17+W28*W27;//for(int i=0;i<8;++i)cout<<"Rw...6"<<Rw[i][6]<<endl;//cout<<"Rw76"<<Rw[7][6]<<endl;

			Rw[0][7]=Rw[0][7]+W11*W18+W21*W28;
			Rw[1][7]=Rw[1][7]+W12*W18+W22*W28;
			Rw[2][7]=Rw[2][7]+W13*W18+W23*W28;
			Rw[3][7]=Rw[3][7]+W14*W18+W24*W28;//cout<<"Rw37"<<Rw[3][7]<<endl;
			Rw[4][7]=Rw[4][7]+W15*W18+W25*W28;
			Rw[5][7]=Rw[5][7]+W16*W18+W26*W28;
			Rw[6][7]=Rw[6][7]+W17*W18+W27*W28;//cout<<"Rw67"<<Rw[6][7]<<endl;
			Rw[7][7]=Rw[7][7]+W18*W18+W28*W28;//for(int i=0;i<8;++i)cout<<"Rw...7"<<Rw[i][7]<<endl;
		};

//public:
	Y_W_::Y_W_():Rss/*0.258*/(0.10941),//*0.435*/),//oooo
 Trr(/*1.12252*/m_Tr_calc_gl/*0.17703*/),/*Lr/Rrr*/////ooo
Ry(0.0),Y11(0.0),Y21(0.0),W11(0.0),W12(0.0),W13(0.0),W14(0.0),W15(0.0),W16(0.0),W17(0.0),W18(0.0),W21(0.0),W22(0.0),W23(0.0),W24(0.0),W25(0.0),W26(0.0),W27(0.0),W28(0.0),isx(0.0),isx_1(0.0),isx_2(0.0),isy(0.0),isy_1(0.0),isy_2(0.0),usx(0.0),usx_2(0.0),usy(0.0),usy_2(0.0),w(0.0),w_2(0.0),usx_temp(0.0),usy_temp(0.0),wr_temp(0.0){
	for(int j=0;j<8;j++){Rwy[j]=0.0;}/*	Rwy[8]={0};Rw[8][8]={0.0};*/for(int i=0;i<8;i++){for(int j=0;j<8;j++){Rw[i][j]=0.0;}};n=0;
	};
	Y_W_::~Y_W_(){};
//	Y_W_():Rwy{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},Rw{{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}}{};
	void Y_W_::enter_medition(double Isx,double Isy,double Usx, double Usy, double Wr){
			//std::lock_guard<std::mutex> lockk(coutt);
			if (Y_W_::n==3)
			{ 
				isx_2=isx_1;
				isy_2=isy_1;
				usx_2=usx_temp;
				usy_2=usy_temp;
				w_2=wr_temp;
				
				isx_1=isx;
				isy_1=isy;
				usx_temp=usx;
				usy_temp=usy;
				wr_temp=w;				
				
				isx=Isx;
				isy=Isy;
				usx=Usx;
				usy=Usy;
				w=Wr;////cout<<"ultimo_Y_W_n"<<Y_W_::n<<endl<<"isx_2:"<<isx_2<<endl<<"isx_1:"<<isx_1<<"isx"<<isx<<"IIsx"<<Isx<<endl;	
				upd_Y_and_W();	//can call upd_Y_and_W
				upd_Ry_();
			}	
			else if (Y_W_::n==0){
				isx_2=Isx;
				isy_2=Isy;
				usx_2=Usx;
				usy_2=Usy;
				w_2=Wr;
				Y_W_::n++;	//cout<<"ultimo_n"<<Y_W_::n<<endl;
				}
			else if (Y_W_::n==1){

				isx_1=Isx;
				isy_1=Isy;
				usx_temp=Usx;
				usy_temp=Usy;
				wr_temp=Wr;
				Y_W_::n++;	//cout<<"ultimo_n"<<Y_W_::n<<endl;
			}	
			else{//==2
				isx=Isx;
				isy=Isy;
				usx=Usx;
				usy=Usy;
				w=Wr;
				//can call upd_Y_and_W
				Y_W_::n++;		
				upd_Y_and_W();
				upd_Ry_();//cout<<"ultimo_n"<<Y_W_::n<<endl;
			}
			
	}
//};
//class r_k2:public Y_W_ {
//protected:
//	float a1[6];
//	float a0[7];
//	float b2[5];
//	float b1[7];
//	float b0[8];
//float a0_2[14];float a0_2b2[19];float a0a1[13];float a0a1b1[20];float a1_2[12];float a1_2b0[20];

	//dimensoes: ex. polin grau 2 logo 2 o que quer dizer q vector tera mais um alemento 2+1, v[3], considerando o 0	
	         // dim pol_fin=dim1+dim2		
	void/*para multiplicar polinomios*/ r_k2::conv(double *pol1,int dim1, double *pol2, int dim2, double *pol_fin){//dim pol_fin 
		//todo:trocar por ciclos for
		for (int e=(dim1+dim2);e>=0;e--)pol_fin[e]=0.0;
			while(dim1>=0){
				for(int i=dim2;i>=0;i--){
				pol_fin[(dim1+i)]=pol_fin[(dim1+i)]+pol1[dim1]*pol2[i];
				};			
			dim1--;
			};
	};
	void r_k2::adic(double *pol1,  int dim1, double *pol2,  int dim2){//dim2<dim1vv
		for( int i=0;i<=dim2;++i)pol1[i]=pol1[i]+pol2[i];
	};
	void r_k2::inv(double *pol_fin, unsigned int dimo){
		for( int i=0;i<=(dimo);++i)pol_fin[i]=-pol_fin[i];
	};
//	public:
 r_k2::r_k2():Y_W_(){
	 for(int j=0;j<7;j++){a1[j]=0.0;}
	 for(int j=0;j<8;j++){a0[j]=0.0;}
	 for(int j=0;j<7;j++){b2[j]=0.0;}
	 for(int j=0;j<8;j++){b1[j]=0.0;}
	 for(int j=0;j<9;j++){b0[j]=0.0;}
	 for(int j=0;j<15;j++){a0_2[j]=0.0;}
	 for(int j=0;j<21;j++){a0_2b2[j]=0.0;}
	 for(int j=0;j<14;j++){a0a1[j]=0.0;}
	 for(int j=0;j<21;j++){a0a1b1[j]=0.0;}
	 for(int j=0;j<13;j++){a1_2[j]=0.0;}
	 for(int j=0;j<21;j++){a1_2b0[j]=0.0;}
	 /*a1[7]={0.0};a0[8]={0.0};b2[6]={0.0};b1[8]={0.0};b0[9]={0.0};a0_2[15]={0.0};a0_2b2[20]={0.0};a0a1[14]={0.0};a0a1b1[21]={0.0};a1_2[13]={0.0};a1_2b0[21]={0.0};*/
	 };
 r_k2::~r_k2(){};
	void r_k2::update_r_k2(){
		
		a1[6]=2*Rw[3][3];
		a1[5]=2*(Rw[0][3]+Rw[3][0]);
		a1[4]=2*(Rw[3][5]+Rw[5][3]/**/+Rw[0][0]);
		a1[3]=2*(Rw[0][5]+Rw[5][0]+Rw[3][6]+Rw[6][3]);
		a1[2]=2*(Rw[0][6]+Rw[6][0]+Rw[5][5]);
		a1[1]=2*(Rw[5][6]+Rw[6][5]);
		a1[0]=2*Rw[6][6];//tava rw77
		
		a0[7]=(Rw[2][3]+Rw[3][2]);
		a0[6]=(Rw[0][2]+Rw[1][3]+Rw[2][0]+Rw[3][1]);
		a0[5]=(-2*Rwy[3]+Rw[0][1]+Rw[2][5]+Rw[1][0]+Rw[5][2]);
		a0[4]=(-2*Rwy[0]+Rw[1][5]+Rw[2][6]+Rw[3][4]+Rw[4][3]+Rw[6][2]+Rw[5][1]);//alteradoOOOOO 1 5 por 51 neste final
		a0[3]=(-2*Rwy[5]+Rw[0][4]+Rw[1][6]+Rw[3][7]+Rw[7][3]+Rw[6][1]+Rw[4][0]);
		a0[2]=(-2*Rwy[6]+Rw[0][7]+Rw[4][5]+Rw[7][0]+Rw[5][4]);
		a0[1]=(Rw[4][6]+Rw[5][7]+Rw[6][4]+Rw[7][5]);
		a0[0]=(Rw[6][7]+Rw[7][6]);//15_07 for(int i=0;i<8/*k2.size()*/;i++){cout<<"a0:"<<a0[i]<<endl;};cout<<"aaaaaaa"<<endl;
		
		b2[6]=2*Rw[3][3];//aumentei
		b2[5]=(Rw[3][0]+Rw[0][3]);
		b2[4]=0;
		b2[3]=(-Rw[3][6]-Rw[6][3]/**/-Rw[0][5]-Rw[5][0]);
		b2[2]=(-2*(Rw[6][0]+Rw[0][6]+Rw[5][5]));
		b2[1]=(-3*(Rw[5][6]+Rw[6][5])/*-4*Rw[6][6]*/);
		b2[0]=-4*Rw[6][6];//tava errado
		
		b1[7]=3*(Rw[2][3]+Rw[3][2]);
		b1[6]=2*(Rw[2][0]+Rw[1][3]+Rw[3][1]+Rw[0][2]);
		b1[5]=(Rw[0][1]+Rw[2][5]+Rw[1][0]+Rw[5][2])/**/-2*Rwy[3];
		b1[4]=0;
		b1[3]=-(Rw[1][6]+Rw[3][7]+Rw[7][3]+Rw[6][1]/**/+Rw[4][0]+Rw[0][4]-2*Rwy[5]);
		b1[2]=(-2*(Rw[0][7]+Rw[4][5]+Rw[7][0]+Rw[5][4])+/**/4*Rwy[6]);
		b1[1]=(-3*(Rw[4][6]+Rw[5][7]+Rw[7][5]+Rw[6][4]));
		b1[0]=(/*4*Rwy[5]*/-4*(Rw[6][7]+Rw[7][6]));//15_07 for(int i=0;i<8/*k2.size()*/;i++){cout<<"b1:"<<b1[i]<<endl;};cout<<"bbb111"<<endl;
		
		b0[8]=4*Rw[2][2];
		b0[7]=3*(Rw[1][2]+Rw[2][1]);
		b0[6]=(-4*Rwy[2]+2*Rw[1][1]);
		b0[5]=Rw[2][4]+Rw[4][2]/**/-2*Rwy[1];
		b0[4]=0;
		b0[3]=2*Rwy[4]-(Rw[1][7]+Rw[7][1]);
		b0[2]=4*Rwy[7]-2*Rw[4][4];
		b0[1]=-3*(Rw[4][7]+Rw[7][4]);
		b0[0]=-4*Rw[7][7];

		conv(a0,7,a0,7,a0_2);//for(int i=0;i<15/*k2.size()*/;i++){cout<<"a0_2:"<<a0_2[i]<<endl;};cout<<"iiiiiii"<<endl;
		conv(a0_2,14,b2,6,a0_2b2);//for(int i=0;i<21/*k2.size()*/;i++){cout<<"a0_2b2:"<<a0_2b2[i]<<endl;};cout<<"iiiiiii"<<endl;
		conv(a0,7,a1,6,a0a1);//for(int i=0;i<14/*k2.size()*/;i++){cout<<"a0a1:"<<a0a1[i]<<endl;};cout<<"iiiiiii"<<endl;
		conv(a0a1,13,b1,7,a0a1b1);//for(int i=0;i<21/*k2.size()*/;i++){cout<<"a0a1b1:"<<a0a1b1[i]<<endl;};cout<<"iiiiiii"<<endl;
		conv(a1,6,a1,6,a1_2);//for(int i=0;i<13/*k2.size()*/;i++){cout<<"a1_2:"<<a1_2[i]<<endl;};cout<<"iiiiiii"<<endl;
		conv(a1_2,12,b0,8,a1_2b0);//for(int i=0;i<21/*k2.size()*/;i++){cout<<"a1_2b0:"<<a1_2b0[i]<<endl;};cout<<"iiiiiii"<<endl;
		
		this->inv(a0a1b1,20);//for(int i=0;i<21/*k2.size()*/;i++)cout<<"inv"<<a0a1b1[i]<<endl;
		
		this->adic(a0a1b1,20,a0_2b2,20);//for(int i=0;i<21/*k2.size()*/;i++)cout<<"prim_adic"<<a0a1b1[i]<<endl;
		this->adic(a0a1b1,20,a1_2b0,20);//for(int i=0;i<21/*k2.size()*/;i++){cout<<"a0a1b1:"<<a0a1b1[i]<<endl;/*Tr_s<<"a0a1b1"<<a0a1b1[i]<<endl;*/};cout<<"iiiiiii"<<endl;//r_k2=a0a1b1
	};
//};
//class Rs_Tr:public r_k2{
//private:
//unsigned k1jf;
//unsigned k2if;
//vector<float> k2;
//vector<float> k1; 
float Rs_Tr::do_it(){
	//std::lock_guard<std::mutex> lockk(coutt);
	update_r_k2();//alteradoOOOOOOO criar construtores nos construtores das classes com herancas
	
	int ind;if(a0a1b1[20]==0&&a0a1b1[19]==0)ind=18; else if(a0a1b1[20]==0){ind=19;}else {ind=20;};//TODO para outros indices com zero, ex. o penultimo
	k2=real_zeros_pol(a0a1b1,ind);//alterOOO 19->20
	if (k2.size()>0){//15_07for(int i=0;i<k2.size();i++){cout<<"kkkkkkkkkkkkkkkkkkkkk222222222222222:"<<k2[i]<<endl;};cout<<"iiiiiii"<<endl;
		k1.resize(0);
		for(unsigned i=0;i<k2.size();++i){
			double g=-fx(a0,7,k2[i])/fx(a1,6,k2[i]);/*TODO!!!???if(g>0)*/k1.push_back(g);}/*k1.push_back(-fx(a0,7,k2[i])/fx(a1,6,k2[i]));*/ //15_07 for(unsigned i=0;i<k1.size();i++){cout<<"kkkkkkkkkkkkkkkkkkkkk1111111111111111111:"<<k1[i]<<endl;};cout<<"iiiiiii"<<endl;
		float min_Ep2__k1_j__k2_i=0.0;
		//for(unsigned i=0;i<k2.size();++i)//estva errado, nao é combinacao mas sim par de k2
			for (unsigned j=0;j<k1.size();++j)//??
						 {unsigned i=j;//TODO??retirar j e k1jf==k2if,pois i=j e k1jf==k2if
							 float temp=
							 Ry
							 -2*(Rwy[0]*k1[j]+
								Rwy[1]*k2[i]+
								Rwy[2]*k2[i]*k2[i]+
								Rwy[3]*k1[j]*k2[i]+
								Rwy[4]/k2[i]+
								Rwy[5]*k1[j]/k2[i]+
								Rwy[6]*k1[j]/(k2[i]*k2[i])+
								Rwy[7]/(k2[i]*k2[i]))
								+
								(Rw[0][0]*k1[j]+
								Rw[0][1]*k2[i]+
								Rw[0][2]*k2[i]*k2[i]+
								Rw[0][3]*k1[j]*k2[i]+
								Rw[0][4]/k2[i]+
								Rw[0][5]*k1[j]/k2[i]+
								Rw[0][6]*k1[j]/(k2[i]*k2[i])+
								Rw[0][7]/(k2[i]*k2[i]))
								*k1[j]+
								(Rw[1][0]*k1[j]+
								Rw[1][1]*k2[i]+
								Rw[1][2]*k2[i]*k2[i]+
								Rw[1][3]*k1[j]*k2[i]+
								Rw[1][4]/k2[i]+
								Rw[1][5]*k1[j]/k2[i]+
								Rw[1][6]*k1[j]/(k2[i]*k2[i])+
								Rw[1][7]/(k2[i]*k2[i]))
								*k2[i]+
								(Rw[2][0]*k1[j]+
								Rw[2][1]*k2[i]+
								Rw[2][2]*k2[i]*k2[i]+
								Rw[2][3]*k1[j]*k2[i]+
								Rw[2][4]/k2[i]+
								Rw[2][5]*k1[j]/k2[i]+
								Rw[2][6]*k1[j]/(k2[i]*k2[i])+
								Rw[2][7]/(k2[i]*k2[i]))
								*k2[i]*k2[i]+
								(Rw[3][0]*k1[j]+
								Rw[3][1]*k2[i]+
								Rw[3][2]*k2[i]*k2[i]+
								Rw[3][3]*k1[j]*k2[i]+
								Rw[3][4]/k2[i]+
								Rw[3][5]*k1[j]/k2[i]+
								Rw[3][6]*k1[j]/(k2[i]*k2[i])+
								Rw[3][7]/(k2[i]*k2[i]))
							*k1[j]*k2[i]+	
								(Rw[4][0]*k1[j]+
								Rw[4][1]*k2[i]+
								Rw[4][2]*k2[i]*k2[i]+
								Rw[4][3]*k1[j]*k2[i]+
								Rw[4][4]/k2[i]+
								Rw[4][5]*k1[j]/k2[i]+
								Rw[4][6]*k1[j]/(k2[i]*k2[i])+
								Rw[4][7]/(k2[i]*k2[i]))
							/k2[i]+	
								(Rw[5][0]*k1[j]+
								Rw[5][1]*k2[i]+
								Rw[5][2]*k2[i]*k2[i]+
								Rw[5][3]*k1[j]*k2[i]+
								Rw[5][4]/k2[i]+
								Rw[5][5]*k1[j]/k2[i]+
								Rw[5][6]*k1[j]/(k2[i]*k2[i])+
								Rw[5][7]/(k2[i]*k2[i]))
							*k1[j]/k2[i]+
								(Rw[6][0]*k1[j]+
								Rw[6][1]*k2[i]+
								Rw[6][2]*k2[i]*k2[i]+
								Rw[6][3]*k1[j]*k2[i]+
								Rw[6][4]/k2[i]+
								Rw[6][5]*k1[j]/k2[i]+
								Rw[6][6]*k1[j]/(k2[i]*k2[i])+
								Rw[6][7]/(k2[i]*k2[i]))
							*k1[j]/(k2[i]*k2[i])+
								(Rw[7][0]*k1[j]+
								Rw[7][1]*k2[i]+
								Rw[7][2]*k2[i]*k2[i]+
								Rw[7][3]*k1[j]*k2[i]+
								Rw[7][4]/k2[i]+
								Rw[7][5]*k1[j]/k2[i]+
								Rw[7][6]*k1[j]/(k2[i]*k2[i])+
								Rw[7][7]/(k2[i]*k2[i]))
								/(k2[i]*k2[i]);
								

				if(min_Ep2__k1_j__k2_i==0.0){min_Ep2__k1_j__k2_i=temp;k1jf=j;k2if=i;}
					else if(temp<min_Ep2__k1_j__k2_i){min_Ep2__k1_j__k2_i=temp;k1jf=j;k2if=i;}
				};
				


		/*h00=                                                           
		  2/pow(k2[k2if],2)*(Rw[6][6]/pow(k2[k2if],2)+Rw[6][5]/k2[k2if]+k2[k2if]*Rw[6][3]+Rw[6][0])
		  +2/k2[k2if]*(Rw[5][6]/pow(k2[k2if],2)+Rw[5][5]/k2[k2if]+k2[k2if]*Rw[5][3]+Rw[5][0])            
		  +2*k2[k2if]*(Rw[3][6]/pow(k2[k2if],2)+Rw[3][5]/k2[k2if]+k2[k2if]*Rw[3][3]+Rw[3][0])+
		  2*Rw[0][6]/pow(k2[k2if],2)+2*Rw[0][5]/k2[k2if]+2*k2[k2if]*Rw[0][3] +2*Rw[0][0];
		  */                               
										
		 h01=
		-2*(-2*Rwy[6]/pow(k2[k2if],3)-Rwy[5]/pow(k2[k2if],2)+Rwy[3])
		-2/pow(k2[k2if],3)*(Rw[7][6]/pow(k2[k2if],2)+Rw[7][5]/k2[k2if]+k2[k2if]*Rw[7][3]+Rw[7][0])/* */+1/pow(k2[k2if],2)*(-2*Rw[7][6]/pow(k2[k2if],3)-Rw[7][5]/pow(k2[k2if],2)+Rw[7][3])/* */-2/pow(k2[k2if],3)*(Rw[6][7]/pow(k2[k2if],2)+k1[k1jf]*Rw[6][6]/pow(k2[k2if],2)+k1[k1jf]*Rw[6][5]/k2[k2if]+Rw[6][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[6][3]+pow(k2[k2if],2)*Rw[6][2]+k2[k2if]*Rw[6][1]+k1[k1jf]*Rw[6][0])
		+1/pow(k2[k2if],2)*(-2*Rw[6][7]/pow(k2[k2if],3)-2*k1[k1jf]*Rw[6][6]/pow(k2[k2if],3)-k1[k1jf]*Rw[6][5]/pow(k2[k2if],2)-Rw[6][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[6][3]+2*k2[k2if]*Rw[6][2]+Rw[6][1])
		-2*k1[k1jf]/pow(k2[k2if],3)*(Rw[6][6]/pow(k2[k2if],2)+Rw[6][5]/k2[k2if]+k2[k2if]*Rw[6][3]+Rw[6][0])
		+k1[k1jf]/pow(k2[k2if],2)*(-2*Rw[6][6]/pow(k2[k2if],3)-Rw[6][5]/pow(k2[k2if],2)+Rw[6][3])
		-1/pow(k2[k2if],2)*(Rw[5][7]/pow(k2[k2if],2)+k1[k1jf]*Rw[5][6]/pow(k2[k2if],2)+k1[k1jf]*Rw[5][5]/k2[k2if]+Rw[5][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[5][3]+pow(k2[k2if],2)*Rw[5][2]+k2[k2if]*Rw[5][1]+k1[k1jf]*Rw[5][0])
		+1/k2[k2if]*(-2*Rw[5][7]/pow(k2[k2if],3)-2*k1[k1jf]/pow(k2[k2if],3)*Rw[5][6]-((k1[k1jf])/pow(k2[k2if],2))*Rw[5][5]-Rw[5][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[5][3]+2*k2[k2if]*Rw[5][2]+Rw[5][1])
		-k1[k1jf]/pow(k2[k2if],2)*(Rw[5][6]/pow(k2[k2if],2)+Rw[5][5]/k2[k2if]+k2[k2if]*Rw[5][3]+Rw[5][0])
		+k1[k1jf]/k2[k2if]*(-2*Rw[5][6]/pow(k2[k2if],3)-Rw[5][5]/pow(k2[k2if],2)+Rw[5][3])
		-1/pow(k2[k2if],2)*(Rw[4][6]/pow(k2[k2if],2)+Rw[4][5]/k2[k2if]+k2[k2if]*Rw[4][3]+Rw[4][0])
		+1/k2[k2if]*(-2*Rw[4][6]/pow(k2[k2if],3)-Rw[4][5]/pow(k2[k2if],2)+Rw[4][3])
		+k2[k2if]*(-2/pow(k2[k2if],3)*Rw[3][7]-2/pow(k2[k2if],3)*k1[k1jf]*Rw[3][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[3][5]-Rw[3][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[3][3]+2*k2[k2if]*Rw[3][2]+Rw[3][1])
		+Rw[3][7]/pow(k2[k2if],2)
		+k1[k1jf]*(Rw[3][6]/pow(k2[k2if],2)+Rw[3][5]/k2[k2if]+k2[k2if]*Rw[3][3]+Rw[3][0])
		+k1[k1jf]*k2[k2if]*(-2/pow(k2[k2if],3)*Rw[3][6]-Rw[3][5]/pow(k2[k2if],2)+Rw[3][3])
		+k1[k1jf]/pow(k2[k2if],2)*Rw[3][6]+k1[k1jf]/k2[k2if]*Rw[3][5]+Rw[3][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[3][3]+pow(k2[k2if],2)*Rw[3][2]+k2[k2if]*Rw[3][1]+k1[k1jf]*Rw[3][0]+
		2*k2[k2if]*(Rw[2][6]/pow(k2[k2if],2)+Rw[2][5]/k2[k2if]+k2[k2if]*Rw[2][3]+Rw[2][0])+
		pow(k2[k2if],2)*(-2/pow(k2[k2if],3)*Rw[2][6]-Rw[2][5]/pow(k2[k2if],2)+Rw[2][3])
		+k2[k2if]*(-2/pow(k2[k2if],3)*Rw[1][6]-Rw[1][5]/pow(k2[k2if],2)+Rw[1][3])
		+Rw[1][6]/pow(k2[k2if],2)+Rw[1][5]/k2[k2if]+k2[k2if]*Rw[1][3]+Rw[1][0]-2/pow(k2[k2if],3)*Rw[0][7]
		+k1[k1jf]*(-2/pow(k2[k2if],3)*Rw[0][6]-Rw[0][5]/pow(k2[k2if],2)+Rw[0][3])
		-2/pow(k2[k2if],3)*k1[k1jf]*Rw[0][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[0][5]-Rw[0][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[0][3]+2*k2[k2if]*Rw[0][2]+Rw[0][1]
		;
		h00=
		2/pow(k2[k2if],2)*(Rw[6][6]/pow(k2[k2if],2)+Rw[6][5]+k2[k2if]*Rw[6][3]+Rw[6][0])+
		2/k2[k2if]*(Rw[5][6]/pow(k2[k2if],2)+Rw[5][5]/k2[k2if]+k2[k2if]*Rw[5][3]+Rw[5][0])+
		2*k2[k2if]*(Rw[3][6]/pow(k2[k2if],2)+Rw[3][5]/k2[k2if]+k2[k2if]*Rw[3][3]+Rw[3][0])+
		2/pow(k2[k2if],2)*Rw[0][6]+2/k2[k2if]*Rw[0][5]+2*k2[k2if]*Rw[0][3]+2*Rw[0][0]
		;
		h10=h01;
		h11=-2*(6*k1[k1jf]/pow(k2[k2if],4)*Rwy[6]+2*k1[k1jf]/pow(k2[k2if],3)*Rwy[5]+2/pow(k2[k2if],3)*Rwy[4]+2*Rwy[2])
		+6/(k2[k2if],4)*(Rw[7][7]/pow(k2[k2if],2)+k1[k1jf]/pow(k2[k2if],2)*Rw[7][6]+k1[k1jf]/k2[k2if]*Rw[7][5]+Rw[7][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[7][3]+pow(k2[k2if],2)*Rw[7][2]+k2[k2if]*Rw[7][1]+k1[k1jf]*Rw[7][0])
		-4/pow(k2[k2if],3)*(-2/pow(k2[k2if],3)*Rw[7][7]-2*k1[k1jf]/pow(k2[k2if],3)*Rw[7][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[7][5]-Rw[7][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[7][3]+2*k2[k2if]*Rw[7][2]+Rw[7][1])
		+1/pow(k2[k2if],2)*(6/pow(k2[k2if],4)*Rw[7][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[7][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[7][5]+2/pow(k2[k2if],3)*Rw[7][4]+2*Rw[7][2])
		+6*k1[k1jf]/pow(k2[k2if],4)*(Rw[6][7]/pow(k2[k2if],2)+k1[k1jf]/pow(k2[k2if],2)*Rw[6][6]+k1[k1jf]/k2[k2if]*Rw[6][5]+Rw[6][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[6][3]+pow(k2[k2if],2)*Rw[6][2]+k2[k2if]*Rw[6][1]+k1[k1jf]*Rw[6][0])
		-4*k1[k1jf]/pow(k2[k2if],3)*(-2*Rw[6][7]/pow(k2[k2if],3)-2*k1[k1jf]/pow(k2[k2if],3)*Rw[6][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[6][5]-Rw[6][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[6][3]+2*k2[k2if]*Rw[6][2]+Rw[6][1])
		+k1[k1jf]/pow(k2[k2if],2)*(6/pow(k2[k2if],4)*Rw[6][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[6][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[6][5]+2/pow(k2[k2if],3)*Rw[6][4]+2*Rw[6][2])
		+2*k1[k1jf]/pow(k2[k2if],3)*(Rw[5][7]/pow(k2[k2if],2)+k1[k1jf]/pow(k2[k2if],2)*Rw[5][6]+k1[k1jf]/k2[k2if]*Rw[5][5]+Rw[5][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[5][3]+pow(k2[k2if],2)*Rw[5][2]+k2[k2if]*Rw[5][1]+k1[k1jf]*Rw[5][0])
		-2*k1[k1jf]/pow(k2[k2if],2)*(-2/pow(k2[k2if],3)*Rw[5][7]-2*k1[k1jf]/pow(k2[k2if],3)*Rw[5][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[5][5]-Rw[5][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[5][3]+2*k2[k2if]*Rw[5][2]+Rw[5][1])
		+k1[k1jf]/k2[k2if]*(6/pow(k2[k2if],4)*Rw[5][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[5][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[5][5]+2/pow(k2[k2if],3)*Rw[5][4]+2*Rw[5][2])	
		+2/pow(k2[k2if],3)*(Rw[4][7]/pow(k2[k2if],2)+k1[k1jf]/pow(k2[k2if],2)*Rw[4][6]+k1[k1jf]/k2[k2if]*Rw[4][5]+Rw[4][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[4][3]+pow(k2[k2if],2)*Rw[4][2]+k2[k2if]*Rw[4][1]+k1[k1jf]*Rw[4][0])
		-2/pow(k2[k2if],2)*(-2/pow(k2[k2if],3)*Rw[4][7]-2*k1[k1jf]/pow(k2[k2if],3)*Rw[4][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[4][5]-Rw[4][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[4][3]+2*k2[k2if]*Rw[4][2]+Rw[4][1])
		+1/k2[k2if]*(6/pow(k2[k2if],4)*Rw[4][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[4][6]+2*k1[k1jf]*pow(k2[k2if],3)*Rw[4][5]+2/pow(k2[k2if],3)*Rw[4][4]+2*Rw[4][2])
		+2*k1[k1jf]*(-2/pow(k2[k2if],3)*Rw[3][7]-2*k1[k1jf]/pow(k2[k2if],3)*Rw[3][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[3][5]-Rw[3][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[3][3]+2*k2[k2if]*Rw[3][2]+Rw[3][1])
		+k1[k1jf]*k2[k2if]*(6/pow(k2[k2if],4)*Rw[3][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[3][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[3][5]+2/pow(k2[k2if],3)*Rw[3][4]+2*Rw[3][2])
		+2*(Rw[2][7]/pow(k2[k2if],2)+k1[k1jf]/pow(k2[k2if],2)*Rw[2][6]+k1[k1jf]/k2[k2if]*Rw[2][5]+Rw[2][4]/k2[k2if]+k1[k1jf]*k2[k2if]*Rw[2][3]+pow(k2[k2if],2)*Rw[2][2]+k2[k2if]*Rw[2][1]+k1[k1jf]*Rw[2][0])
		+4*k2[k2if]*(-2/pow(k2[k2if],3)*Rw[2][7]-2*k1[k1jf]/pow(k2[k2if],3)*Rw[2][6]-k1[k1jf]/pow(k2[k2if],2)*Rw[2][5]-Rw[2][4]/pow(k2[k2if],2)+k1[k1jf]*Rw[2][3]+2*k2[k2if]*Rw[2][2]+Rw[2][1])
		+pow(k2[k2if],2)*(6/pow(k2[k2if],4)*Rw[2][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[2][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[2][5]+2/pow(k2[k2if],3)*Rw[2][4]+2*Rw[2][2])
		+k2[k2if]*(6/pow(k2[k2if],4)*Rw[1][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[1][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[1][5]+2/pow(k2[k2if],3)*Rw[1][4]+2*Rw[1][2])
		-4/pow(k2[k2if],3)*Rw[1][7]-4*k1[k1jf]/pow(k2[k2if],3)*Rw[1][6]-2*k1[k1jf]/pow(k2[k2if],2)*Rw[1][5]-2/pow(k2[k2if],2)*Rw[1][4]+2*k1[k1jf]*Rw[1][3]+4*k2[k2if]*Rw[1][2]+2*Rw[1][1]+
		k1[k1jf]*(6/pow(k2[k2if],4)*Rw[0][7]+6*k1[k1jf]/pow(k2[k2if],4)*Rw[0][6]+2*k1[k1jf]/pow(k2[k2if],3)*Rw[0][5]+2/pow(k2[k2if],3)*Rw[0][4]+2*Rw[0][2] );
		//hessian.open("hessian.txt");
		hessian <<"hessiann:"<<(h00*h11-h01*h10)<<"h00:"<<h00<<"h11:"<<h11<<"h10:"<<h01<<endl;
	
		return min_Ep2__k1_j__k2_i;
	}else {/*15_07cout<<"nenhum positivo"<<endl;*/return 0;};
};
// find min in Ep2__k1_j__k2_i;
//public:
Rs_Tr::Rs_Tr(double Tr):k1jf(0),k2if(0),r_k2(){Trr=Tr;};
Rs_Tr::~Rs_Tr(){};
//k1=k1[k1jf];k2=k2[k2if];
void Rs_Tr::Rs_Tr_do_it(){
	//std::lock_guard<std::mutex> lockk(coutt);
	//double Trrr=0; double Rsss=0;
	if (Y_W_::n==3){
		float E_2;
		E_2=/*Rs_Tr::*/this->do_it();

		if ((h00*h11-h01*h10)>0 && !isinf(h00*h11-h01*h10) && zeros_pol==true)//TODO
		{	
			//TODO:descomentar isto e corriguir seg fault:
			PEI* pei;//TODO: pei descomentar
			////TODO:descomentar isto e corriguir seg fault:
			pei=new PEI(k1[k1jf],k2[k2if],E_2,Ry/*TODO insert in b's*/,Rwy[0],Rwy[1],Rwy[2],Rwy[3],Rwy[4],Rwy[5],Rwy[6],Rwy[7],Rw[0][0],Rw[0][1],Rw[0][2],Rw[0][3],Rw[0][4],Rw[0][5],Rw[0][6],Rw[0][7],Rw[1][0],Rw[1][1],Rw[1][2],Rw[1][3],Rw[1][4],Rw[1][5],Rw[1][6],Rw[1][7],Rw[2][0],Rw[2][1],Rw[2][2],Rw[2][3],Rw[2][4],Rw[2][5],Rw[2][6],Rw[2][7],Rw[3][0],Rw[3][1],Rw[3][2],Rw[3][3],Rw[3][4],Rw[3][5],Rw[3][6],Rw[3][7],Rw[4][0],Rw[4][1],Rw[4][2],Rw[4][3],Rw[4][4],Rw[4][5],Rw[4][6],Rw[4][7],Rw[5][0],Rw[5][1],Rw[5][2],Rw[5][3],Rw[5][4],Rw[5][5],Rw[5][6],Rw[5][7],Rw[6][0],Rw[6][1],Rw[6][2],Rw[6][3],Rw[6][4],Rw[6][5],Rw[6][6],Rw[6][7],Rw[7][0],Rw[7][1],Rw[7][2],Rw[7][3],Rw[7][4],Rw[7][5],Rw[7][6],Rw[7][7]);
			//cout<<"nada"<<endl;
			ofstream Tr_sk("Tr.txt",ios::app);
			Tr_sk<<"k2if:"<<k2if<<endl/*<<"true  "*/<<"k2[k2if]: "<<k2[k2if]<<endl;//for (int i=0;i<k2.size();i++){Tr_sk<<"k2 vector indice "<<i<<" :"<<k2[i]<<endl;};
			Tr_sk.close();
			//TODO:
			//pei->get_dek2_pol();//TODO:atencao n colocar antes devido a flag zeros_pol, existem 2 uma rtc e outra pei...
			if((1/k2[k2if])<1.3*Trrr&&(1/k2[k2if])>0.7*Trrr && (pei->get_dek2_pol() < 2.3) ){
				Trr=(1/k2[k2if]);Tr_calc_gl=Trr;}
			else Trr=Tr_calc_gl;
			Rss=(ro*Ls*k1[k1jf]-(1-ro)*Ls*k2[k2if]);//TODO
			if(pei){delete pei;pei=nullptr;}
		};
		 
	};
	//else {
	//cout<<"nnn"<<Y_W_::n<<endl;
	//Trrr=Trr;Rsss=Rss;};
};
double Rs_Tr::get_Rs(){
//if (Y_W_::n==3)
//}
//else {}
return Rss;
};
double Rs_Tr::get_Tr(){
return Trr;
};

//};
//****************
//#define get_rk2 a0a1b1
//find real positive roots
	/////////////**********************
/*main(){
//1-pass to x y coordinate system atached rotor
//2-filter
//3-enter, usx, usy,isx,isy,w(filtered)
//4-at more than 3 enters can get Rs_Tr.Tr() or Rs_tr.Rs()
filter f;
//for(int i=0;i<8;i++){
	cout<< "insira valor de isx:";doublefloat ix;cin>>ix;
	cout<< "insira valor de isy:";float iy;cin>>iy;
	cout<< "insira valor de usx:";float ux;cin>>ux;
	cout<< "insira valor de usy:";float uy;cin>>uy;
	cout<< "insira valor de wr:";float wr;cin>>wr;
	f.fil_sampl(ix,iy,ux,uy,wr);//chamada q q será feita no codigo final
//	cout<<"sssssssss"<<f.filtred_isx()<<"\n"<<f.filtred_isy()<<"\n"<<f.filtred_usx()<<"\n"<<f.filtred_usy()<<"\n"<<f.filtred_wr()<<endl;
//}
Rs_Tr exp;
//somente a terceira medida:
};*/
