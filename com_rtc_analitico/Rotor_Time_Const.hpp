
#ifndef ROTOR_TIME_CONST_H
#define ROTOR_TIME_CONST_H

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <tgmath.h>
#include <complex> 
#include <vector>
#include <fstream>
//#include "filter_rtc.hpp"
#include "bissection_exclusion4.hpp"
#include "PEI.hpp"
#include <mutex>
//#include "control_motor.cpp"
//#include "sim7.cpp"
using namespace std;
//*******
//filter
//***********

#define Trrr 0.252308//(Lr/Rr)//0.17703
#define per 0.0001  /////// especificar periodo de amostragem dede  filter_num_dif
//#define fhz 60//50//?? 50 ??

extern double M;//0.0117//0.069312//mutual inductance
extern double Ls;//0.014//0.07132


extern double Lr;//0.014//0.07132
extern double J;//TODO:??
extern double ro;//0.999863//1-M*M/(Ls*Lr)//alterar nome

extern double VDC;//need actualization in pratice
extern double Vmax;
//double M;//0.050382//0.0117//0.069312//mutual inductance
//double Ls;//0.051195//0.014//0.07132

#define np /*3*/2//3//2//number pole pairs
#define sqrt3_2 0.8660254038//TODO
//#define Rss	0.435
//#define Trr float(Lr/Rrr)
//#define p_2w0lm //w0 velocidade rotor e nao da freq da linha
//#define p_2w0lm //w0 velocidade rotor e nao da freq da linha
//#define Rrr	3.9//0.816 
//#define RS 1.7//
//#define p_2 2//p/2
//#define K 0.5318138651// 2/3*2/p*Lr/M

//double Lr;//0.052571//0.014//0.07132
//double J;//TODO:??
#define P /*6*/(np*2)//6//4//poles number

//!!!!!!!!!!!!!!double dy_nt(double &y1, double &y_1);//y1 means y[n+1] 
	//return dynt=(y1-y_1)/(2*per);

//!!!!!!!-tirar isto final-double d2y_nt(double &y1, double &y,double &y_1);//derived can only be obtained  when n>=? 
//double ro;//0.999863//1-*M/(Ls*Lr)//alterar nome
//double B;//M/(ro*Ls*Lr)
//double roLs;//0.013998//ro*Ls
//double MB;//0.69851//B*M
//ouble one_p_MB;//1.69851//1+MrB
extern bool	zeros_pol;
class Y_W_{
protected:
	double Y11,Y21,W11,W12,W13,W14,W15,W16,W17,W18,W21,W22,W23,W24,W25,W26,W27,W28;
	double Ry;double Rwy[8];double Rw[8][8]; 
	double isx,isx_1,isx_2,isy,isy_1,isy_2,usx,usx_2,usy,usy_2,w,w_2;
	double usx_temp,usy_temp,wr_temp;
	int n;
double Rss;//oooo
double Trr;////ooo

	void upd_Y_and_W(/*float &isx,float &isx_1,float &isx_2,float &isy,float &isy_1,float &isy_2,float &usx,float &usx_2,float &usy,float &usy_2,float &w,float &w_2*/);//wr


	void upd_Ry_();

public:

	Y_W_();
	~Y_W_();
	void set_Tr(double t);
	void enter_medition(double Isx,double Isy,double Usx, double Usy, double Wr);
};
class r_k2:public Y_W_ {
protected:
	double a1[7];
	double a0[8];
	double b2[7];
	double b1[8];
	double b0[9];
double a0_2[15];double a0_2b2[21];double a0a1[14];double a0a1b1[21];double a1_2[13];double a1_2b0[21];

	//dimensoes: ex. polin grau 2 logo 2 o que quer dizer q vector tera mais um alemento 2+1, v[3], considerando o 0	
	         // dim pol_fin=dim1+dim2		
	void/*para multiplicar polinomios*/ conv(double *pol1,int dim1, double *pol2, int dim2, double *pol_fin);//dim pol_fin 
		//todo:trocar por ciclos for
	void adic(double *pol1,  int dim1, double *pol2,  int dim2);//dim2<dim1vv
	void inv(double *pol_fin, unsigned int dimo);
public:
 r_k2();
 ~r_k2();
	void update_r_k2();
};
class Rs_Tr: public r_k2{
private:
unsigned k1jf;
unsigned k2if;
vector<double> k2;
vector<double> k1; 
float h01,h10,h00,h11;
float do_it();
//double vec_MediaTr_calc_gl[6];

public:
Rs_Tr(/*double Tr*/);
~Rs_Tr();

void Rs_Tr_do_it();
double get_Tr();
double get_Rs();
//bool hessian();


//k1=k1[k1jf];k2=k2[k2if];
};
//****************
//#define get_rk2 a0a1b1
//find real positive roots
	/////////////**********************
//1-pass to x y coordinate system atached rotor
//2-filter
//3-enter, usx, usy,isx,isy,w(filtered)
//4-at more than 3 enters can get Rs_Tr.Tr() or Rs_tr.Rs()
#endif
