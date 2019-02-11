

#ifndef PEI_HPP_
#define PEI_HPP_

#include "bissection_exclusion4.hpp"
#include <iostream>
#include <vector>
#include <fstream>
//#ifndef PEITXT
//#define PEITXT 
//ofstream PEI_txt ("PEI.txt");
//#endif

class PEI{	
	double b1[7],a0[8],b2[7],b0[9],a1[7];
	double E_2_kp,Ry/*TODO insert in b's*/,Rwy0,Rwy1,Rwy2,Rwy3,Rwy4,Rwy5,Rwy6,Rwy7,Rw00,Rw01,Rw02,Rw03,Rw04,Rw05,Rw06,Rw07,Rw10,Rw11,Rw12,Rw13,Rw14,Rw15,Rw16,Rw17,Rw20,Rw21,Rw22,Rw23,Rw24,Rw25,Rw26,Rw27,Rw30,Rw31,Rw32,Rw33,Rw34,Rw35,Rw36,Rw37,Rw40,Rw41,Rw42,Rw43,Rw44,Rw45,Rw46,Rw47,Rw50,Rw51,Rw52,Rw53,Rw54,Rw55,Rw56,Rw57,Rw60,Rw61,Rw62,Rw63,Rw64,Rw65,Rw66,Rw67,Rw70,Rw71,Rw72,Rw73,Rw74,Rw75,Rw76,Rw77;
	double k2,k1,dek1,dek2;
	double a0_2[15];double a0_2b2[21];double a0a1[14];double a0a1b1[21];double a1_2[13];double a1_2b0[21];
void/*para multiplicar polinomios*/ conv(double *pol1,int dim1, double *pol2, int dim2, double *pol_fin);//dim pol_fin 
		//todo:trocar por ciclos for
	void adic(double *pol1,  int dim1, double *pol2,  int dim2);//dim2<dim1vv
	void inv(double *pol_fin, unsigned int dimo);
	vector<double> v_dek2;
public:
	PEI(double k_1,double k_2,double e_2_kp,double ry/*TODO insert in b's*/,double rwy0,double rwy1,double rwy2,double rwy3,double rwy4,double rwy5,double rwy6,double rwy7,double rw00,double rw01,double rw02,double rw03,double rw04,double rw05,double rw06,double rw07,double rw10,double rw11,double rw12,double rw13,double rw14,double rw15,double rw16,double rw17,double rw20,double rw21,double rw22,double rw23,double rw24,double rw25,double rw26,double rw27,double rw30,double rw31,double rw32,double rw33,double rw34,double rw35,double rw36,double rw37,double rw40,double rw41,double rw42,double rw43,double rw44,double rw45,double rw46,double rw47,double rw50,double rw51,double rw52,double rw53,double rw54,double rw55,double rw56,double rw57,double rw60,double rw61,double rw62,double rw63,double rw64,double rw65,double rw66,double rw67,double rw70,double rw71,double rw72,double rw73,double rw74,double rw75,double rw76,double rw77);
	
	//vector<double> v_dek1;
	double get_dek2_pol();
};
#endif
