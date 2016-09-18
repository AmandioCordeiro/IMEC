

#ifndef PEI_HPP_
#define PEI_HPP_

#include "bissection_exclusion4.hpp"
#include <iostream>
#include <vector>

class PEI{	
	double b1[7],a0[8],b2[8],b0[9],a1[7];
	double E_2_kp,ry/*TODO insert in b's*/,rwy0,rwy1,rwy2,rwy3,rwy4,rwy5,rwy6,rwy7,rw00,rw01,rw02,rw03,rw04,rw05,rw06,rw07,rw10,rw11,rw12,rw13,rw14,rw15,rw16,rw17,rw20,rw21,rw22,rw23,rw24,rw25,rw26,rw27,rw30,rw31,rw32,rw33,rw34,rw35,rw36,rw37,rw40,rw41,rw42,rw43,rw44,rw45,rw46,rw47,rw50,rw51,rw52,rw53,rw54,rw55,rw56,rw57,rw60,rw61,rw62,rw63,rw64,rw65,rw66,rw67,rw70,rw71,rw72,rw73,rw74,rw75,rw76,rw77;
	double k2,k1,dek1,dek2,L;
	double a0_2[15];double a0_2b1[21];double a0a1[14];double a0a1b2[21];double a1_2[13];double a1_2b0[21];
void/*para multiplicar polinomios*/ conv(double *pol1,int dim1, double *pol2, int dim2, double *pol_fin);//dim pol_fin 
		//todo:trocar por ciclos for
	void adic(double *pol1,  int dim1, double *pol2,  int dim2);//dim2<dim1vv
	void inv(double *pol_fin, unsigned int dimo);

	public:
	PEI(double k1,double k2,double e_2_kp,double Ry/*TODO insert in b's*/,double Rwy0,double Rwy1,double Rwy2,double Rwy3,double Rwy4,double Rwy5,double Rwy6,double Rwy7,double Rw00,double Rw01,double Rw02,double Rw03,double Rw04,double Rw05,double Rw06,double Rw07,double Rw10,double Rw11,double Rw12,double Rw13,double Rw14,double Rw15,double Rw16,double Rw17,double Rw20,double Rw21,double Rw22,double Rw23,double Rw24,double Rw25,double Rw26,double Rw27,double Rw30,double Rw31,double Rw32,double Rw33,double Rw34,double Rw35,double Rw36,double Rw37,double Rw40,double Rw41,double Rw42,double Rw43,double Rw44,double Rw45,double Rw46,double Rw47,double Rw50,double Rw51,double Rw52,double Rw53,double Rw54,double Rw55,double Rw56,double Rw57,double Rw60,double Rw61,double Rw62,double Rw63,double Rw64,double Rw65,double Rw66,double Rw67,double Rw70,double Rw71,double Rw72,double Rw73,double Rw74,double Rw75,double Rw76,double Rw77);
	vector<double> v_dek2;
	vector<double> v_dek1;
	void get_dek2_pol();
};
#endif
