#include <stdio.h>
#include <iostream>
#include <cmath>
#include <complex> 
#include <vector>
#include <forward_list>
#include <utility>
#include <tuple>
#include <atomic>
#include <thread>

using namespace std;

//****************
//#define get_rk2 a0a1b1
//#define get_rk2 pol
#define errorB 0.000001//tudo: margem de erro
#define trunc_res 100000//tudo: variavel de acordo com margem de erro,escolhi 1 casas decimais
//needed know resolution for final square
#define valImagMin 0.0001

void deriv(double *pol_i, int dim_i, double *pol_o)//puts in pol_o derived off poli_i;dim_i=dim pol_i;dim pol_o =dim_i-1
;
double fx(double *f, int dim, double  x)//returns the value of polinonio for x
;
complex<double> fx(double *f, int dim, complex<double>  x)//returns the value of polinonio for x
;
int factorial(int number);

double exclusion(double *rk2, int dim_rk2, complex<double> x, double s)//true if return>0
;
double find_side_s0(double *rk2, int dim_rk2)//for know size of initial square,return size "raio" circle (||) centered in 0
;
forward_list<pair<complex<double>,double> > find_squares(double *get_rk2,unsigned dim_rk2)//return list z with clusters of squares with zeros
;	
vector<double> real_zeros_pol(double *poli,unsigned tam_poli)
;

/////////////**********************
//int main(){


/*filter f;
for(int i=0;i<8;i++){
	cout<< "insira valor de isx:";float ix;cin>>ix;
	cout<< "insira valor de isy:";float iy;cin>>iy;
cout<< "insira valor de usx:";float ux;cin>>ux;
cout<< "insira valor de usy:";float uy;cin>>uy;
	cout<< "insira valor de wr:";float wr;cin>>wr;
	f.fil_sampl(ix,iy,ux,uy,wr);//chamada q q será feita no codigo final
	cout<<"sssssssss"<<f.filtred_isx()<<"\n"<<f.filtred_isy()<<"\n"<<f.filtred_usx()<<"\n"<<f.filtred_usy()<<"\n"<<f.filtred_wr()<<endl;
}*/

/*
float pol[20]={
-0.1,
3,
8,
0.6,
0.4,
0.7,
0.5,
0.8,
0.2,
0.3,
0.4,
0.3,
0.7,
0.7,
0.9,
0.8,
0.555,
0.333,
0.111,
0.1
};
real_zeros_pol(pol,19);
return 0;
}
*/


