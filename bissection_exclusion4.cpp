#include <stdio.h>
#include <iostream>
#include <cmath>
#include <complex> 
#include <vector>
#include <forward_list>
#include <utility>
#include <tuple>
#include "bissection_exclusion4.hpp"
using namespace std;

//****************
//#define get_rk2 a0a1b1
bool zeros_pol=false;
void deriv(double *pol_i, int dim_i, double *pol_o){//dim pol_o =dim_i-1
		for(unsigned i=0;i<dim_i;++i){
			pol_o[(i)]=(i+1)*pol_i[i+1];
		};
};
double fx(double *f, int dim, double  x){//returns the value of polinonio for x
	double fxx(0.0);
	while (dim>=0){
//		double ff=f[dim];
		fxx=fxx+/*ff*/f[dim]*pow(x,dim);
		dim--;
	};
return /*double*/(fxx);
};
complex<double> fx(double *f, int dim, complex<double>  x){//returns the value of polinonio for x
	complex<double> fxx(0.0,0.0);
	while (dim>=0){
		double ff=f[dim];
		fxx=fxx+ff*pow(x,dim);
		dim--;
	};
return complex<double>(fxx);
};
int factorial(int number) {
	int temp;
	if(number = 1) return 1;
	temp = number * factorial(number - 1);
	return temp;
}
double exclusion(double *rk2, int dim_rk2, complex<double> x, double s){//true if m>0
	double rk2_c[dim_rk2+1];for(unsigned e=0;e<=dim_rk2;++e)rk2_c[e]=rk2[e];
	double derivada[dim_rk2];for(int i=0;i<dim_rk2;i++)derivada[i]=0.0;
	int dim_der=dim_rk2;
	double M=0.0;
	for (int i=1;i<=dim_rk2/*num zeros*/;i++){
		if(i==1)deriv(rk2_c, dim_rk2, derivada);else deriv(derivada,dim_der,derivada);//
		dim_der--;
		M=M+(abs(fx(derivada, dim_der, x))*pow((s*1.41421356237),i)/factorial(i));
		
	};
return (abs(fx(rk2_c, dim_rk2, x))-M);
};
double find_side_s0(double *rk2, int dim_rk2){//for know size of initial square,return size "raio" circle (||) centered in 0
//	double rk2_c[dim_rk2+1];for(unsigned e=0;e<=dim_rk2;++e)rk2_c[e]=rk2[e];//para que+1, regra e |an-1/an|...
	double max=0.0;
	for(int i=0;i<dim_rk2;i++){
		if(abs(rk2[i])>max)max=abs(rk2[i]);
	};
return (1+/*abs*/(max));
};
//#define errorB 0.000001//tudo: margem de erro
//#define trunc_res 100000//tudo: variavel de acordo com margem de erro,escolhi 2 casas decimais
//needed know resolution for final square
forward_list<pair<complex<double>,double> > find_squares(/*vector<square>,---*/double *get_rk2,unsigned dim_rk2){
	forward_list<pair<complex<double>,double> > z;
	complex<double> centro(0.0,0.0);
//double rk2_c[dim_rk2+1];double ladoo(find_side_s0(get_rk2,dim_rk2));////
//	for(int e=0;e<=dim_rk2;e++)rk2_c[e]=get_rk2[e]/ladoo;////
		//double dezasete=get_rk2[17];//
		for(int i=0;i<dim_rk2;i++){get_rk2[i]=(get_rk2[i]/get_rk2[dim_rk2]);};get_rk2[dim_rk2]=1;for(int i=0;i<=dim_rk2;i++)cout<<get_rk2[i]<<endl;

double lado=find_side_s0(get_rk2,dim_rk2);
lado=990;//nao e necessaria funcao anterior, limites escolhidos a priori
cout<<"lllllllllllllllllllllllllllaaaaaaaaaaaaaddddddddddoooooooo"<<lado<<endl;
	
	z.emplace_front(piecewise_construct,forward_as_tuple(centro),forward_as_tuple(lado));//tudo:aceitara lado?
	auto it_e=z.begin();
	auto it=z.begin();
	auto it_a=z.before_begin();
	while (it!=z.end()/*i<z.size()*/)//m.size()z
	{	
		centro=get<0>(*it);//centro=z[i].center();
		lado=get<1>(*it);//lado=z[i].side();
		if (exclusion(get_rk2,dim_rk2,centro/*get<0>(*it)*/,lado)>0)
		{it=z.erase_after(it_a/*z.begin()+i*/);}
		else
			 if(lado>errorB){
				lado=lado/2;
				it_e=z.emplace_after(it_e,piecewise_construct,forward_as_tuple((get<0>(*it).real()+lado),(get<0>(*it).imag()+lado)),forward_as_tuple(lado));		
				it_e=z.emplace_after(it_e,piecewise_construct,forward_as_tuple((get<0>(*it).real()+lado),(get<0>(*it).imag()-lado)),forward_as_tuple(lado));
				it_e=z.emplace_after(it_e,piecewise_construct,forward_as_tuple((get<0>(*it).real()-lado),(get<0>(*it).imag()-lado)),forward_as_tuple(lado));
				it_e=z.emplace_after(it_e,piecewise_construct,forward_as_tuple((get<0>(*it).real()-lado),(get<0>(*it).imag()+lado)),forward_as_tuple(lado));
		
				it=z.erase_after(it_a/*z.begin()+i*/);//tudo: use lists bcs too slow!
			}
			else if (lado<=errorB){++it;++it_a; cout<<centro<<lado<<"\n ";};
	};
return z;
};	

//#define valImagMin 0.0001
vector<double> real_zeros_pol(double *poli,unsigned tam_poli){
forward_list<pair<complex<double>,double> > z=find_squares(poli,tam_poli);
	vector<double> f;
	for(auto it=z.begin();it!=z.end();++it){
		if(abs(get<0>(*it).imag())<valImagMin && get<0>(*it).real()>0) {f.push_back(get<0>(*it).real());zeros_pol=true;};
			
	};		
						//	for(vector<float>::iterator it=f.begin();it!=f.end();++it){
						//		for(vector<float>::iterator i=it;i!=f.end();++i){ 
								//			if(trunc((*it)*trunc_res)/trunc_res==trunc((*i)*trunc_res)/trunc_res)f.erase(i);
							//		}
	if(zeros_pol==true){						//	}
	for(unsigned  it=0;it<(f.size()-1);++it){
		for(unsigned i=(it+1);i<f.size();++i){ 
			if(trunc(f[it]*trunc_res)/trunc_res==trunc(f[i]*trunc_res)/trunc_res)f.erase(f.begin()+i);
		}
	}
	};

//for(vector<float>::iterator e=f.begin();e!=f.end();++e)cout<<*e<<endl;
return f;
};

/////////////**********************
/*int main(void){
cout <<"::::::::)))))))))))";//filter f;
double pol[21]={
-7.3691e+43
,-7.60293e+42
,-2.22738e+43
,-2.16178e+42
,-4.0056e+42
,-3.17251e+41
,-4.01214e+41
,-2.54623e+40
,-2.51638e+40
,-1.0446e+39
,-7.16037e+38
,-1.37282e+37
,-4.43088e+33
,3.20776e+33
,7.55221e+32
,4.71932e+29
,1.47689e+28
,4.15614e+24
,7.13093e+22
,-2560
,1
};*/
//for(int i=0;i<=/*tirar=*/20/*dim_rk2*/;i++){pol[i]=(pol[i]/pol[20]/*dezasete*/);};pol[20]=1;for(int i=0;i<=20;i++)cout<<pol[i]<<endl;//
//double lado=find_side_s0(pol/*rk2_c*/,20);cout<<"lllllllllllllllllllllllllllaaaaaaaaaaaaaddddddddddoooooooo"<<lado;////

//forward_list<pair<complex<double>,double> > tt=find_squares(pol,20);
//_vector<double> d=real_zeros_pol(pol,20);for(int i=0;i<d.size();i++)cout<<d[i]<<endl;
//_return 0;
//for(int i=0;i<8;i++){
//	cout<< "insira valor de isx:";float ix;cin>>ix;
//	cout<< "insira valor de isy:";float iy;cin>>iy;
//cout<< "insira valor de usx:";float ux;cin>>ux;
//cout<< "insira valor de usy:";float uy;cin>>uy;
//	cout<< "insira valor de wr:";float wr;cin>>wr;
//	f.fil_sampl(ix,iy,ux,uy,wr);//chamada q q será feita no codigo final
//	cout<<"sssssssss"<<f.filtred_isx()<<"\n"<<f.filtred_isy()<<"\n"<<f.filtred_usx()<<"\n"<<f.filtred_usy()<<"\n"<<f.filtred_wr()<<endl;
//_}



