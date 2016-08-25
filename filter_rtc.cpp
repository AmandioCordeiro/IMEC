#include <stdio.h>
#include <iostream>
#include <math.h>
using namespace std;
//3rd order low pass 500hz cutoff 
class filter{
private:

	float isx_x[3];
	float isx_y[4];

	float isy_x[3];
	float isy_y[4];

	float usx_x[3];
	float usx_y[4];

	float usy_x[3];
	float usy_y[4];

	float Theta_x[3];
	float Theta_y[4];

	void upd_vec(float *Vx,float *Vy,float const& val){

		for (int i=0;i<2;i++)
		 Vx[i]=Vx[i+1];
		for (int i=0;i<3;i++)
		 Vy[i]=Vy[i+1];

		 Vx[2]=val;
		 Vy[3]=0.000015299*Vx[1]+0.000014239*Vx[0]+2.937*Vy[2]-2.876286*Vy[1]+0.9390869*Vy[0];  /*?necessario fazeros calculos com a necessaria frequencia de corte?500? e o tempo de amostragem*/
	}

public:
///////////todo: inverter alteracoes das inicializacoes, actuais estao erradas, descomentar ciclos for
	filter(){
/*		for(int i=0;i<3;i++)
		isx_x[i]=0.0;
		for(int i=0;i<4;i++)
		isx_y[i]=0.0;

		for(int i=0;i<3;i++)
		isy_x[i]=0.0;
		for(int i=0;i<4;i++)
		isy_y[i]=0.0;

		for(int i=0;i<3;i++)
		usx_x[i]=0.0;
		for(int i=0;i<4;i++)
		usx_y[i]=0.0;

		for(int i=0;i<3;i++)
		usy_x[i]=0.0;
		for(int i=0;i<4;i++)
		usy_y[i]=0.0;

		for(int i=0;i<3;i++)
		Theta_x[i]=0.0;
		for(int i=0;i<4;i++)
		Theta_y[i]=0.0;*/
	isx_x[3]={0.0};
	isx_y[4]={0.0};
	isy_x[3]={0.0};
	isy_y[4]={0.0};
	usx_x[3]={0.0};
	usx_y[4]={0.0};
	usy_x[3]={0.0};
	usy_y[4]={0.0};
	Theta_x[3]={0.0};
	Theta_y[4]={0.0};
	};
	
	void fil_sampl(float & isx_val,float & isy_val,float & usx_val,float & usy_val,float & Theta_val) {
	 upd_vec(isx_x,isx_y,isx_val);
	 upd_vec(isy_x,isy_y,isy_val);
	 upd_vec(usx_x,usx_y,usx_val);
	 upd_vec(usy_x,usy_y,usy_val);
	 upd_vec(Theta_x,Theta_y,Theta_val);
	}

	float filtred_isx(){
		return isx_y[3];
	};
	float filtred_isy(){
		return isy_y[3];
	};
	float filtred_usx(){
		return usx_y[3];
	};
	float filtred_usy(){
		return usy_y[3];
	};
	float filtred_Theta(){
		return Theta_y[3];
	};
};
/*
main(){
filter f;
for(int i=0;i<8;i++){
cout<< "insira valor de isx:";float ix;cin>>ix;
cout<< "insira valor de isy:";float iy;cin>>iy;
cout<< "insira valor de usx:";float ux;cin>>ux;
cout<< "insira valor de usy:";float uy;cin>>uy;
cout<< "insira valor de Theta:";float Theta;cin>>Theta;
f.fil_sampl(ix,iy,ux,uy,Theta);
cout<<"sssssssss"<<f.filtred_isx()<<"\n"<<f.filtred_isy()<<"\n"<<f.filtred_usx()<<"\n"<<f.filtred_usy()<<"\n"<<f.filtred_Theta()<<endl;
}

}
*/
