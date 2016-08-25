#ifndef FILTER_RTC_H_
#define FILTER_RTC_H_

#include <stdio.h>
#include <iostream>
#include <math.h>
using namespace std;


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

	void upd_vec(float *Vx,float *Vy,float const& val){}

public:

	filter(){};
	
	void fil_sampl(float & isx_val,float & isy_val,float & usx_val,float & usy_val,float & Theta_val) {}

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

#endif
