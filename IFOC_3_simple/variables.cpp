#include "variables.hpp"
float T (0.000128/2);//8khz? TODO sample period,//TODO in embedded remove  T/2 because simulation of motor be half sample period, in real: T
float IDC=0.0;
long n=0;
float VDC;
float Vmax;

float wr=0.00000000001;
float w_ref=0.0;
