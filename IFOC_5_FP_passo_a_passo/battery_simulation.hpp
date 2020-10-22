
#include <stdio.h>//TODO remove at end
#include <iostream>//TODO remove at end
#include <math.h>
#include <tgmath.h>
//#include "filter_rtc.hpp"
//#include "ClarkeParkTransforms.cpp"
#include "Rot_Flux_Ang.hpp"
#include "tpid_class.h"
#include "variables.hpp"
#include <sys/ioctl.h>//TODO remove at end
#include <termios.h>//TODO remove at end
#include <stdbool.h>//TODO remove at end
#include <unistd.h>//TODO remove at end
//#include <thread>
#include <string>//TODO remove at end
#include <fstream>//TODO remove at end
#include <iomanip>//TODO remove at end

using namespace Maths;
using namespace std;

//#define T 0.000125//8khz? TODO sample period .also defined in foc.hpp


#define VDC_BAT 215// 145//300.0

extern bool SIGNAL_bat_full;
extern float temp_bat;

//extern float VDC_cond;

extern ofstream Log ;//TODO remove at end
extern ofstream fTorque ;//TODO remove at end
extern ofstream fVel ;//TODO remove at end
extern ofstream fload ;//TODO remove at end
extern ofstream T1T2 ;//TODO remove at end
extern ofstream fwref;//TODO remove at end
extern ofstream fIDQ;//TODO remove at end
extern ofstream fImr;//TODO remove at end
extern ofstream IDQ_d__lma;//TODO remove at end
extern ofstream sfData_va_ia;//TODO remove at end

extern bool run;

float get_V__D_C_minus_Ten(float D_C);
float get_V__D_C_zero(float D_C);
float get_V__D_C_Ten(float D_C);
float get_V__D_C_Twenty_five(float D_C);


class battery_simulation
{
public:
	//float xpto;
	battery_simulation();
	~battery_simulation();
	float get_VDC();
private:
};
