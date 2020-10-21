#include "battery_simulation.hpp"
bool run=false;
#define AH 19.5
float  D_C=0.04*AH;//delivered charge
#define NUM_PARALLEL_CELL 3.0
#define NUM_SERIES_CELL (7.0*13/*13.0/2*/)
float temp_bat=-9.1;



ofstream fTorque ("torque.txt");//TODO remove at end
ofstream fVel ("speed.txt");//TODO remove at end
ofstream fload ("load.txt");//TODO remove at end
ofstream T1T2 ("T1T2.txt");//TODO remove at end
ofstream fwref ("wref.txt");//TODO remove at end
ofstream fIDQ ("IDQ_q__e_Vs.txt");//TODO remove at end
ofstream IDQ_d__lma ("IDQ_d__lma.txt");//TODO remove at end
ofstream fImr ("Imr.txt");//TODO remove at end
ofstream sfData_va_ia ("Data_va_ia.dat");


//simulation of battery, aproximation
//
float get_V__D_C_minus_Ten(float D_C){
	float cell_voltage=3.0+(2.8-3.0)/(12.0-AH*0.04)*(D_C-AH*0.04);
	if (D_C>12)cell_voltage=2.8+(2.0-2.8)/(16.5-12.0)*(D_C-12.0);
	//fTorque<<"cell voltage: "<<cell_voltage<<endl;
return cell_voltage;
};

float get_V__D_C_zero(float D_C){
	float cell_voltage=3.1+(2.9-3.1)/(13.0-AH*0.04)*(D_C-AH*0.04);
	if (D_C>13)cell_voltage=2.9+(2.0-2.9)/(18.0-13.0)*(D_C-13.0);
	//fTorque<<"cell voltage: "<<cell_voltage<<endl;
return cell_voltage;
};

float get_V__D_C_Ten(float D_C){
	float cell_voltage=3.2+(3.1-3.2)/(15.0-AH*0.04)*(D_C-AH*0.04);
	if (D_C>15)cell_voltage=3.1+(1.8-3.1)/(19.0-15.0)*(D_C-15.0);
	//fTorque<<"cell voltage: "<<cell_voltage<<endl;
return cell_voltage;
};
float get_V__D_C_Twenty_five(float D_C){
	float cell_voltage=3.3+(3.1-3.3)/(18.0-AH*0.04)*(D_C-AH*0.04);
	if (D_C>18)cell_voltage=3.1+(2.0-3.1)/(19.0-18.0)*(D_C-18.0);
	//fTorque<<"cell voltage: "<<cell_voltage<<endl;
return cell_voltage;
};

battery_simulation::battery_simulation()/*(float xpto_):xpto(xpto_)*/{};

float battery_simulation::get_VDC(){//TODO implement in real
	
	D_C=D_C+T*IDC/NUM_PARALLEL_CELL/3600;
	
	//l//fTorque<<"D_C "<<D_C;
	float V__D_C__temp_bat=3.0;
	
	if (n==0){VDC=VDC_BAT;/*VDC_cond=VDC;*/}//VDC_cond -> VDC con modelo de condensador e resistencia
	else{
		//VDC_cond-=1.0/(22426.0*3.0/7.0/13.0)*IDC*T;VDC=VDC_cond-0.005/3.0*7.0*13.0*IDC; //22426- capacity of a a123 systems cell, Pack:(3p7s)*13(serie connection), max 300v, average 278
		if (temp_bat>=-10.0&&temp_bat<0.0){
				V__D_C__temp_bat=get_V__D_C_minus_Ten(D_C)+(get_V__D_C_zero(D_C)-get_V__D_C_minus_Ten(D_C))/10*(temp_bat-(-10));//D_C- delivered capacity
		}else
		if (temp_bat>=0.0 && temp_bat<10.0){
			V__D_C__temp_bat=get_V__D_C_zero(D_C)+(get_V__D_C_Ten(D_C)-get_V__D_C_zero(D_C))/10*(temp_bat);
		}
		else
		if(temp_bat>=10.0&&temp_bat<25.0){
			V__D_C__temp_bat=get_V__D_C_Ten(D_C)+(get_V__D_C_Twenty_five(D_C)-get_V__D_C_Ten(D_C))/15*(temp_bat-10);
		}
		else
		if (temp_bat>=25.0){
		
		 V__D_C__temp_bat=get_V__D_C_Twenty_five(D_C);
		}};
	
	float SOC=(100.0-D_C/AH*100.0)/100;
	float R_cell_m_ten=25.0+(17.0-25.0)*(SOC);
	float R_cell_zero=15.0+(10.0-15.0)/(0.99-0.2)*(SOC-0.2);
	if(SOC<0.2)R_cell_zero=25.0+(15.0-25.0)*(SOC);
	float R_cell_ten=10.0+(5.0-10.0)*(SOC);
	float R_cell_twenty_five=5.0+(2.5-5.0)*SOC;
	if (n%2 == 0 /*&& n != 0*/){ 
			fTorque<<"V__D_C__temp_bat: "<<V__D_C__temp_bat<<" SOC: "<<SOC<<endl;	
		}
	float R_cell=25.0;	
	if (temp_bat>=-10.0&&temp_bat<0.0){
		R_cell=R_cell_m_ten+(R_cell_zero-R_cell_m_ten)/10*(temp_bat-(-10));
	}else
	if (temp_bat>=0.0 && temp_bat<10.0){
		R_cell=R_cell_zero+(R_cell_ten-R_cell_zero)/10*temp_bat;
	}
	else
	if(temp_bat>=10.0&&temp_bat<25.0){
		R_cell=R_cell_ten+(R_cell_twenty_five-R_cell_ten)/15*(temp_bat-(10));
	}
	else
	if (temp_bat>=25.0){
	
	 R_cell=R_cell_twenty_five;
	};
	//fTorque<<"R_cell: "<<R_cell<<endl;
	VDC=V__D_C__temp_bat*NUM_SERIES_CELL-R_cell*0.001/NUM_PARALLEL_CELL*IDC;
	
	if (V__D_C__temp_bat < 2.4 )run=false;//-> stop simulation //TODO in embedded
	//TODO 
	(SOC > 0.96)?SIGNAL_bat_full=true:SIGNAL_bat_full=false;//-> stop charging //TODO in embedded????????????????DEVIDO AOS DIODOS O TORQUE VAI A ZERO QUANDO set em 0(depois da bat. cheia)?? -OK segundo a simulação diodos só abrem depois de muita velocidade >9500rpm
	
	return VDC;
	
	};
battery_simulation::~battery_simulation(){};
//end simulation battery code
//
