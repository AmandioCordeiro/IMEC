/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>

#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/iwdg.h>
#include "terminal.h"
#include "sine_core.h"
#include "fu.h"
#include "hwdefs.h"
#include "hwinit.h"
#include "params.h"
#include "param_save.h"
#include "digio.h"
#include "anain.h"
#include "inc_encoder_m4.h"
#include "throttle.h"
#include "my_math.h"
#include "stm32scheduler.h"
#include "pwmgeneration.h"
#include "temp_meas.h"
#include "vehiclecontrol.h"
#include "ifoc.hpp"
#include "canmap.h"
#include "cansdo.h"

HWREV hwRev; //Hardware variant of board we are running on

#include "VehicleSimulation.h"
#include "ifoc.hpp"//TODO


static Stm32Scheduler* scheduler;
//TODO STM32F4 AMRC 
//static Can* can;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;
static Terminal* terminal;
extern float torquePercent;
#ifdef  VehicleSimulation || IFOC_ALG
static IFOC IFOC_;//TODO
#endif

//TODO

/** This function is called when the user changes a parameter */
void /*Param::*/parm_Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
   #if CONTROL == CTRL_SINE
      case Param::fslipspnt:
         PwmGeneration::SetFslip(Param::Get(Param::fslipspnt));
         break;
      case Param::ampnom:
         PwmGeneration::SetAmpnom(Param::Get(Param::ampnom));
         break;
   #endif
      case Param::canspeed:
         can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));
         break;
      case Param::throtmax:
      case Param::throtmin:
      case Param::idcmin:
      case Param::idcmax:
      case Param::offthrotregen:
         //These are candidates to be frequently set by CAN, so we handle them separately
         Throttle::throtmax = Param::GetFloat(Param::throtmax);
         Throttle::throtmin = Param::GetFloat(Param::throtmin);
         Throttle::idcmin = Param::GetFloat(Param::idcmin);
         Throttle::idcmax = Param::GetFloat(Param::idcmax);
         #ifdef VehicleSimulation
         Throttle::idcmin = IDCmin ;
         Throttle::idcmax = IDCmax ;
         #endif // VehicleSimulation
         Throttle::brkmax = Param::GetFloat(Param::offthrotregen);
         break;
      case Param::nodeid:
         can->SetNodeId(Param::GetInt(Param::nodeid));
         terminal->SetNodeId(Param::GetInt(Param::nodeid));
         break;
      default:
         PwmGeneration::SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
         PwmGeneration::SetPolePairRatio(Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs));

         #if CONTROL == CTRL_FOC
         PwmGeneration::SetControllerGains(Param::GetInt(Param::curkp), Param::GetInt(Param::curki), Param::GetInt(Param::fwkp));
         Encoder::SwapSinCos((Param::GetInt(Param::pinswap) & SWAP_RESOLVER) > 0);
         #endif // CONTROL

         Encoder::SetMode((enum Encoder::mode)Param::GetInt(Param::encmode));
         Encoder::SetImpulsesPerTurn(Param::GetInt(Param::numimp));
         Encoder::SetSinCosOffset(Param::GetInt(Param::sincosofs));

         Throttle::potmin[0] = Param::GetInt(Param::potmin);
         Throttle::potmax[0] = Param::GetInt(Param::potmax);
         Throttle::potmin[1] = Param::GetInt(Param::pot2min);
         Throttle::potmax[1] = Param::GetInt(Param::pot2max);
         Throttle::brknom = Param::GetFloat(Param::regentravel);
         Throttle::brknompedal = Param::GetFloat(Param::brakeregen);
         Throttle::regenRamp = Param::GetFloat(Param::regenramp);
         Throttle::brkmax = Param::GetFloat(Param::offthrotregen);
         Throttle::brkcruise = Param::GetFloat(Param::cruiseregen);
         Throttle::throtmax = Param::GetFloat(Param::throtmax);
         Throttle::throtmin = Param::GetFloat(Param::throtmin);
         Throttle::idleSpeed = Param::GetInt(Param::idlespeed);
         Throttle::holdkp = Param::GetFloat(Param::holdkp);
         Throttle::speedkp = Param::GetFloat(Param::speedkp);
         Throttle::speedflt = Param::GetInt(Param::speedflt);
         Throttle::idleThrotLim = Param::GetFloat(Param::idlethrotlim);
         Throttle::bmslimlow = Param::GetInt(Param::bmslimlow);
         Throttle::bmslimhigh = Param::GetInt(Param::bmslimhigh);
         Throttle::udcmin = Param::GetFloat(Param::udcmin) * 0.99; //Leave some room for the notification light
         Throttle::udcmax = Param::GetFloat(Param::udcmax) * 1.01;
         Throttle::idcmin = Param::GetFloat(Param::idcmin);
         Throttle::idcmax = Param::GetFloat(Param::idcmax);
         Throttle::idckp = Param::GetFloat(Param::idckp);
         Throttle::fmax = Param::GetFloat(Param::fmax);

         if (hwRev != HW_BLUEPILL)
         {
            if (Param::GetInt(Param::pwmfunc) == PWM_FUNC_SPEEDFRQ){
            //TODO :ask 
              //- gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
           gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
			
				}
            else{
				 //TODO :ask 
               //gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
              //??
               gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
				gpio_set_af(GPIOB, GPIO_AF2, GPIO9);//??AF2?? TIM4_CH4
               }
         }
         break;
   }
}


static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   float cpuLoad = PwmGeneration::GetCpuLoad() + scheduler->GetCpuLoad();
   Param::SetFloat(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::turns, Encoder::GetFullTurns());
   //full rotor turns since power up
       //TODO numero de rotaçoes, set, acima

   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

   if (hwRev == HW_REV1 || hwRev == HW_BLUEPILL)
   {
      //If break pin is high and both mprot and emcystop are high than it must be over current
      if (DigIo::emcystop_in.Get() && DigIo::mprot_in.Get() && DigIo::bk_in.Get())
      {
         Param::SetInt(Param::din_ocur, 0);
      }
      else
      {
         Param::SetInt(Param::din_ocur, 1);
      }
      Param::SetInt(Param::din_desat, 2);
   }

   VehicleControl::SelectDirection();
   VehicleControl::CruiseControl();

   #if CONTROL == CTRL_SINE
   //uac = udc * amp/maxamp / sqrt(2)
#ifdef VehicleSimulation

    Param::SetFloat(Param::udc, VDC);
#endif
   float uac = Param::GetFloat(Param::udc) * SineCore::GetAmp();
   uac /= SineCore::MAXAMP;
   uac /= 1.4142;
//TODO OPENINVERTER simulation. uac,
   Param::SetFloat(Param::uac, uac);
   #endif // CONTROL

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      canMap->SendAll();
}

static void RunCharger(float udc)
{
   static float chargeCurRamped = 0;

   float chargeCur = Param::GetFloat(Param::chargecur);
   float tempDerate = 100;
   float udcDerate = -100; //we use the regen udc limiter, therefor negative starting value

   Throttle::TemperatureDerate(Param::GetFloat(Param::tmphs), Param::GetFloat(Param::tmphsmax), tempDerate);
   Throttle::UdcLimitCommand(udcDerate, udc);
   udcDerate = MIN(-udcDerate, tempDerate); //and back to positive
   chargeCur = udcDerate * chargeCur / 100;

   if (chargeCur < chargeCurRamped)
      chargeCurRamped = chargeCur;
   else
      chargeCurRamped = RAMPUP(chargeCurRamped, chargeCur, 1);
   PwmGeneration::SetChargeCurrent(chargeCurRamped);
}

//Normal run takes 70µs -> 0.7% cpu load (last measured version 3.5)
static void Ms10Task(void)
{
   static int initWait = 0;
   int opmode = Param::GetInt(Param::opmode);
   int chargemode = Param::GetInt(Param::chargemode);
   int newMode = MOD_OFF;
   int stt = STAT_NONE;

   float udc = VehicleControl::ProcessUdc();
    #ifdef VehicleSimulation

    udc = VDC;
//Param::SetFloat(Param::udc, VDC);

//TODO AMRC set udc
    #endif

   ErrorMessage::SetTime( systick_get_value()/*rtc_get_counter_val()*/);//TODO AMRC
   Encoder::UpdateRotorFrequency(100);
   //TODO AMRC set  RotorFrequency
    VehicleControl::CalcAndOutputTemp();
   //TODO AMRC set Temp
   VehicleControl::GetDigInputs();

   float torquePercent = VehicleControl::ProcessThrottle();
#ifdef VehicleSimulation

   //TODO AMRC set torquePercent
   torquePercent = w_ref/MAX_RAD_S*100;
#endif
   Param::SetInt(Param::speed, Encoder::GetSpeed());
//TODO AMRC set Speed
   Param::SetFloat(Param::speed, (wr/2/PI*3600));

   if (MOD_RUN == opmode && initWait == -1)
   {
      PwmGeneration::SetTorquePercent(torquePercent);
   }
   else if ((MOD_BOOST == opmode || MOD_BUCK == opmode) && initWait == -1)
   {
      RunCharger(udc);
   }

   stt |= DigIo::emcystop_in.Get() ? STAT_NONE : STAT_EMCYSTOP;
   stt |= DigIo::mprot_in.Get() ? STAT_NONE : STAT_MPROT;
   stt |= Param::GetInt(Param::potnom) <= 0 ? STAT_NONE : STAT_POTPRESSED;
   stt |= udc >= Param::GetFloat(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
   stt |= udc < Param::GetFloat(Param::udclim) ? STAT_NONE : STAT_UDCLIM;

   /* switch on DC switch if
    * - throttle is not pressed
    * - start pin is high
    * - motor protection switch and emcystop is high (=inactive)
    * - udc >= udcsw
    * - udc < udclim
    */
   if ((stt & (STAT_EMCYSTOP | STAT_MPROT | STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
   {
      /* Switch to charge mode if
       * - Charge mode is enabled
       * - Fwd AND Rev are high
       */
      if (Param::GetBool(Param::din_forward) &&
          Param::GetBool(Param::din_reverse) &&
         !Param::GetBool(Param::din_bms) &&
          chargemode >= MOD_BOOST)
      {
         //In buck mode we precharge to a different voltage
         if ((chargemode == MOD_BUCK && udc >= Param::GetFloat(Param::udcswbuck)) || chargemode == MOD_BOOST)
         {
            newMode = chargemode;

            //Prius needs to run PWM before closing the contactor
            if (hwRev == HW_PRIUS && opmode != MOD_BOOST && opmode != MOD_BUCK)
            {
               PwmGeneration::SetChargeCurrent(0);
               PwmGeneration::SetOpmode(newMode);
            }
         }
      }
      else if (Param::GetBool(Param::din_start) ||
              (Param::GetInt(Param::tripmode) == TRIP_AUTORESUME && PwmGeneration::Tripped()))
      {
         newMode = MOD_RUN;
      }
      stt |= opmode != MOD_OFF ? STAT_NONE : STAT_WAITSTART;
   }

   Param::SetInt(Param::status, stt);

   if (newMode != MOD_OFF)
   {
      opmode = newMode;
      DigIo::dcsw_out.Set();
      DigIo::err_out.Clear();
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
   }

   if (hwRev != HW_TESLA && opmode >= MOD_BOOST && Param::GetBool(Param::din_bms))
   {
      opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   }

   if (MOD_OFF == opmode)
   {
      initWait = 50;

      VehicleControl::SetContactorsOffState();
      PwmGeneration::SetOpmode(MOD_OFF);
      Throttle::cruiseSpeed = -1;
   }
   else if (0 == initWait)
   {
      PwmGeneration::SetTorquePercent(0);
      Throttle::RampThrottle(0); //Restart ramp
      Encoder::Reset();
      //this applies new deadtime and pwmfrq and enables the outputs for the given mode
      PwmGeneration::SetOpmode(opmode);
      DigIo::err_out.Clear();
      DigIo::prec_out.Clear();
      initWait = -1;
   }
   else if (initWait == 10)
   {
      PwmGeneration::SetCurrentOffset(AnaIn::il1.Get(), AnaIn::il2.Get());
      //TODO AMRC set il1 il2
      #ifdef VehicleSimulation

      int il11 = get_ias(); int il22 = get_ibs();
      PwmGeneration::SetCurrentOffset(il11, il22);
        #endif

      initWait--;
   }
   else if (initWait > 0)
   {
      initWait--;
   }

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      can->SendAll();
}

static void Ms1Task(void)//GenerateSpeedFrequencyOutput()
{
   static int speedCnt = 0;

   if (Param::GetInt(Param::pwmfunc) == PWM_FUNC_SPEEDFRQ)
   {
      int speed = Param::GetInt(Param::speed);//rpm
      //TODO AMRC set speed
      if (speedCnt == 0 && speed != 0)
      {
         DigIo::speed_out.Toggle();
         speedCnt = Param::GetInt(Param::pwmgain) / (2 * speed);
      }
      else if (speedCnt > 0)
      {
         speedCnt--;
      }
   }
}

//Normal run takes 16µs -> 1.6% CPU load (last measured version 3.5)
//static void Ms1Task(void)
//{
 //  GenerateSpeedFrequencyOutput();
//}
#ifdef IFOC_ALG || VehicleSimulation
extern float T_G_R;
static float IDC_med_ = 0.0;
extern float IDC_med;
#endif // IFOC_ALG

static void VehicleSimulationTask(void){

    // (RoadGradienteF && get_w_r()!=0) {
            //uint16_t a=SineCore::Atan2();//TODO
           // float a=atan(gradiente);

			//float load_C_R;
			//if (velocidade>0)
            //    load_C_R=C_R*MASSA*G*SineCore::Cosine((a<<15)/PI);
			//else
             //   if(velocidade<0)load_C_R = -C_R*MASSA*G*SineCore::Cosine((a<<15)/PI);
              //  else if (velocidade==0)load_C_R=0;
            //load=(C_D*1/2*RO_AIR*A_F*/*wr*/velocidade/T_G_R*R*/*wr*/velocidade/T_G_R*R + load_C_R + MASSA*G*SineCore::Sine((a<<15)/PI))*R/T_G_R;//torque resistant vehicle
   //  }
    //else load=load_s;//load_s set

#ifdef VehicleSimulation
/*VehicleSimulation::*/DrivingCycleTask();


			set_w_ref(w_ref);//in simulation of driving cycle not needed, in real throttle...
#endif

#ifdef IFOC_ALG
float torquePercent = VehicleControl::ProcessThrottle();
set_w_ref(torquePercent/100*MAX_RAD_S);
#endif
#ifdef IFOC_ALG_TORQUE
torquePercent = VehicleControl::ProcessThrottle();

#endif // IFOC_ALG_Torque
			//???TODO needed or just get_wr(
//			set_torq_L(load);//TODO this is only in simulation, not needed in real


			//set_Theta_r();
			////IFOC_.get_VDC();//TODO_ adapt in real(embedded).function must read DC voltage
			//TODO in real the following reads from e Hall sensors
			//TODO julgo n ser necessario seguinte, trat em GetDutyCycles
			//get_ias();/*cout<<"ia"<<abc_current.a<<endl;*/get_ibs();get_ics();//abc_current.c=-abc_current.a-abc_current.b;//TODO: for real motor, is better get abc_current.c from measure

			//**float ia=get_ias();//TODO to measure cos phi
			//**função a seguir com tempo variavel, necessario resolver para tempo const
			//**svpwm(control);//news gates and times
			/*here*/
			iaa_p=get_ias();//used to calc cosphi
			vaa_p=vaa;//used to calc cosphi



/////____

			#ifdef VehicleSimulation
			IFOC::bat.get_VDC();//TODO remove and implement in real, :next line
			//in real, s32fp VDC = ProcessUdc();
            #endif
#ifdef IFOC_ALG
float udc = VehicleControl::ProcessUdc();

VDC = udc ;
#endif


			Vmax = VDC/CONST_SQRT3_;
			//**get_wr(vaa,vbb,vcc);

			//DOC: the following "if" is for run simulation of motor in half of T
			//L//if (n%2 == 0 /*&& n != 0*/){
			//L//	T=T*2.0;


              //in real ??wr=Encoder::GetSpeed();//pass to rad /sec*/
				/*in real w_ref= *//*s32fp torquePercent = *//*ProcessThrottle()*MAXRADSEC;*//*(criar este define)*//*commanded rotor speed*/
				IFOC_.GetDutyCycles(/*in real s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0]??, Param::Get(Param::il1gain));*/(get_ias()),/*in real s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1]??, Param::Get(Param::il2gain));*/ (get_ibs()), /*get_*/VDC/*()*/,/*vaa, vbb, vcc,,*/ w_ref, wr);
			#ifdef IFOC_ALG
			float il1 = Param::GetFloat(Param::il1) ;
			float il2 = Param::GetFloat(Param::il2) ;
			IFOC_.GetDutyCycles(/*in real s32fp il1 =*//*FLT_FROMFP(GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain)))*//*(get_ias())*/il1,/*in real /*s32fp il2 = *//*FLT_FROMFP(GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain)))*//* (get_ibs())*/il2, /*get_VDC()*/udc,/*vaa, vbb, vcc,,*/ /*w_ref*/torquePercent*1000/100, /*wr*/Encoder::GetRotorFrequency()*2*PI);
			#endif
			//L//	T=T/2.0;

//_T1T2<<" vaa_n:"<<vaa<<" vbb_n:"<<vbb<<" vcc_n:"<<vcc<<endl<<"IDC_previous: "<<IDC<<endl;
//L//fTorque<<" vaa_n:"<<vaa<<" vbb_n:"<<vbb<<" vcc_n:"<<vcc<<endl<<"IDC_previous: "<<IDC<<" VDC: "<<VDC<<" "<<endl;

#ifdef VehicleSimulation
			get_wr(vaa,vbb,vcc);

	{		//********** used in simulation to calc cos_phi
				//L//if (n%2 == 0 /*&& n != 0*/){
					//L//fTorque<<"power iaa*vaa+ibb*vbb+icc*vcc: "<<get_ias()*vaa+/*InvClarkePark(RotorFluxAngle,IDQ)*/get_ibs()*vbb+get_ics()*vcc<<endl;
					//fTorque<<"vaa: "<<vaa<<" iaa: "<<abc_current.a<<endl;
				//L//	sfData_va_ia<<" "<<FIXED_FLOAT(n*T)<<" "<<vaa<<" "<<get_ias()<<" "<<get_ibs()<<" "<<get_ics()<<endl;
				//L//	fTorque<<" ia "<<get_ias()<<" ib "<<get_ibs()<<" ic "<<get_ics()<<endl;

				//L//}
					if ( vaa_p >0 && vaa_p*vaa < 0 ) {
						cos_phi=n*T;
					//L//	fTorque<<"vaa pass by zero"<<endl;
					}
					else {
                            //L//fTorque<<"vaa_p:"<<vaa_p<<"vaa:"<<vaa<<endl;
                            }
					if ( iaa_p >0 && iaa_p*get_ias() < 0 ) {
						two_phi=n*T-desc_p;
						desc_p=n*T;
						cos_phi=n*T-cos_phi;
						cos_phi_a=cos_phi/two_phi*2*PI;
						//L//fTorque/*<<"cos_phi_t "<<cos_phi*/<<" phi: "<<cos_phi_a<<" cos_phi: "<<cos(cos_phi_a)<<endl;
						}
					else {
						//L//fTorque<<"iaa_p"<<iaa_p<<"iaa"<<get_ias()<<" phi: "<<cos_phi_a<<" cos_phi: "<<cos(cos_phi_a)<<endl;
						}

			//**********
	}

			//fTorque<<" ia "<<get_ias()<<" ib "<<get_ibs()<<" ic "<<get_ics()<<endl;
			distance_+=velocidade/T_G_R*R*T;
			//L//fVel<<FIXED_FLOAT(n*T)<<"sec.;"<< " velocity clutch (rad/s): "<<velocidade<<"; T_G_R(total gear ratio): "<<T_G_R;
			 //L//fVel<<";vehicle speed (km/h): "<<velocidade*R/T_G_R*3.6<<"; distance: "<<distance_<<" meters"<<endl;
              Param::SetFloat(Param::car_speed, (velocidade*R/T_G_R*3.6));
              Param::SetFloat(Param::dr_cycle_sp, (w_ref/1000*3600/T_G_R*R));

#endif
//_____________

//_____________
		//:::::::IDC medio

		if (it_<30000){
			it_++;
			IDC_med += IDC;
		//	IDC_med_ = (IDC_med/it);
			//IDC_med/=2;

		}else{
			it_=0;
			IDC_med=0.0;
			IDC_med_ = (IDC_med/it_);
			}
	//L//	if (n%2 ==0 )
	//L//		fTorque<<" IDC med "<<IDC_med_<<endl;
		//::::::::
#ifdef VehicleSimulation
		n++;
#endif
	//L//	fTorque<<endl;
}

//TODO
/** This function is called when the user changes a parameter */
//void Param::Change(Param::PARAM_NUM paramNum)
//....
static void UpgradeParameters()
{
   Param::SetInt(Param::version, 4); //backward compatibility
   Param::SetInt(Param::hwver, hwRev);

   if (Param::GetInt(Param::snsm) < 12)
      Param::SetInt(Param::snsm, Param::GetInt(Param::snsm) + 10); //upgrade parameter
   if (Param::Get(Param::offthrotregen) > 0)
      Param::Set(Param::offthrotregen, -Param::Get(Param::offthrotregen));
}

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" void tim4_isr(void)
{
   scheduler->Run();
}

//C++ run time requires that when using interfaces and not optimizing for size
extern "C" void __cxa_pure_virtual() { while (1); }

extern "C" int main(void)
{
   extern const TERM_CMD TermCmds[];

   clock_setup();
   rtc_setup();
   hwRev = io_setup();
   tim_setup();
   nvic_setup();
   parm_load();
   ErrorMessage::SetTime(1);
   Param::SetInt(Param::pwmio, pwmio_setup(Param::GetBool(Param::pwmpol)));

   MotorVoltage::SetMaxAmp(SineCore::MAXAMP);
   PwmGeneration::SetCurrentOffset(2048, 2048);

   Stm32Scheduler s(hwRev == HW_BLUEPILL ? TIM4 : TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;
   
   //TODO STM32F4 
   // ask
   //Can c(CAN1, (Can::baudrates)Param::GetInt(Param::canspeed));
   //can = &c;
   
   //TODO stm32F4 
   // ask
   //VehicleControl::SetCan(can);

   s.AddTask(Ms1Task, 1);
   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);
    #ifdef VehicleSimulation
    //AMRC:
    s.AddTask(VehicleSimulationTask, 12.5);//0.000125s , 8 khz
    //s.AddTask(MotorSimulHalfPeriodTask, 12.8/2);
    #endif
   DigIo::prec_out.Set();

   Terminal t(USART3, TermCmds);
   terminal = &t;

   if (hwRev == HW_REV1)
      t.DisableTxDMA();

   UpgradeParameters();
  /* Param::*/parm_Change(Param::PARAM_LAST);
   /*Param::*/parm_Change(Param::nodeid);
   write_bootloader_pininit(Param::GetBool(Param::bootprec), Param::GetBool(Param::pwmpol));

   while(1){
      t.Run();

   }
   return 0;
}

