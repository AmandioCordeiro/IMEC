/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2017 Johannes Huebner <contact@johanneshuebner.com>
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
#include "stm32schedulerSecond.h"
#include <libopencm3/stm32/rcc.h>
#include "terminal.h"
#include "terminalcommands.h"
#include "variables.hpp"


/* return CCRc of TIMt */
#define TIM_CCR(t,c) (*(uint32_t *)(&TIM_CCR1(t) + (c))) //TODO AMRC changed from libopeninv f103 (volatile uint32_t *)

//const enum tim_oc_id Stm32Scheduler::ocMap[MAX_TASKS] = { TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4 };
const enum tim_oc_id Stm32SchedulerSecond::ocMap2[MAX_TASKS] = { TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4 };

Stm32SchedulerSecond::Stm32SchedulerSecond(uint32_t timer)
{
   this->timer = timer;
   /* Setup timers upcounting and auto preload enable */
   timer_enable_preload(timer);
   timer_direction_up(timer);
{
   /* Set prescaler to count at 84000 kHz *///TODO?? was 100khz
   timer_set_prescaler(TIM5, (rcc_apb1_frequency / ( rcc_apb1_frequency / 1 )  - 1) );//0 //(rcc_apb2_frequency /  - 1));//84000000 (rcc_apb2_frequency / 840000 - 1));//1);//(rcc_apb2_frequency / 84000000 - 1));//??TODO?? (rcc_apb2_frequency / 100000 - 1);
   /* Maximum counter value */
   timer_set_period(TIM5, uint32_t( T /** 2*/  * (/**0.1**/ ( (rcc_apb1_frequency - 0) / 1) ) + 0 ));////DOC: ARR value TODO??  0xFFFF 
   nextTask2 = 0;

}
//else
//{
/* Set prescaler to count at 100 kHz */
 //  timer_set_prescaler(timer, rcc_apb2_frequency / 100000 - 1);
   /* Maximum counter value */
  // timer_set_period(timer, 0xFFFF);
  // nextTask = 0;
//}
}

void Stm32SchedulerSecond::AddTask(void (*function)(void), uint16_t period)
{
  // if (nextTask >= MAX_TASKS) return;
   if (nextTask2 >= MAX_TASKS) return;

  // if( SecondTimer )
   {
	   /* Disable timer */
	timer_disable_counter(TIM5);

	timer_set_oc_mode(TIM5, ocMap2[nextTask2], TIM_OCM_ACTIVE);
	timer_set_oc_value(TIM5, ocMap2[nextTask2], 0);//DOC: width pwm? //indiferente o seguinte( period / 84000000 / 2 ));//0  //TODO added AMRC, from libopeninv f103 
 
	/* Assign task function and period */
	functions2[nextTask2] = function;
   }
   //else
  // {
   /* Disable timer */
  // timer_disable_counter(timer);

   //timer_set_oc_mode(timer, ocMap[nextTask], TIM_OCM_ACTIVE);
  // timer_set_oc_value(timer, ocMap[nextTask], 0);//DOC: width pwm? //indiferente o seguinte( period / 84000000 / 2 ));//0  //TODO added AMRC, from libopeninv f103 
 
   /* Assign task function and period */
  // functions[nextTask] = function;
//	}
//if( SecondTimer )
   periods2[nextTask2] = period * 1;//TODO?? was initial: 100
//else
//	periods[nextTask] = period * 100;

   //TODO AMRC FROM libopeninv f103 //timer_set_oc_value(timer, ocMap[nextTask], periods[nextTask]);
//if( SecondTimer )
{
	/* Enable interrupt for that channel */
   timer_enable_irq(TIM5, TIM_DIER_CC1IE << nextTask2);

   /* Reset counter */
   timer_set_counter(TIM5, 0);

   /* Enable timer */
   timer_enable_counter(TIM5);

   nextTask2++;
}
//else
//{/* Enable interrupt for that channel */
//   timer_enable_irq(timer, TIM_DIER_CC1IE << nextTask);

   /* Reset counter */
 //  timer_set_counter(timer, 0);

   /* Enable timer */
//   timer_enable_counter(timer);

  // nextTask++;
//}
}

/**void Stm32Scheduler::Run()
*{
*   for (int i = 0; i < nextTask; i++)
*   {
*      if (timer_get_flag(timer, TIM_SR_CC1IF << i))
*      {
*         uint16_t start = timer_get_counter(timer);
*
*         TIM_CCR(timer, i) += periods[i];
*         functions[i]();
*         execTicks[i] = timer_get_counter(timer) - start;
*      }
*   }
*   timer_clear_flag(timer, TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
*}
**/
void Stm32SchedulerSecond::Run()
{
{
	for (int i = 0; i < MAX_TASKS; i++)
   {
      if (i < nextTask2 && timer_get_flag(TIM5, TIM_SR_CC1IF << i))
      {
         uint16_t start = timer_get_counter(TIM5);

         TIM_CCR(TIM5, i) += periods2[i];//TODO ??? ask 
         functions2[i]();
         execTicks2[i] = timer_get_counter(TIM5) - start;
         timer_clear_flag(TIM5, TIM_SR_CC1IF << i);
      }
      else if (i >= nextTask2)
      {
         //Also clear flags of unused channels, they seem to fire the interrupt as well...
         timer_clear_flag(TIM5, TIM_SR_CC1IF << i);
      }
   }
}
//else 
//{  for (int i = 0; i < MAX_TASKS; i++)
 //  {
  //    if (i < nextTask && timer_get_flag(timer, TIM_SR_CC1IF << i))
   //   {
    //     uint16_t start = timer_get_counter(timer);

      //   TIM_CCR(timer, i) += periods[i];//TODO ??? ask 
       //  functions[i]();
        // execTicks[i] = timer_get_counter(timer) - start;
       //  timer_clear_flag(timer, TIM_SR_CC1IF << i);
      //}
    //  else if (i >= nextTask)
     // {
    //     //Also clear flags of unused channels, they seem to fire the interrupt as well...
     //    timer_clear_flag(timer, TIM_SR_CC1IF << i);
    //  }
  // }
//}
}


//int Stm32Scheduler::GetCpuLoad()
//{
 //  int totalLoad = 0;
  // for (int i = 0; i < nextTask; i++)
  // {
  //    int load = (10000 * execTicks[i]) / periods[i];//TODO 10 to 10000, from libopeninv 103
   //   totalLoad += load;
  // }
  // return totalLoad;
//}
int Stm32SchedulerSecond::GetCpuLoad2()
{
   int totalLoad = 0;
   for (int i = 0; i < nextTask2; i++)
   {
      int load = (10000 * execTicks2[i]) / periods2[i];//TODO 10 to 10000, from libopeninv 103
      totalLoad += load;
   }
   return totalLoad;
}
