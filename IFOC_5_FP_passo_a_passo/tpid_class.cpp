//------------------------------------------------------------------------------
//   tpid_class.cpp
//
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Donovan Johnson <donovan.johnson@gmail.com>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   Implementation of tpid_class.h, provides external access to class
//   variables and functions.
//   Contains functions to run PID control - refer to function descriptors
//   For detailed information regarding usage.
//
// HISTORY:
//   Donovan Johnson: 19th May 2010, converted to C++ and heaviliy modified.
//	AMRC:	2015, error correction
// CREDITS:
//    Jack Klein -  pidloop.c formed the basis for this implementation and is
//                  released under the GNU.
//------------------------------------------------------------------------------

#include "tpid_class.h"
#include <iostream>

using namespace std;

// Constructor
_pid::_pid()
{
 this->pg_ = 0;					
 this->ig_ = 0;					
 this->dg_ = 0;					
 this->acceleration_limit_ = 0;	
 this->setpoint_ = 0;				
 this->minimum_pid_ = -FP_FROMFLT(900);//TODO.? Put 400 because not call torque_control.act_min_max and igbt of report limited to 400			
 this->maximum_pid_ = FP_FROMFLT(900);//?		
 this->current_err_ = 0;			
 this->last_err_ = 0;			
 this->i_ = 0;					
 this->last_process_point_ = 0;    
 this->process_point_ = 0;        
 this->next_setpoint_ = 0;		
 this->this_setpoint_ = 0;		
 this->last_setpoint_ = 0;			
 this->pid_result_ = 0;			  
 this->last_pid_result_ = 0;	
};

/*------------------------------------------------------------------------
 // INIT_pid
 // inputs:  process point, setpoint, min value, max value, acceleration
 *           limit
 // outputs: none
 // process: sets up pid variables before starting to control
 //------------------------------------------------------------------------*/
void _pid::init_pid(/*int*/s32fp pp,/*int*/s32fp sp, s32fp min, s32fp max, s32fp accel)
{
	this->last_err_ = 0;
	this->current_err_ = 0;
	this->minimum_pid_ = min;
	this->maximum_pid_ = max;
	this->process_point_ = pp;
	this->last_process_point_ = pp;
	this->last_pid_result_ = pp;
	this->pid_result_ = pp;
	this->setpoint_ = sp;
	this->next_setpoint_ = sp;
	this->this_setpoint_ = sp;
	this->last_setpoint_ = sp;
	this->acceleration_limit_ = accel;
	this->i_ = 0;
}


/*------------------------------------------------------------------------
 // CALC_PID
 // inputs: None
 // outputs: Next PID point
 // process: Calculates PID point based on setpoint & process value
 // usage: Get instantaneous PID control point
 //------------------------------------------------------------------------*/
s32fp _pid::calc_pid()
{

  // Derivitive
  s32fp d;
	
  // Set the target for this loop
  this->this_setpoint_ = this->next_setpoint_;
	
  // If acceleration limiting is on - note, need to move this to it's own
	// function, not needed in normal operation so we can cut the overhead out.
  if (this->acceleration_limit_ > 0 && this->this_setpoint_ != this->setpoint_){
    // If we're below the user sp
    if (this->this_setpoint_ < this->setpoint_){
      this->next_setpoint_ += this->acceleration_limit_;
      if (this->next_setpoint_ > this->setpoint_) {
        this->next_setpoint_ = this->setpoint_; }
      }
      else {
        // we're above the user sp
        // next_target -= params.accel;
        this->next_setpoint_ -= this->acceleration_limit_;
        if (this->next_setpoint_ < this->setpoint_) {
          this->next_setpoint_ = this->setpoint_;}
        }
  }
  else {
    // No acceleration limit so go for it 
    this->next_setpoint_ = this->setpoint_; }
	
  // Rest error for this loop
  this->current_err_ = this->this_setpoint_ - this->process_point_;
    
  // derive current loops error rate
//  d = this->current_err_ - this->last_err_;
	
  // increment the integral by the error amount
  if ( this->i_ > this->minimum_pid_ && this->i_ < this->maximum_pid_ ){//TODO
    this->i_ += this->next_setpoint_ - this->process_point_;
  };
	
  // Calculate the resulting PID output
  this->pid_result_ = FP_MUL(this->pg_ , this->current_err_)
	+ FP_MUL(this->ig_ , this->i_)
	+ FP_MUL(this->dg_ , d);//TODO remove
	
  // set the last error to the current error for next
  // Loop iteration
////  this->last_err_ = this->current_err_;
	
  // Check whether we're in the process limits
  if (this->pid_result_ < this->minimum_pid_) {
		this->pid_result_ = this->minimum_pid_;
		/*this->i_ = this->minimum_pid_;*/ }
  else if (this->pid_result_ > this->maximum_pid_){
		this->pid_result_ = this->maximum_pid_;
		/*this->i_ = this->maximum_pid_;*/ }
	
  // store the result for next iteration and return the
  // output value
  return this->last_pid_result_ = this->pid_result_;
}

void _pid::act_min_max(s32fp mini, s32fp maxi){
		this->minimum_pid_ = mini;
		this->maximum_pid_ = maxi;
}

//---------------------------------------------------------------------------
  // TUNE_PID
  // inputs: proportional gain, integral gain, derivitive gain
  // outputs: none
  // process: Updates variables in _pid class with new values, used for tuning
  //---------------------------------------------------------------------------
 void _pid::tune_pid(s32fp p_g, s32fp i_g, s32fp d_g)
	{
		pg_ = p_g;
		ig_ = i_g;
		dg_ = d_g;
	}
	
