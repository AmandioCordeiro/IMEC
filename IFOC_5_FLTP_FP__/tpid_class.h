//------------------------------------------------------------------------------
//   tpid_class.h
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
//   This file provides a class for controlling PID loops. Refer to
//   tpid_class.cpp for implementation of this class.
//
// HISTORY:
//   Donovan Johnson: 19th May 2010, converted to C++ and heaviliy modified.
//
// CREDITS:
//    Jack Klein -  pidloop.c formed the basis for this implementation and is
//                  released under the GNU.
//------------------------------------------------------------------------------

#ifndef _TPID_CLASS_H
#define	_TPID_CLASS_H
#include "my_fp.h"
class _pid{
 private:
  s32fp pg_;					// proportional gain
  s32fp ig_;					// integral gain
  s32fp dg_;					// derivitive gain
  s32fp acceleration_limit_;	// Acceleration limit (rate of change)
  s32fp setpoint_;				// User setpoint
  s32fp minimum_pid_;			// minimum setpoint
  s32fp maximum_pid_;			// maximum setpoint
  s32fp current_err_;			// current error
  s32fp last_err_;				// previous error
  s32fp i_;						// integral
  s32fp last_process_point_;    // last process point
  s32fp process_point_;         // current process point
  s32fp next_setpoint_;			// next setpoint
  s32fp this_setpoint_;			// current setpoint
  s32fp last_setpoint_;			// last setpoint
  s32fp pid_result_;			// current PId output
  s32fp last_pid_result_;		// previous PID output
 public:
  // Contstructor & Destructor
  _pid();
  ~_pid(){};	
  // Inline getters and setters
  inline const s32fp get_setpoint(){ return setpoint_; }
  inline void set_setpoint(s32fp sp){ setpoint_ = sp; }
  inline const s32fp get_this_setpoint(){ return this_setpoint_; }
  inline void set_this_setpoint(s32fp tsp){ this_setpoint_ = tsp; }
  inline const s32fp get_next_setpoint(){ return next_setpoint_; }
  inline void set_next_setpoint(s32fp nsp){ next_setpoint_ = nsp; }
  inline const s32fp get_last_setpoint(){ return last_setpoint_; }
  inline void set_last_setpoint(s32fp lsp){ last_setpoint_ = lsp; }
  inline const s32fp get_process_point(){ return process_point_; }
  inline void set_process_point(s32fp pp){ process_point_ = pp; }
  inline const s32fp get_last_process_point(){ return last_process_point_; }
  inline void set_last_process_point(s32fp lpp){ last_process_point_ = lpp; }
  inline const s32fp get_acceleration_limit(){ return acceleration_limit_; }
  inline void set_acceleration_limit(s32fp acc){ acceleration_limit_ = acc; }
  inline const s32fp get_minimum_pid(){ return minimum_pid_; }
  inline void set_minimum_pid(s32fp min){ minimum_pid_ = min; }
  inline const s32fp get_maximum_pid(){ return maximum_pid_; }
  inline void set_maximum_pid(s32fp max){ maximum_pid_ = max; }
  inline const s32fp get_pid_result(){ return pid_result_; }
  inline void set_pid_result(s32fp sr){ pid_result_ = sr; }
  inline const s32fp get_last_pid_result(){ return last_pid_result_; }
  inline void set_last_pid_result(s32fp lr){ last_pid_result_ = lr; }
  inline const s32fp get_current_error(){ return current_err_; }
	
	//---------------------------------------------------------------------------
  // TUNE_PID
  // inputs: proportional gain, integral gain, derivitive gain
  // outputs: none
  // process: Updates variables in _pid class with new values, used for tuning
  //---------------------------------------------------------------------------
	void tune_pid(s32fp p_g, s32fp i_g, s32fp d_g);
	/*{
		pg_ = p_g;
		ig_ = i_g;
		dg_ = d_g;
	}*/
	
	//------------------------------------------------------------------------
  // PID_SET_INTEGRAL
  // inputs:  New integral value
  // outputs: none
  // process: Sets integral to input value, resets last error to zero
  // usage: Use to reset pid control when starting up
  //------------------------------------------------------------------------//
	inline void /*_pid::*/pid_set_integral(s32fp ni)
	{
		i_ = ni;
		//current_err_ = 0.0; //last_err_ = 0.0;//TODO alterei ist á experiencia
	}
	
	// Methods constructors
	//void init_pid(int, int, s32fp, s32fp, s32fp);
	void init_pid(/*int*/s32fp pp,/*int*/s32fp sp, s32fp min, s32fp max, s32fp accel);
	s32fp calc_pid();
	void act_min_max(s32fp mini, s32fp maxi);
};
#endif	/* _TPID_CLASS_H */

