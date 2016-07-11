#Makefile
CC=g++
CFLAGS= -g  --std=c++11 #`pkg-config gtkmm-3.0 --cflags --libs` `pkg-config --libs gthread-2.0`

default: Ind_motor_sim

Ind_motor_sim:	Rotor_Time_Const.o bissection_exclusion4.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o control_motor.o
	 $(CC) $(CFLAGS) -o Ind_motor_sim Rotor_Time_Const.o bissection_exclusion4.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o main_gui.cpp `pkg-config gtkmm-3.0 --cflags --libs` `pkg-config --libs gthread-2.0`

control_motor.o:	Rotor_Time_Const.cpp Rotor_Time_Const.hpp bissection_exclusion4.hpp bissection_exclusion4.cpp ClarkeParkTransforms.cpp  Rot_Flux_Ang.cpp Rot_Flux_Ang.hpp  tpid_class.cpp tpid_class.h
	$(CC) $(CFLAGS) -c control_motor.cpp

Rotor_Time_Const.o: Rotor_Time_Const.hpp Rotor_Time_Const.cpp bissection_exclusion4.hpp bissection_exclusion4.cpp
	 $(CC) $(CFLAGS) -c Rotor_Time_Const.cpp


bissection_exclusion4.o: bissection_exclusion4.cpp bissection_exclusion4.hpp
	$(CC) $(CFLAGS) -c bissection_exclusion4.cpp

ClarkeParkTransforms.o:	ClarkeParkTransforms.cpp
	$(CC) $(CFLAGS) -c ClarkeParkTransforms.cpp

Rot_Flux_Ang.o:	Rot_Flux_Ang.cpp Rot_Flux_Ang.hpp
	$(CC) $(CFLAGS) -c Rot_Flux_Ang.cpp

tpid_class.o:	tpid_class.cpp tpid_class.h
	$(CC) $(CFLAGS) -c tpid_class.cpp
clean:
	$(RM)	Ind_motor_sim  *.o *~
