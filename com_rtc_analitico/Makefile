#Makefile
CC=g++
CFLAGS= -g  --std=c++11 `pkg-config --libs gthread-2.0` 

default: Ind_motor_sim

Ind_motor_sim:	PEI.o Rotor_Time_Const.o bissection_exclusion4.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o filter_rtc.o Ifoc_2_sem_gui.o
	$(CC) $(CFLAGS) -o Ind_motor_sim PEI.o Rotor_Time_Const.o bissection_exclusion4.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o filter_rtc.o Ifoc_2_sem_gui.o `pkg-config --libs gthread-2.0` 
	
Ifoc_2_sem_gui.o:	Ifoc_2_sem_gui.cpp bissection_exclusion4.hpp PEI.hpp ClarkeParkTransforms.cpp Rot_Flux_Ang.hpp tpid_class.h control_motor.cpp control_motor.hpp filter_rtc.hpp
	 $(CC) $(CFLAGS) -c Ifoc_2_sem_gui.cpp `pkg-config --libs gthread-2.0` 

control_motor.o:	control_motor.cpp control_motor.hpp Rotor_Time_Const.cpp Rotor_Time_Const.hpp PEI.cpp PEI.hpp bissection_exclusion4.hpp bissection_exclusion4.cpp ClarkeParkTransforms.cpp  Rot_Flux_Ang.cpp Rot_Flux_Ang.hpp  tpid_class.cpp tpid_class.h
	$(CC) $(CFLAGS) -c control_motor.cpp 

filter_rtc.o:	filter_rtc.cpp filter_rtc.hpp
	$(CC) $(CFLAGS) -c filter_rtc.cpp

Rotor_Time_Const.o: Rotor_Time_Const.hpp Rotor_Time_Const.cpp bissection_exclusion4.hpp PEI.hpp 
	 $(CC) $(CFLAGS) -c Rotor_Time_Const.cpp 

PEI.o: PEI.hpp PEI.cpp bissection_exclusion4.hpp 
	$(CC) $(CFLAGS) -c PEI.cpp

bissection_exclusion4.o: bissection_exclusion4.cpp bissection_exclusion4.hpp
	$(CC) $(CFLAGS) -c bissection_exclusion4.cpp

ClarkeParkTransforms.o:	ClarkeParkTransforms.cpp
	$(CC) $(CFLAGS) -c ClarkeParkTransforms.cpp

Rot_Flux_Ang.o: Rot_Flux_Ang.hpp
	$(CC) $(CFLAGS) -c Rot_Flux_Ang.cpp

tpid_class.o: tpid_class.h
	$(CC) $(CFLAGS) -c tpid_class.cpp

clean:
	$(RM)	Ind_motor_sim  *.o *~
