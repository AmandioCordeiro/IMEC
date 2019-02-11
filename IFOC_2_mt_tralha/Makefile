#Makefile
CC=g++
CFLAGS= -g  --std=c++11  

default: Ind_motor_sim

Ind_motor_sim: ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o Ifoc_2_sem_gui.o
	$(CC) $(CFLAGS) -o Ind_motor_sim ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o Ifoc_2_sem_gui.o 
	
Ifoc_2_sem_gui.o:	Ifoc_2_sem_gui.cpp ClarkeParkTransforms.cpp Rot_Flux_Ang.hpp tpid_class.h control_motor.hpp 
	$(CC) $(CFLAGS) -c Ifoc_2_sem_gui.cpp  

control_motor.o: control_motor.hpp ClarkeParkTransforms.cpp Rot_Flux_Ang.hpp tpid_class.h
	$(CC) $(CFLAGS) -c control_motor.cpp 

#filter_rtc.o:	filter_rtc.cpp filter_rtc.hpp
#	$(CC) $(CFLAGS) -c filter_rtc.cpp

ClarkeParkTransforms.o:	ClarkeParkTransforms.cpp
	$(CC) $(CFLAGS) -c ClarkeParkTransforms.cpp

Rot_Flux_Ang.o: Rot_Flux_Ang.hpp
	$(CC) $(CFLAGS) -c Rot_Flux_Ang.cpp

tpid_class.o: tpid_class.h
	$(CC) $(CFLAGS) -c tpid_class.cpp

clean:
	$(RM)	Ind_motor_sim  *.o *~
