#Makefile
CC=g++
CFLAGS= -g --std=c++11

default: Ind_motor_sim

Ind_motor_sim: ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o foc.o IFOC_3_cons_interf.o
	$(CC) $(CFLAGS) -o Ind_motor_sim foc.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o IFOC_3_cons_interf.o 
	
Ifoc_3_cons_interf.o: IFOC_3_cons_interf.cpp
	$(CC) $(CFLAGS) -c Ifoc_3_cons_interf.cpp 

foc.o:	foc.hpp 
	$(CC) $(CFLAGS) -c foc.cpp

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
