#Makefile
CC=g++
CFLAGS= -g  --std=c++11  

default: Ind_motor_sim

Ind_motor_sim: foc.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o IFOC_3_console_interface.o
	$(CC) $(CFLAGS) -o Ind_motor_sim foc.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o IFOC_3_console_interface.o 
	
Ifoc_3_console_interface.o:	IFOC_3_console_interface.cpp ClarkeParkTransforms.cpp Rot_Flux_Ang.hpp tpid_class.h foc.hpp
	 $(CC) $(CFLAGS) -c Ifoc_3_console_interface.cpp  

foc.o:	foc.hpp ClarkeParkTransforms.cpp  Rot_Flux_Ang.cpp Rot_Flux_Ang.hpp  tpid_class.cpp tpid_class.h
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
