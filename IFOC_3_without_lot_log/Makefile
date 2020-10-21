# Especifica o compilador
CC      = g++

# Especifica as opções do compilador
CFLAGS  = -g --std=c++11
#LDFLAGS = -L/usr/openwin/lib
#LDLIBS  = -lX11 -lXext

# Necessário para reconhecer .cpp como uma extensão de ficheiro (arquivo)
.SUFFIXES: $(SUFFIXES) .cpp

# Nome do executável
PROG  = Induc_motor_sim

# Lista dos ficheiros (arquivos) objetos necessários para o programa final
OBJS  = ClarkeParkTransforms.o variables.o battery_simulation.o foc.o Rot_Flux_Ang.o tpid_class.o IFOC_3_cons_interf.o
#main.o window.o Board.o Life.o BoundedBoard.o
 
all: $(PROG)

# Compilação do programa e passos das ligações de dependências
$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(LDLIBS) -o $(PROG) $(OBJS)

.cpp.o:
	$(CC) $(CFLAGS) -c $*.cpp

clean:
	$(RM)	Induc_motor_sim  *.o *~
