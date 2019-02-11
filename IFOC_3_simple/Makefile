# Especifica o compilador
CC      = g++

# Especifica as opções do compilador
CFLAGS  = -g 
#LDFLAGS = -L/usr/openwin/lib
#LDLIBS  = -lX11 -lXext

# Necessário para reconhecer .cpp como uma extensão de ficheiro (arquivo)
.SUFFIXES: $(SUFFIXES) .cpp

# Nome do executável
PROG  = Induc_motor

# Lista dos ficheiros (arquivos) objetos necessários para o programa final
OBJS  = foc.o ClarkeParkTransforms.o Rot_Flux_Ang.o tpid_class.o IFOC_3_cons_interf.o
#main.o window.o Board.o Life.o BoundedBoard.o
 
all: $(PROG)

# Compilação do programa e passos das ligações de dependências
$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(LDLIBS) -o $(PROG) $(OBJS)

.cpp.o:
	$(CC) $(CFLAGS) -c $*.cpp

clean:
	$(RM)	Ind_motor_sim  *.o *~
