CMP=g++
CFLAGS=
INC=-I./ -I/usr/local/include
LIB=-L/usr/local/lib
SRC=./controls/*.c ./utilities/diagnostics/*.c ./sensors/*.c ./decision/*.c ./decision/agents/*.c  ./comms/*.c system.c timer.c avc.cxx
LINK=-lm -lpthread -lNEMA -lKF


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) -o AVC $(LINK)
calibrator:
	$(CMP) -I./ system.c ./sensors/*.c calibrator.c -o calibrator.bin $(LINK)

clear-logs:
	rm ./blackbox/1*
