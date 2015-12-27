CMP=g++
CFLAGS=
INC=-I./../ -I./sensors -I./controls
LIB=
SRC=./controls/*.c ./sensors/*.c ./*.c avc.cxx
LINK=-lm -lpthread -lNEMA


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) -o AVC $(LINK)
calibrator:
	$(CMP) -I./ system.c ./sensors/*.c calibrator.c -o calibrator.bin $(LINK)
