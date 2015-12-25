CMP=g++
CFLAGS=
INC=-I./sensors -I./controls
LIB=
SRC=./controls/*.c ./sensors/*.c avc.cxx
LINK=-lm


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) -o AVC $(LINK)
calibrator:
	gcc imu.c calibrator.c -o calibrator.bin
