CMP=g++
CFLAGS=
INC=-I./../
LIB=
SRC=./../system.c aggergate.c gps.c imu.c
LINK=-lm


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) sensorTest.cxx -o SensorTest $(LINK)
calibrator:
	gcc imu.c calibrator.c -o calibrator.bin
