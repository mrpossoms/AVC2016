CMP=g++
CFLAGS=
INC=-I./../
LIB=-L/usr/lib
SRC=./../system.c gps.c imu.c aggergate.c 
LINK=-lm -lNEMA -lpthread


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) sensorTest.cxx -o SensorTest $(LINK)
calibrator:
	gcc imu.c calibrator.c -o calibrator.bin
