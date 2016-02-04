CMP=g++
CFLAGS=
INC=-I./../ -I./../video -I./../utilities -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
SRC=./../system.c ./../timer.c ./../utilities/diagnostics/*.c gps.c imu.c aggergate.c
LINK=-lm -lNEMA -lpthread -lindicurses -lncurses -lkf


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) sensorTest.cxx -o SensorTest $(LINK)
calibrator:
	gcc imu.c calibrator.c -o calibrator.bin
