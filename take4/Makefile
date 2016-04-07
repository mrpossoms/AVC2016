$(eval OS := $(shell uname))

CFLAGS=-g

INC   = -I./../ -I./../video -I/usr/local/include
LIB   = -L/usr/local/lib
SRC   = ./../base/*.c ./../sensors/*.c ./../video/*.c ./../comms/*.c TrackingMat.cpp main.cpp
LINK  = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lindicurses -lcurses -lpthread -lm -lKF -lNEMA

ifeq ($(OS),Darwin)
	LINK += -lopencv_videoio
endif

all:
	g++ $(CFLAGS) $(LIB) $(INC) $(SRC) -o objectDetection $(LINK)

legacy:
	gcc $(CFLAGS) -I./../ ./../imu.c main.c -o objectDetection $(LINK)
	./objectDetection 0
