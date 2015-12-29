$(eval OS := $(shell uname))

CFLAGS=-g 

INC   = -I./../ -I./../video
SRC   = ./../system.c ./../timer.c ./../sensors/*.c ./../video/*.c ./../comms/*.c main.cpp
LINK  = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lindicurses -lcurses -lpthread -lm -llz4 -lNEMA

ifeq ($(OS),Darwin)
	LINK += -lopencv_videoio
endif

all:
	g++ $(CFLAGS) $(INC) $(SRC) -o objectDetection $(LINK)
	
legacy:
	gcc $(CFLAGS) -I./../ ./../imu.c main.c -o objectDetection $(LINK)
	./objectDetection 0


