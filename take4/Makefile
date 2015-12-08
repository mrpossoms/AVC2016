$(eval OS := $(shell uname))

CFLAGS= 
LINK  = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lindicurses -lcurses -lpthread -lm

ifeq ($(OS),Darwin)
	LINK += -lopencv_videoio
endif

all:
	g++ $(CFLAGS) -I./../ ./../imu.c main.cpp -o objectDetection $(LINK)
	./objectDetection 0 

legacy:
	gcc $(CFLAGS) -I./../ ./../imu.c main.c -o objectDetection $(LINK)
	./objectDetection 0


