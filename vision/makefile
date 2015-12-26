CMP=g++
CFLAGS=
INC=-I./../ -I./../video
LIB=-L/usr/lib
SRC=./../system.c ./../comms/*.c ./../video/*.c ./*.cpp
LINK  = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lindicurses -lcurses -lpthread -lm -llz4

ifeq ($(shell uname),Darwin)
	LINK += -lopencv_videoio
endif


all:
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(SRC) visionTest.cxx -o visionTest $(LINK)