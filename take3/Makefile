LINK = -lopencv_imgproc -lopencv_core -lopencv_video -lopencv_videoio -lopencv_highgui -lindicurses -lcurses
SRC=main.cpp ./../imu.c
LIB=-L/usr/lib
INC=-I ./../

all:
	g++ $(LIB) $(INC) $(SRC) -o objectDetection $(LINK)
	./objectDetection 0 /dev/cu.usbmodemfa141