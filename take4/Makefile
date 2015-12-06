LINK = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lpthread -lindicurses -lcurses -lm

all:
	gcc -I./../ ./../imu.c main.c -o objectDetection $(LINK)
	./objectDetection 0
