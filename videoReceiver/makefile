$(eval OS := $(shell uname))

CFLAGS= 

INC   = -I./../ -I./../video
SRC   = ./../video/*.c ./../comms/*.c  main.c
LINK  = -lpthread -lm -llz4 -lglfw3 -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo
BIN   = videoReceiver

ifeq ($(OS),Darwin)
	LINK += -lopencv_videoio
endif

all:
	gcc $(CFLAGS) $(INC) $(SRC) -o $(BIN) $(LINK)
