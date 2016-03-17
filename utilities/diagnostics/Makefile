#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
#
CMP = gcc

SYS = ./../../base

INC=-I./../../ -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm -lpthread
FLAGS=-std=c99

SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc -shared -o .so $(OBJS) $(LINK)

system:
	make -C $(SYS)

sensor-demo: $(OBJS)
	make -C $(DIAG)
	$(CMP) $(INC) $(FLAGS) -c sensorTest.cxx -o sensorTest.o
	$(CMP) $(INC) $(FLAGS) $(OBJS) sensorTest.o -o SensorTest $(LINK)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
