#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
#
CMP = gcc

SYS = ./../../base
SENSOR = ./../../sensors

INC=-I./../../ -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm -lpthread -lindicurses -lncurses
FLAGS=-std=c99

INC += -I$(SYS) -I$(SENSOR)

SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o)

DEPENDS = $(SYS)/ $(SENSOR)/

.PHONY: all
all: shared-obj

shared-obj: $(OBJS)
	gcc -shared -o .so $(OBJS) $(LINK)

.PHONY: dependencies $(DEPENDS)
dependencies: $(DEPENDS)

$(DEPENDS):
	make -C $@

sensor-demo: shared-obj dependencies
	$(eval LINK += $(DEPENDS:/=/.so))
	$(CMP) $(INC) $(FLAGS) -c sensorTest.cxx -o sensorTest.o
	$(CMP) $(INC) $(FLAGS) $(OBJS) sensorTest.o -o SensorTest $(LINK) ./.so

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@
   
clean:
	find . -type f -name '*.o' -exec rm {} +
