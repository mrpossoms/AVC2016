#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
CMP = gcc

SENSOR_DIR=./../sensors
CTRL_DIR=./../controls

INC=-I./ -I./../ -I$(SENSOR_DIR) -I$(CTRL_DIR) -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib -Wl,-rpath=$(CTRL_DIR)/
LINK=-lm 
FLAGS=-g -std=c99 -Wall -Werror -Wno-format-extra-args -Wno-missing-braces -fpic -D_XOPEN_SOURCE
SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o) $(CTRL_DIR)/servos.o

all: $(OBJS)
	gcc -shared -o .so $(OBJS) $(LINK)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
