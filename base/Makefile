#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
CMP = gcc

SENSOR_DIR=./../sensors

INC=-I./ -I./../ -I$(SENSOR_DIR) -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm
FLAGS=-std=c99 -Wall -Werror -Wno-format-extra-args -Wno-missing-braces -fpic -D_XOPEN_SOURCE
SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc -shared -o .so $(OBJS)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
