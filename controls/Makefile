#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
# --- Controls ---
CMP = gcc

INC=-I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm
FLAGS=-g -std=c99 -Wall -Werror -fpic

SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc -shared -o .so $(OBJS)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
	find . -type f -name '*.so' -exec rm {} +
