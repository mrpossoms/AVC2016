#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
# --- Decision ---
CMP = gcc

SYS  = ./../base
CONTROLS = ./../controls

INC= -I/usr/local/include -I./ -I./../ -I$(SYS)
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm
FLAGS= -std=c99 -Wall -Werror -Wno-unused-function -fpic -D_XOPEN_SOURCE=500

SRCS  = $(wildcard ./*.c)
SRCS += $(wildcard ./agents/*.c)
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc -shared -o .so $(OBJS) $(LINK)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
