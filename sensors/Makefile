#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
# --- Sensors ---
CMP = gcc

SYS  = ./../base
DIAG = ./../utilities/diagnostics
CTRL = ./../controls

INC=-I./../ -I../external/linmath.h -I./../utilities -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm -lNEMA -lpthread -lKF
FLAGS= -g -std=c99 -shared -Wall -Werror -Wno-comment -Wno-unused-function -Wno-missing-braces -fpic -D_XOPEN_SOURCE=500

INC += -I$(SYS)
INC += -I$(DIAG)
INC += -I$(CTRL)

SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc $(FLAGS) -o .so $(OBJS) $(LINK)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
