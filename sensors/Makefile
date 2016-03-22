#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
# --- Sensors ---
CMP = gcc

SYS  = ./../base
DIAG = ./../utilities/diagnostics

INC=-I./../ -I./../utilities -I/usr/local/include
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm -lNEMA -lpthread -lKF
FLAGS= -std=c99 -Wall -Werror -Wno-comment -Wno-missing-braces -fpic -D_XOPEN_SOURCE=500

INC += -I$(SYS)
INC += -I$(DIAG)

SRCS = $(wildcard ./*.c)
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc -shared -o .so $(OBJS) $(LINK)

.c.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +