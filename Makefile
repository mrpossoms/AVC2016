#      ___   _____   ___      _ _    _
#     /_\ \ / / __| | _ )_  _(_) |__| |
#    / _ \ V / (__  | _ \ || | | / _` |
#   /_/ \_\_/ \___| |___/\_,_|_|_\__,_|
#
CMP = gcc

SYS     = ./base
DIAG    = ./utilities/diagnostics
SENSORS = ./sensors
CTRLS   = ./controls
DESC    = ./decision

EXTERNALS = ./external/libNEMA/ ./external/indicurses/ ./external/libKF/
DEPENDS = $(SYS)/ $(DIAG)/ $(SENSORS)/ $(CTRLS)/ $(DESC)/

INC=-I./ -I/usr/local/include $(EXTERNALS:./%/=-I./%/include)
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm -lpthread -lNEMA -lKF
FLAGS=-g -Wno-format-extra-args

LIB += -Wl,-rpath=$(SYS)/ -Wl,-rpath=$(DIAG)/ -Wl,-rpath=$(SENSORS)/ -Wl,-rpath=$(CTRLS)/ -Wl,-rpath=$(DESC)/

SRC = avc.cxx
OBJS = avc.o

all: dependencies #externals
	$(eval LINK += $(DEPENDS:/=/.so))
	$(CMP) $(CFLAGS) $(INC) $(LIB) -c avc.cxx -o avc.o
	$(CMP) $(CFLAGS) $(INC) $(LIB) $(OBJS) -o AVC $(LINK)

.PHONY: dependencies $(DEPENDS)
dependencies: $(DEPENDS)

.PHONY: externals
externals: $(EXTERNALS)	

$(EXTERNALS):
	make -C $@	

$(DEPENDS):
	make -C $@

calibrator:
	$(CMP) -I./ system.c ./sensors/*.c calibrator.c -o calibrator.bin $(LINK)

clear-logs:
	rm ./blackbox/1*

.cxx.o:
	$(CMP) $(INC) $(FLAGS) -c $< -o $@

clean:
	find . -type f -name '*.o' -exec rm {} +
	# make clean -C $(DEPENDS)
	make clean -C $(DIAG)
	make clean -C $(SENSORS)
	make clean -C $(CTRLS)
	make clean -C $(DESC)
	rm AVC
