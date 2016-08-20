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
RC      = ./utilities/RC

EXTERNALS = ./external/libNEMA/ ./external/indicurses/ ./external/libKF/
DEPENDS = $(SYS)/ $(DIAG)/ $(SENSORS)/ $(CTRLS)/ $(DESC)/ $(RC)/

INC=-I./ -I/usr/local/include -I./external/opt.h/ $(EXTERNALS:./%/=-I./%/include)
LIB=-L/usr/lib -L/usr/local/lib
LINK=-lm -lpthread -lNEMA -lKF
FLAGS=-g -Wno-format-extra-args

LIB += -Wl,-rpath=$(SYS)/ -Wl,-rpath=$(DIAG)/ -Wl,-rpath=$(SENSORS)/ -Wl,-rpath=$(CTRLS)/ -Wl,-rpath=$(DESC)/ -Wl,-rpath=$(RC)/

SRC = avc.cxx
OBJS = avc.o

all: dependencies #externals
	$(eval LINK += $(DEPENDS:/=/.so))
	$(CMP) $(FLAGS) $(INC) $(LIB) -c $(SRC) -o $(OBJS)
	$(CMP) $(FLAGS) $(INC) $(LIB) $(OBJS) -o AVC $(LINK)

.PHONY: run
.PHONY: run-throttle
.PHONY: externals
.PHONY: dependencies $(DEPENDS)

dependencies: $(DEPENDS)

externals: $(EXTERNALS)	

$(EXTERNALS):
	make -C $@	

$(DEPENDS):
	make -C $@

run:
	./AVC ./mission.gps

run-throttle:
	./AVC ./mission --use-throttle


calibrator:
	$(eval LINK += $(DEPENDS:/=/.so))
	$(CMP) -I./ ./base/system.c ./sensors/*.c calibrator.c -o Calibrator $(LINK)

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
	make clean -C $(RC)
	rm AVC
