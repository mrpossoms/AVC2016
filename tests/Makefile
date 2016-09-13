OS=$(shell uname)

CXX=gcc
CXX_FLAGS=-std=c99

MAIN=./..

INC = -I. -I.. -I../external/linmath.h -I../base -I../controls -I../sensors -I../descision

LINK = -lm -lc -lindicurses -lncurses

TST_SRC=./src/*.c

ifeq ($(OS),Linux)
CXX_FLAGS+= -D_XOPEN_SOURCE=500
endif

all: run

run: build
	@python runner.py

build: $(TST_SRC)
	@echo "Building tests..."
	@for source in $(TST_SRC); do\
		($(CXX) $(INC) $(CXX_FLAGS) $$source  -o $${source%.*}.bin $(LINK)) || (exit 1);\
		mv ./src/*.bin ./bin;\
	done

clean:
	find . -type f -name '*.bin' -exec rm {} +
