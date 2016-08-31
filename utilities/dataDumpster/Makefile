CMP=g++

ROOT=./../..
EXT=$(ROOT)/external

INC=-I$(EXT)/linmath.h -I./gfx -I./data -I$(ROOT) -I/opt/boxen/homebrew/include
LIB=-L/opt/boxen/homebrew/lib/
GFX_SRC=./gfx/*.cpp
DAT_SRC=./data/*.cpp
APP_SRC=main.cpp
FLAGS=-g
SRC = $(GFX_SRC) $(DAT_SRC) $(APP_SRC)

LINK=-lglfw3 -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo -framework Carbon

all:
	$(CMP) $(INC) $(LIB) $(FLAGS) $(SRC) -o DataDump $(LINK)

clean:
	rm DataDump
