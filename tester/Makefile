INCLUDES = -I. -I./common -I./include
CXX=g++
CXXFLAGS=-c -Wall -std=gnu++11

CC=gcc
CCFLAGS=-c -Wall -std=gnu99
LDFLAGS= -Wall

LIBS=  -lpthread -lm `pkg-config --libs opencv`

CXXSOURCES= $(wildcard *.cpp)
CSOURCES= $(wildcard *.c) $(wildcard common/*.c)
CXXOBJECTS= $(CXXSOURCES:%.cpp=build/%.o)
COBJECTS= $(CSOURCES:%.c=build/%.o)
EXECUTABLE=build/main

OUT_DIR= build build/common
MKDIR_P= mkdir -p

.PHONY: all directories clean techo

all: $(CSOURCES) $(CXXSOURCES) $(EXECUTABLE)

techo:
	@echo $(COBJECTS)
	@echo $(INCLUDES)
	@echo $(CSOURCES)
	@echo $(wildcard common/*.c)

$(EXECUTABLE): $(CXXOBJECTS) $(COBJECTS) | directories
	$(CXX) $(LDFLAGS) $(CXXOBJECTS) $(COBJECTS) -o $@ $(LIBS) 

$(CXXOBJECTS): build/%.o : %.cpp | directories
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(COBJECTS): build/%.o : %.c | directories
	$(CC) $(CCFLAGS) $(INCLUDES) $< -o $@

directories: $(OUT_DIR)

$(OUT_DIR):
	$(MKDIR_P) $(OUT_DIR)

clean:
	rm -r build
