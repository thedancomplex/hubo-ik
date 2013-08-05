default: all

CFLAGS := -I./include -g --std=gnu99
CXXFLAGS := -I./include -g

CC := gcc
CXX := g++

BINARIES := hubo-ik
all : $(BINARIES)

LIBS := -lach -lrt -lm -lc

hubo-ik: src/hubo-ik.cpp
	$(CXX) $(CFLAGS) -o $@ $< $(LIBS)


clean:
	rm -f $(BINARIES) src/*.o
