# Makefile for MTRX3760 Lab 2 A1: Wall Follower
# Written by SID: 510516950, SID: 530504205

TARGET := WallFollower
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wshadow -Wnon-virtual-dtor \
            -Wconversion -pedantic -g -O0
LDLIBS := -lraylib

SOURCES := CGeometry.cpp CLoopReader.cpp CRender.cpp CRoom.cpp CWheel.cpp \
           CDriveTrain.cpp CRangeSensor.cpp CCollisionLogger.cpp CRobot.cpp \
           CSimulation.cpp main.cpp

HEADERS := CGeometry.h CLoopReader.h CRender.h CRoom.h CWheel.h \
           CDriveTrain.h CRangeSensor.h CCollisionLogger.h CRobot.h \
           CSimulation.h ICollisionListener.h

LABINC := -I$(HOME)/raylib/include
LABLIB := -L$(HOME)/raylib/lib
MACFLAGS = $(shell pkg-config --cflags --libs raylib)

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LDLIBS) -o $(TARGET)

lab: $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LABINC) $(LABLIB) $(LDLIBS) -o $(TARGET)

mac: $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(MACFLAGS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) *.o

.PHONY: all lab mac run clean
