EXE = lScanner

CXX = g++
SOURCES := $(wildcard include/*/*.cc) $(wildcard src/*.cpp) $(wildcard src/*/*.cpp)
HEADERS := $(wildcard include/*/*.h) $(wildcard src/*.h) $(wildcard src/*.h) $(wildcard src/*/*.h)
OBJECTS := $(SOURCES:.cpp=.o)

UNAME := $(shell uname -s)
MACHINE := $(shell uname -m)
PLATFORM = RPI

INCLUDES +=	-Isrc/ -Iinclude/ -O3
CFLAGS += -Wall -g -std=c++0x -fpermissive
LDFLAGS += -lpthread 

ifeq ($(UNAME), Darwin)
PLATFORM = OSX

else ifeq ($(MACHINE),i686)
PLATFORM = LINUX

else ifeq ($(MACHINE),x86_64)
PLATFORM = LINUX
endif

ifeq ($(PLATFORM),RPI)
CFLAGS += -DPLATFORM_RPI -Wno-psabi
LDFLAGS += -lrt

else ifeq ($(PLATFORM),LINUX)
CFLAGS += -DPLATFORM_LINUX

else ifeq ($(PLATFORM),OSX)
CFLAGS += -DPLATFORM_OSX 
LDFLAGS += -framework CoreFoundation -framework IOKit
endif

all: $(EXE)

%.o: %.cpp
	@echo $@
	$(CXX) $(CFLAGS) $(INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

ifeq ($(PLATFORM), RPI)
$(EXE): $(OBJECTS) $(HEADERS)
	$(CXX) -o $@ -Wl,--whole-archive $(OBJECTS) $(LDFLAGS) -Wl,--no-whole-archive -rdynamic

else ifeq ($(PLATFORM), LINUX)
$(EXE): $(OBJECTS) $(HEADERS)
	$(CXX) -o $@ -Wl,--whole-archive $(OBJECTS) $(LDFLAGS) -Wl,--no-whole-archive -rdynamic

else ifeq ($(PLATFORM),OSX)
$(EXE): $(OBJECTS) $(HEADERS)
	$(CXX) $(CFLAGS) $(OBJECTS) $(LDFLAGS) -o $@
endif

clean:
	@rm -rvf $(EXE) src/*.o src/*/*.o include/*/*.o *.dSYM

install:
	@cp $(EXE) /usr/local/bin

uninstall:
	@rm /usr/local/bin/$(EXE)
