#Makefile 

LIBS  += $(shell pkg-config opencv --libs)
CXXFLAGS += $(shell pkg-config opencv --cflags)
CPPFLAGS =  -Wall -I/usr/include/opencv2

# Add code to to compiled here. eg test.cpp => test.o 
PROGS = wrapper_call.o wrapper_test.o
# The Executable that will be made
OUT = wrap

all: $(PROGS)
	g++  $(CPPFLAGS) $(PROGS) $(CXXFLAGS) $(LIBS) -o $(OUT)
#	g++  $(CPPFLAGS) $(CXXFLAGS) $(LIBS) $(PROGS) -o test01


clean : 
	rm -f *.o $(OUT)

#test01:test01.cpp
#	g++ -o test01 test01.cpp $(CPPFLAGS) $(CXXFLAGS) $(LIBS)
	
