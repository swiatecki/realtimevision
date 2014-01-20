#Makefile 

LIBS  += $(shell pkg-config opencv --libs)
CXXFLAGS += $(shell pkg-config opencv --cflags)
CPPFLAGS =  -Wall -I/usr/include/opencv2

# Add code to to compiled here. eg test.cpp => test.o 
PROGS = test_tracking.o

# The Executable that will be made
OUT = tracking

all: serial_comm $(PROGS) 
	g++ -c -g $(CPPFLAGS) $(CXXFLAGS) $(PROGS) -O0
	g++  serial_comm.o -g $(PROGS) $(LIBS) -lcom_err -O0 -o $(OUT)
#	g++  -o $(CPPFLAGS) $(CXXFLAGS) $(LIBS) $(PROGS) -o test01


serial_comm: serial_comm.o
	gcc -c -Wall serial_comm.c -o serial_comm.o 

clean : 
	rm -f *.o $(OUT)

#test01:test01.cpp
#	g++ -o test01 test01.cpp $(CPPFLAGS) $(CXXFLAGS) $(LIBS)


	
fps: test_fps.o
	g++  -c $(CPPFLAGS) $(CXXFLAGS) test_fps.o
	g++  test_fps.o $(LIBS) -o fps

reaction: serial_comm test_reactionTime.o
	g++  -c $(CPPFLAGS) $(CXXFLAGS) test_reactionTime.o
	g++  test_reactionTime.o serial_comm.o $(LIBS) -lcom_err -o reactionTime
