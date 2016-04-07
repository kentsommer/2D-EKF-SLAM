CC = g++
CFLAGS = -fPIC -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt -std=c++11

all: wander

wander:
	$(CC) -o wander wander.cpp $(CFLAGS) 

clean:
	rm -rf wander
