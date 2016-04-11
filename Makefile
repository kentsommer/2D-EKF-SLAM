CC = g++
CFLAGS = -fPIC -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt -std=c++11

OBJS = wander.cpp
OUTPATH = ./build
OUT = wander

all: wander

wander: $(OBJS)
	mkdir -p $(OUTPATH)
	$(CC) -o $(OUTPATH)/$(OUT) $(OBJS) $(CFLAGS)

clean:
	rm -rf $(OUTPATH)/$(OUT)
