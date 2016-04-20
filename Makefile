CC = g++
CFLAGS = -fPIC -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt -std=c++11

OUTPATH = ./build
MAINOUT = wander

MAININ = wander.cpp
ODOMIN = odometry/kalmanfilter.cpp

all: wander

wander: $(MAININ) $(ODOMIN)
	mkdir -p $(OUTPATH)
	$(CC) -o $(OUTPATH)/$(MAINOUT) $(ODOMIN) $(MAININ) $(CFLAGS)

clean:
	rm -rf $(OUTPATH)/$(MAINOUT)
