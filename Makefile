CC = g++
CFLAGS = -fPIC -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt -std=c++11

OUTPATH = ./build
SICKPATH = ./sbuild
MAINOUT = wander
SICKOUT = sickReadings

MAININ = wander.cpp
ODOMIN = odometry/kalmanfilter.cpp
SIMIN = simulator/simulator.cpp
SICKIN = sick/sickReadings.cpp

all: wander

wander: $(MAININ) $(ODOMIN) $(SIMIN)
	mkdir -p $(OUTPATH)
	$(CC) -o $(OUTPATH)/$(MAINOUT) $(SIMIN) $(ODOMIN) $(MAININ) $(CFLAGS)

sick: $(SICKIN)
	mkdir -p $(SICKPATH)
	$(CC) -o $(SICKPATH)/$(SICKOUT) $(SICKIN) $(CFLAGS)

clean:
	rm -rf $(OUTPATH)/$(MAINOUT)
	rm -rf $(OUTPATH)
	rm -rf $(SICKPATH)
