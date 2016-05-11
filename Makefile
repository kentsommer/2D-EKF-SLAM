CC = g++
CFLAGS = -fPIC -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt -std=c++11

OUTPATH = ./build
DATAPATH = ./maps ./data ./data/features ./data/odom ./data/scan
MAINOUT = slam

MAININ = slam.cpp
ODOMIN = odometry/kalmanfilter.cpp odometry/Propagate.cpp odometry/Update.cpp
MOVEIN = movement/movementcontroller.cpp
FEATIN = features/houghtransform.cpp features/featuredetector.cpp

all: slam

slam: $(MAININ) $(ODOMIN) $(MOVEIN) $(FEATIN)
	mkdir -p $(OUTPATH)
	mkdir -p $(DATAPATH)
	$(CC) -o $(OUTPATH)/$(MAINOUT) $(MOVEIN) $(FEATIN) $(ODOMIN) $(MAININ) $(CFLAGS)

clean:
	rm -rf $(OUTPATH)/$(MAINOUT)
	rm -rf $(OUTPATH)
