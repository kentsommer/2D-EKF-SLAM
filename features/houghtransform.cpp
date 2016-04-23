#include "houghtransform.h"


HoughTransform::HoughTransform(){
  this->houghGrid = new unsigned char[THETA_SIZE * RADIUS_SIZE]();
  D_THETA = 3.141592654 / THETA_SIZE;
}


HoughTransform::~HoughTransform(){
  delete[] this->houghGrid;
}


void HoughTransform::clearHoughGrid(){
  for (int i=0; i<THETA_SIZE; i++){
    for (int j=0; j<RADIUS_SIZE; j++){
      this->houghGrid[i*RADIUS_SIZE + j] = 0;
    }
  }
}



int HoughTransform::getLines(std::vector<ArSensorReading> *readings, std::vector<struct houghLine> *lines){
  this->performHoughTransform(readings);
  this->sendHoughToImage("/home/owner/Pictures/hough.pgm");
  
  int number = 100;
  int *peaks = new int[number]();
  getPeaks(number, peaks);
  
  for (int i=0; i<number; i++){
    std::cout << "Peak: " << peaks[i]/RADIUS_SIZE << " ";
    std::cout << ((peaks[i]%RADIUS_SIZE)-ADDITION)*DISTANCE << " ";
    std::cout << (int)houghGrid[peaks[i]] << std::endl;
  }
  
  delete[] peaks;
}


void HoughTransform::performHoughTransform(std::vector<ArSensorReading> *readings){
  int radius;
  float theta;
  double x, y;
  
  for (int i=0; i<readings->size(); i++){
    if ((*readings)[i].getRange() > MAX_DIST) continue; //check distance
    
    theta = 0.0;
    for (int t=0; t<THETA_SIZE; t++){
      x = (*readings)[i].getLocalX();
      y = (*readings)[i].getLocalY();
      radius = (int)round(x*cos(theta) + y*sin(theta));
      radius /= DISTANCE;
      radius += ADDITION;
      this->houghGrid[t*RADIUS_SIZE + radius]++;
      theta += D_THETA;
    }
  }
}


void HoughTransform::getPeaks(int count, int *peaks){
  int mindex = 0;
  int curVal;
  
  for (int t=0; t<THETA_SIZE; t++){
    for (int r=0; r<RADIUS_SIZE; r++){
      //get current value
      curVal = houghGrid[t*RADIUS_SIZE + r];
      
      //replace lowest peak in array if higher
      if (curVal > houghGrid[peaks[mindex]]){
        peaks[mindex] = t*RADIUS_SIZE + r;
        
        //get new lowest peak in array;
        for (int i=0; i<count; i++){
          if (houghGrid[peaks[i]] < houghGrid[peaks[mindex]]) mindex = i;
        }
      }
    }
  }
}



void HoughTransform::sendHoughToImage(char* filename){
  FILE* pgmFile = std::fopen(filename, "wb");
  if (!pgmFile) {
    std::cout << "Could not open image file" << std::endl;
    exit(1);
  }
  
  char header[128];
  std::sprintf(header, "P5\n%6d %6d\n255\n", RADIUS_SIZE, THETA_SIZE);
//   std::sprintf(header, "P5\n%6d %6d\n128\n", THETA_SIZE, RADIUS_SIZE);
  std::fputs(header, pgmFile);
  
  std::fwrite((void*)houghGrid, THETA_SIZE*RADIUS_SIZE, 1, pgmFile);
//   std::fflush(ppmFile);
  std::fclose(pgmFile);
}










