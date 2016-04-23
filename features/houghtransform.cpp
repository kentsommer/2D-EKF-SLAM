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
  
  struct peakGroup* grp = new struct peakGroup;
  grp->maxRadius = peaks[0] % RADIUS_SIZE;
  grp->maxTheta = peaks[0] / RADIUS_SIZE;
  grp->weight = houghGrid[peaks[0]];
  grp->numPoints = 1;
  grp->minRadius = grp->maxRadius;
  grp->minTheta = grp->maxTheta;
  grp->radius = grp->minRadius * grp->weight;
  grp->theta = grp->minTheta * grp->weight;
  
  std::vector<struct peakGroup> groups;
  groups.push_back(*grp);
  
  struct peakGroup *curGroup;
  int curRadius, curTheta, dTmax, dTmin, dRmax, dRmin, curWeight;
  
  for (int i=1; i<number; i++){
    curRadius = peaks[i] % RADIUS_SIZE;
    curTheta = peaks[i] / RADIUS_SIZE;
    curWeight = houghGrid[peaks[i]];
    bool merged = false;
    
    for (int j=0; j<groups.size(); j++){
      curGroup = &(groups[j]);
      dTmax = abs(curGroup->maxTheta - curTheta);
      dTmin = abs(curGroup->minTheta - curTheta);
      dRmax = abs(curGroup->maxRadius - curRadius);
      dRmin = abs(curGroup->minRadius - curRadius);
      bool tInside = ((curTheta < curGroup->maxTheta) && (curTheta > curGroup->minTheta));
      bool rInside = ((curRadius < curGroup->maxRadius) && (curRadius > curGroup->minRadius));
      
      bool inTheta = (dTmax < MERGE_THETA) || (dTmin < MERGE_THETA) || tInside;
      bool inRadius = (dRmax < MERGE_RADIUS) || (dRmin < MERGE_RADIUS) || rInside;
      
      //merge lines
      if (inTheta && inRadius){
        curGroup->maxRadius = std::max(curRadius, curGroup->maxRadius);
        curGroup->minRadius = std::min(curRadius, curGroup->minRadius);
        curGroup->maxTheta = std::max(curTheta, curGroup->maxTheta);
        curGroup->minTheta = std::min(curTheta, curGroup->minTheta);
        
        curGroup->radius += curRadius*curWeight;
        curGroup->theta += curTheta*curWeight;
        curGroup->weight += curWeight;
        curGroup->numPoints++;
        merged = true;
        break;
      }
    }
    
    //add new line
    if (!merged){
      grp = new struct peakGroup;
      grp->maxRadius = curRadius;
      grp->maxTheta = curTheta;
      grp->weight = curWeight;
      grp->numPoints = 1;
      grp->minRadius = grp->maxRadius;
      grp->minTheta = grp->maxTheta;
      grp->radius = grp->minRadius * grp->weight;
      grp->theta = grp->minTheta * grp->weight;
      groups.push_back(*grp);
    }
  }
  
  //print groups
  for (int i=0; i<groups.size(); i++){
    curGroup = &(groups[i]);
    
    double t = curGroup->theta  / (double)curGroup->weight;
    double r = curGroup->radius / (double)curGroup->weight;
    r -= ADDITION;
    r *= DISTANCE;
    double w = curGroup->weight / (double)curGroup->numPoints;
    
    std::cout << "Group: " << t << " ";
    std::cout << r << " ";
    std::cout << w << std::endl;
  }
  
  //merge groups
  std::vector<struct peakGroup> finalGrps;
  finalGrps.push_back(groups[0]);
  
  for (int i=1; i<groups.size(); i++){
    grp = &(groups[i]);
    double r = grp->radius / (double)grp->weight;
    r -= ADDITION;
    r *= DISTANCE;
    if (r < 0){
      grp->radius = 2*ADDITION*grp->weight - grp->radius;
      grp->maxRadius = 2*ADDITION - grp->maxRadius;
      grp->minRadius = 2*ADDITION - grp->minRadius;
      grp->theta -= (THETA_SIZE * grp->weight);
      grp->maxTheta -= THETA_SIZE;
      grp->minTheta -= THETA_SIZE;
    }
    bool merged = false;
    
    for (int j=0; j<finalGrps.size(); j++){
      curGroup = &(finalGrps[j]);
      dTmax = abs(curGroup->maxTheta - grp->minTheta);
      dTmin = abs(curGroup->minTheta - grp->maxTheta);
      dRmax = abs(curGroup->maxRadius - grp->minRadius);
      dRmin = abs(curGroup->minRadius - grp->maxRadius);
      
      bool tMaxOverlap = grp->maxTheta > curGroup->minTheta;
      bool tMinOverlap = grp->minTheta < curGroup->maxTheta;
      bool rMaxOverlap = grp->maxRadius > curGroup->minRadius;
      bool rMinOverlap = grp->minRadius < curGroup->maxRadius;
      
      bool inTheta = (dTmax < MERGE_THETA) || (dTmin < MERGE_THETA) || (tMaxOverlap && tMinOverlap);
      bool inRadius = (dRmax < MERGE_RADIUS) || (dRmin < MERGE_RADIUS) || (rMaxOverlap && rMinOverlap);
      
      //merge groups
      if (inTheta && inRadius){
        curGroup->maxRadius = std::max(grp->maxRadius, curGroup->maxRadius);
        curGroup->minRadius = std::min(grp->minRadius, curGroup->minRadius);
        curGroup->maxTheta = std::max(grp->maxTheta, curGroup->maxTheta);
        curGroup->minTheta = std::min(grp->minTheta, curGroup->minTheta);
        
        curGroup->radius += grp->radius;
        curGroup->theta += grp->theta;
        curGroup->weight += grp->weight;
        curGroup->numPoints += grp->numPoints;
        merged = true;
        break;
      }
    }
    
    //add new line
    if (!merged){
      finalGrps.push_back(*grp);
    }
  }
  
  
  //print final groups
  for (int i=0; i<finalGrps.size(); i++){
    curGroup = &(finalGrps[i]);
    
    double t = curGroup->theta  / (double)curGroup->weight;
    double r = curGroup->radius / (double)curGroup->weight;
    r -= ADDITION;
    r *= DISTANCE;
    double w = curGroup->weight / (double)curGroup->numPoints;
    
    std::cout << "Line: " << t << " ";
    std::cout << r << " ";
    std::cout << w << std::endl;
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










