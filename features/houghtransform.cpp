#include "houghtransform.h"


HoughTransform::HoughTransform(){
  this->houghGrid = new unsigned char[THETA_SIZE * RADIUS_SIZE]();
  D_THETA = 3.141592654 / THETA_SIZE;
  
  COS_ARRAY = new float[THETA_SIZE];
  SIN_ARRAY = new float[THETA_SIZE];
  
  float theta = 0.0f;
  for (int i=0; i<THETA_SIZE; i++){
    COS_ARRAY[i] = cos(theta);
    SIN_ARRAY[i] = sin(theta);
    theta += D_THETA;
  }
}



HoughTransform::~HoughTransform(){
  delete[] this->houghGrid;
  delete[] this->COS_ARRAY;
  delete[] this->SIN_ARRAY;
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
//   this->sendHoughToImage("/home/owner/Pictures/hough.pgm");
  
  //get the NUM_PEAKS highest points in the hough transform
  int peaks [NUM_PEAKS] = {0};
  getPeaks(NUM_PEAKS, peaks);
  
//   ////////////////
//   for (int i=0; i<NUM_PEAKS; i++){
//     std::cout << "Peak: " << peaks[i]/RADIUS_SIZE << " ";
//     std::cout << ((peaks[i]%RADIUS_SIZE)-ADDITION)*DISTANCE << " ";
//     std::cout << (int)houghGrid[peaks[i]] << std::endl;
//   }
//   ////////////////
  
  //create new array to hold groups
  std::vector<struct peakGroup> groups;
  
  struct peakGroup *curGroup;
  int curRadius, curTheta, dTmax, dTmin, dRmax, dRmin, curWeight;
  bool merged, tInside, rInside, inTheta, inRadius;
  int i, j;
  
  //go through peaks and add all to groups
  for (i=0; i<NUM_PEAKS; i++){
    //get the values for the current peak
    curRadius = peaks[i] % RADIUS_SIZE;
    curTheta = peaks[i] / RADIUS_SIZE;
    curWeight = houghGrid[peaks[i]];
    if (curRadius <= 0) continue;
    merged = false;
    
    //try to merge the peak with existing groups
    for (j=0; j<groups.size(); j++){
      curGroup = &(groups[j]);
      dTmax = abs(curGroup->maxTheta - curTheta);
      dTmin = abs(curGroup->minTheta - curTheta);
      dRmax = abs(curGroup->maxRadius - curRadius);
      dRmin = abs(curGroup->minRadius - curRadius);
      tInside = ((curTheta < curGroup->maxTheta) && (curTheta > curGroup->minTheta));
      rInside = ((curRadius < curGroup->maxRadius) && (curRadius > curGroup->minRadius));
      
      //true if the peak is close enough radius-wise or theta-wise
      inTheta = (dTmax < MERGE_THETA) || (dTmin < MERGE_THETA) || tInside;
      inRadius = (dRmax < MERGE_RADIUS) || (dRmin < MERGE_RADIUS) || rInside;
      
      //merge the peak with the group if it fits
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
    
    //add new group if we couldn't merge the peak
    if (!merged){
      struct peakGroup newGroup;
      newGroup.maxRadius = curRadius;
      newGroup.maxTheta = curTheta;
      newGroup.weight = curWeight;
      newGroup.numPoints = 1;
      newGroup.minRadius = newGroup.maxRadius;
      newGroup.minTheta = newGroup.maxTheta;
      newGroup.radius = newGroup.minRadius * newGroup.weight;
      newGroup.theta = newGroup.minTheta * newGroup.weight;
      groups.push_back(newGroup);
    }
  }
  

  int size = groups.size();
  
  //make all groups have positive radii
  for (i=0; i<size; i++){
    curGroup = &(groups[i]);
    
    if (curGroup->radius < (ADDITION*curGroup->weight)){
      curGroup->radius = 2*ADDITION*curGroup->weight - curGroup->radius;
      curGroup->maxRadius = 2*ADDITION - curGroup->maxRadius;
      curGroup->minRadius = 2*ADDITION - curGroup->minRadius;
      curGroup->theta -= (THETA_SIZE * curGroup->weight);
      curGroup->maxTheta -= THETA_SIZE;
      curGroup->minTheta -= THETA_SIZE;
    }
  }
  
//   ////////////////
//   //print groups
//   for (int i=0; i<groups.size(); i++){
//     curGroup = &(groups[i]);
//     
//     double t = curGroup->theta  / (double)curGroup->weight;
//     double r = curGroup->radius / (double)curGroup->weight;
//     r -= ADDITION;
//     r *= DISTANCE;
//     double w = curGroup->weight / (double)curGroup->numPoints;
//     
//     std::cout << "Group: " << t << " ";
//     std::cout << r << " ";
//     std::cout << w << " ";
//     std::cout << curGroup->maxTheta << " ";
//       std::cout << curGroup->minTheta << " ";
//       std::cout << curGroup->maxRadius << " ";
//       std::cout << curGroup->minRadius << std::endl;
//   }
//   ////////////////

  
  //merge groups into lines
  struct peakGroup* mergeGrp;
  char mergeMatrix [size];
  for (i=0; i<size; i++) mergeMatrix[i] = -1;
  
  bool tMaxOverlap, tMinOverlap, rMaxOverlap, rMinOverlap;
  
  //see which groups need to be merged
  for (i=0; i<size; i++){
    mergeGrp = &(groups[i]);
    for (j=i+1; j<size; j++){
      curGroup = &(groups[j]);
      
      dTmax = abs(curGroup->maxTheta - mergeGrp->minTheta);
      dTmin = abs(curGroup->minTheta - mergeGrp->maxTheta);
      dRmax = abs(curGroup->maxRadius - mergeGrp->minRadius);
      dRmin = abs(curGroup->minRadius - mergeGrp->maxRadius);
      
      tMaxOverlap = mergeGrp->maxTheta > curGroup->minTheta;
      tMinOverlap = mergeGrp->minTheta < curGroup->maxTheta;
      rMaxOverlap = mergeGrp->maxRadius > curGroup->minRadius;
      rMinOverlap = mergeGrp->minRadius < curGroup->maxRadius;
      
      //true if group is close enough theta-wise or radius-wise
      inTheta = (dTmax < MERGE_THETA) || (dTmin < MERGE_THETA) || (tMaxOverlap && tMinOverlap);
      inRadius = (dRmax < MERGE_RADIUS) || (dRmin < MERGE_RADIUS) || (rMaxOverlap && rMinOverlap);
      
      //merge groups if close enough
      if (inTheta && inRadius){
        mergeMatrix[j] = i;
      }
    }
  }
  
  
  //merge groups
  for (i=0; i<size; i++){
    if (mergeMatrix[i] == -1) continue;
    
    j = i;
    while (mergeMatrix[j] != -1) j = mergeMatrix[j];
    
    mergeGrp = &(groups[i]);
    curGroup = &(groups[j]);
    
    curGroup->maxRadius = std::max(mergeGrp->maxRadius, curGroup->maxRadius);
    curGroup->minRadius = std::min(mergeGrp->minRadius, curGroup->minRadius);
    curGroup->maxTheta = std::max(mergeGrp->maxTheta, curGroup->maxTheta);
    curGroup->minTheta = std::min(mergeGrp->minTheta, curGroup->minTheta);
    
    curGroup->radius += mergeGrp->radius;
    curGroup->theta += mergeGrp->theta;
    curGroup->weight += mergeGrp->weight;
    curGroup->numPoints += mergeGrp->numPoints;
  }
  
  
  //turn groups to lines and return
  for (i=0; i<size; i++){
    if (mergeMatrix[i] != -1) continue;
    
    curGroup = &(groups[i]);
    if (curGroup->numPoints < MIN_PEAKS) continue;
    
    struct houghLine curLine;
    curLine.theta = curGroup->theta / (double)curGroup->weight;
    curLine.theta *= 3.141592654 / THETA_SIZE;
    curLine.radius = curGroup->radius / (double)curGroup->weight;
    curLine.radius -= ADDITION;
    curLine.radius *= DISTANCE;
    curLine.weight = curGroup->weight / (double)curGroup->numPoints;
    
//     std::cout << "Lines: " << curLine.theta * 180.0/3.141592654 << " ";
//     std::cout << curLine.radius << " ";
//     std::cout << curLine.weight << " ";
//     std::cout << curGroup->numPoints << std::endl;
    
    lines->push_back(curLine);
  }
}



void HoughTransform::performHoughTransform(std::vector<ArSensorReading> *readings){
  int radius;
  double x, y;
  
  for (int i=0; i<readings->size(); i++){
    if ((*readings)[i].getRange() > MAX_DIST) continue; //check distance
    
    for (int t=0; t<THETA_SIZE; t++){
      x = (*readings)[i].getLocalX();
      y = (*readings)[i].getLocalY();
      radius = (int)round(x*COS_ARRAY[t] + y*SIN_ARRAY[t]);
      radius /= DISTANCE;
      radius += ADDITION;
      this->houghGrid[t*RADIUS_SIZE + radius]++;
    }
  }
}



void HoughTransform::getPeaks(int count, int peaks[]){
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
  std::sprintf(header, "P5\n%6d %6d\n100\n", RADIUS_SIZE, THETA_SIZE);
//   std::sprintf(header, "P5\n%6d %6d\n128\n", THETA_SIZE, RADIUS_SIZE);
  std::fputs(header, pgmFile);
  
  std::fwrite((void*)houghGrid, THETA_SIZE*RADIUS_SIZE, 1, pgmFile);
//   std::fflush(ppmFile);
  std::fclose(pgmFile);
}










