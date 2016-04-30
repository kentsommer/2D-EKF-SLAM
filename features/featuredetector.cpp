#include "featuredetector.h"

FeatureDetector::FeatureDetector(ArSick* sick){
  this->sick = sick;
  this->hough = new HoughTransform();
}


FeatureDetector::~FeatureDetector(){
  delete this->hough;
}


int FeatureDetector::getFeatures(std::vector<Feature> *featVec, double* structCompass){
  sick->lockDevice();
    std::vector<ArSensorReading> *readings = sick->getRawReadingsAsVector();
  sick->unlockDevice();
  
  std::vector<struct houghLine> houghLines;
  this->hough->getLines(readings, &houghLines);
  this->hough->clearHoughGrid();
  
  std::vector<struct lineSegment> lineSegments;
  this->fitLineSegments(readings, &houghLines, &lineSegments);
  
//   for (int i=0; i<lineSegments.size(); i++){
//     std::cout << "Segment: ";
//     std::cout << lineSegments[i].startX << " ";
//     std::cout << lineSegments[i].startY << " ";
//     std::cout << lineSegments[i].endX << " ";
//     std::cout << lineSegments[i].endY << " ";
//     std::cout << lineSegments[i].theta << " ";
//     std::cout << lineSegments[i].radius << " ";
//     std::cout << lineSegments[i].numPoints << std::endl;
//   }
//   std::cout << std::endl;
  
//   std::cout << "getting features...\n";
  int numf = this->extractCorners(featVec, &lineSegments);
  return numf;
  
//   for (int i=0; i<featVec->size(); i++){
//     std::cout << "Feature: ";
//     std::cout << (*featVec)[i].x << " ";
//     std::cout << (*featVec)[i].y << std::endl;;
//   }
  
//   (*structCompass) = getStructCompass(&lineSegments);
//   return numf;
}


//fit scan points to line segments
int FeatureDetector::fitLineSegments(std::vector<ArSensorReading> *readings, 
                                     std::vector<struct houghLine> *lines, 
                                     std::vector<struct lineSegment> *segments){
  int lineSize = lines->size();
  double locX, locY;
  double theta, radius, curRad, curDiff, minDiff;
  int mindex;
  
  float sin_array[lineSize];
  float cos_array[lineSize];
  std::vector<struct lineSegment*> initialSegs;
  
  for (int i=0; i<lineSize; i++){
    theta = (*lines)[i].theta;
    sin_array[i] = sin(theta);
    cos_array[i] = cos(theta);
    
//     std::cout << "Line: " << theta << " " << (*lines)[i].radius << std::endl;
    
    initialSegs.push_back(nullptr);
  }
  
  
  struct lineSegment* mergeSeg;
  
  for (int r=0; r<readings->size(); r++){
    if ((*readings)[r].getRange() > MAX_DIST) continue;
    
    minDiff = 1000000.0;
    locX = (*readings)[r].getLocalX();
    locY = (*readings)[r].getLocalY();
//     std::cout << "Point: " << locX << " " << locY << std::endl;
    
    //get closest line to point
    for (int l=0; l<lineSize; l++){
      radius = (*lines)[l].radius;
      curRad = locX*cos_array[l] + locY*sin_array[l];
      curDiff = fabs(radius - curRad);
      
      if (curDiff < minDiff){
        minDiff = curDiff;
        mindex = l;
      }
    }
    
//     std::cout << " " << minDiff << " " << (*lines)[mindex].theta << " " << (*lines)[mindex].radius << std::endl;
    
    //check if point is close enough to be on line at all
    if (minDiff > POINT_DIST) continue;
    
    
    //merge point with line segments
    mergeSeg = initialSegs[mindex];
    
    //horizontalish line
    if (fabs(sin_array[mindex]) > fabs(cos_array[mindex])){
      while (mergeSeg != nullptr){
        //point is in middle of line segment
        if ((locX <= mergeSeg->startX) && (locX >= mergeSeg->endX)){
          mergeSeg->numPoints++;
          break;
        }
        
        //point is on start end of line segment
        else if ((locX > mergeSeg->startX) && (fabs(locX - mergeSeg->startX) <= POINT_DIST)){
          mergeSeg->startX = locX;
          mergeSeg->startY = locY;
          mergeSeg->numPoints++;
          break;
          
        //point is on end end of line segment
        } else if ((locX < mergeSeg->endX) && (fabs(locX - mergeSeg->endX) <= POINT_DIST)){
          mergeSeg->endX = locX;
          mergeSeg->endY = locY;
          mergeSeg->numPoints++;
          break;
        
        //point is not on line segment
        } else {
          mergeSeg = mergeSeg->next;
        }
      }
    
    //verticalish line
    } else {
      while (mergeSeg != nullptr){
        //point is in middle of line segment
        if ((locY <= mergeSeg->startY) && (locY >= mergeSeg->endY)){
          mergeSeg->numPoints++;
          break;
        }
        
        //point is on start end of line segment
        else if ((locY > mergeSeg->startY) && (fabs(locY - mergeSeg->startY) <= POINT_DIST)){
          mergeSeg->startX = locX;
          mergeSeg->startY = locY;
          mergeSeg->numPoints++;
          break;
          
        //point is on end end of line segment
        } else if ((locY < mergeSeg->endY) && (fabs(locY - mergeSeg->endY) <= POINT_DIST)){
          mergeSeg->endX = locX;
          mergeSeg->endY = locY;
          mergeSeg->numPoints++;
          break;
        
        //point is not on line segment
        } else {
          mergeSeg = mergeSeg->next;
        }
      }
    }
    
    //if point was not merged with any existing segments, create new segment
    if (mergeSeg == nullptr){
      mergeSeg = new struct lineSegment;
      mergeSeg->theta = (*lines)[mindex].theta;
      mergeSeg->radius = (*lines)[mindex].radius;
      mergeSeg->numPoints = 1;
      mergeSeg->startX = locX;
      mergeSeg->startY = locY;
      mergeSeg->endX = locX;
      mergeSeg->endY = locY;
      mergeSeg->next = initialSegs[mindex];
      initialSegs[mindex] = mergeSeg;
    }
  }
  
  
  //push good line segments into vector
  int count = 0;
  struct lineSegment* last;
  bool isHoriz;
  
  for (int i=0; i<initialSegs.size(); i++){
    isHoriz = fabs(sin_array[i]) > fabs(cos_array[i]);
    if (isHoriz){
      mergeSeg = mergeSegmentsX(initialSegs[i]);
    } else {
      mergeSeg = mergeSegmentsY(initialSegs[i]);
    }
    
    while (mergeSeg != nullptr){
      if (mergeSeg->numPoints > MIN_POINTS){
        segments->push_back(*mergeSeg);
        count++;
      }
      
      last = mergeSeg;
      mergeSeg = mergeSeg->next;
      delete last;
    }
  }
  
  //return number of line segments
  return count;
}


struct FeatureDetector::lineSegment* FeatureDetector::mergeSegmentsX(struct lineSegment* segments){
  int size = 0;
  struct lineSegment* curSeg = segments;
  
  while(curSeg != nullptr){
    size++;
    curSeg = curSeg->next;
  }
  
  if (size < 2) return segments;
  
  int i,j;
  char mergeMatrix [size];
  struct lineSegment* segArray [size];
  curSeg = segments;
  for (int i=0; i<size; i++){
    mergeMatrix[i] = -1;
    segArray[i] = curSeg;
    curSeg = curSeg->next;
  }
  
  bool startClose, endClose, startOverlap, endOverlap;
  struct lineSegment* mergeSeg = segments;
  curSeg = segments;
  
  //see which segments need to be merged
  for (i=0; i<size; i++){
    mergeSeg = segArray[i];
    for (j=i+1; j<size; j++){
      curSeg = segArray[j];
      
      startClose = (fabs(mergeSeg->startX - curSeg->endX) < POINT_DIST);
      endClose = (fabs(mergeSeg->endX - curSeg->startX) < POINT_DIST);
      startOverlap = (mergeSeg->startX > curSeg->endX) && (mergeSeg->startX < curSeg->startX);
      endOverlap = (mergeSeg->endX > curSeg->endX) && (mergeSeg->endX < curSeg->startX);
      
      if (startClose || endClose || startOverlap || endOverlap){
        mergeMatrix[j] = i;
      }
    }
  }
  
  
  //merge segments
  for (i=0; i<size; i++){
    if (mergeMatrix[i] == -1) continue;
    
    j = i;
    while (mergeMatrix[j] != -1) j = mergeMatrix[j];
    
    mergeSeg = segArray[i];
    curSeg = segArray[j];
    
    if (mergeSeg->startX > curSeg->startX){
      curSeg->startX = mergeSeg->startX;
      curSeg->startY = mergeSeg->startY;
    }
    
    if (mergeSeg->endX < curSeg->endX){
      curSeg->endX = mergeSeg->endX;
      curSeg->endY = mergeSeg->endY;
    }
    
    curSeg->numPoints += mergeSeg->numPoints;
    delete mergeSeg;
  }
  
  curSeg = nullptr;
  //turn segment array to segment list and return
  for (i=0; i<size; i++){
    if (mergeMatrix[i] != -1) continue;
    
    mergeSeg = segArray[i];
    mergeSeg->next = curSeg;
    curSeg = mergeSeg;
  }
  
  return curSeg;
}


struct FeatureDetector::lineSegment* FeatureDetector::mergeSegmentsY(struct lineSegment* segments){
  int size = 0;
  struct lineSegment* curSeg = segments;
  
  while(curSeg != nullptr){
    size++;
    curSeg = curSeg->next;
  }
  
  if (size < 2) return segments;
  
  int i,j;
  char mergeMatrix [size];
  struct lineSegment* segArray [size];
  curSeg = segments;
  for (int i=0; i<size; i++){
    mergeMatrix[i] = -1;
    segArray[i] = curSeg;
    curSeg = curSeg->next;
  }
  
  bool startClose, endClose, startOverlap, endOverlap;
  struct lineSegment* mergeSeg = segments;
  curSeg = segments;
  
  //see which segments need to be merged
  for (i=0; i<size; i++){
    mergeSeg = segArray[i];
    for (j=i+1; j<size; j++){
      curSeg = segArray[j];
      
      startClose = (fabs(mergeSeg->startY - curSeg->endY) < POINT_DIST);
      endClose = (fabs(mergeSeg->endY - curSeg->startY) < POINT_DIST);
      startOverlap = (mergeSeg->startY > curSeg->endY) && (mergeSeg->startY < curSeg->startY);
      endOverlap = (mergeSeg->endY > curSeg->endY) && (mergeSeg->endY < curSeg->startY);
      
      if (startClose || endClose || startOverlap || endOverlap){
        mergeMatrix[j] = i;
      }
    }
  }
  
  
  //merge segments
  for (i=0; i<size; i++){
    if (mergeMatrix[i] == -1) continue;
    
    j = i;
    while (mergeMatrix[j] != -1) j = mergeMatrix[j];
    
    mergeSeg = segArray[i];
    curSeg = segArray[j];
    
    if (mergeSeg->startY > curSeg->startY){
      curSeg->startX = mergeSeg->startX;
      curSeg->startY = mergeSeg->startY;
    }
    
    if (mergeSeg->endY < curSeg->endY){
      curSeg->endX = mergeSeg->endX;
      curSeg->endY = mergeSeg->endY;
    }
    
    curSeg->numPoints += mergeSeg->numPoints;
    delete mergeSeg;
  }
  
  curSeg = nullptr;
  //turn segment array to segment list and return
  for (i=0; i<size; i++){
    if (mergeMatrix[i] != -1) continue;
    
    mergeSeg = segArray[i];
    mergeSeg->next = curSeg;
    curSeg = mergeSeg;
  }
  
  return curSeg;
}



double FeatureDetector::getStructCompass(std::vector<struct lineSegment> *segments){
  return NO_COMPASS;
}


int FeatureDetector::extractCorners(std::vector<Feature> *featVec, std::vector<struct lineSegment> *segments){
  int numSeg = segments->size();
  
  float sin_array[numSeg];
  float cos_array[numSeg];
  
  double theta;
  for (int i=0; i<numSeg; i++){
    theta = (*segments)[i].theta;
    sin_array[i] = sin(theta);
    cos_array[i] = cos(theta);
  }
    
  struct lineSegment seg1, seg2;
  double thetaDiff;
  double det, x, y, dx, dy;
  bool start1, end1, start2, end2;
  int count;
  
  //get all intersects between close line segments
  for (int i=0; i<numSeg; i++){
    seg1 = (*segments)[i];
    
    for (int j=i+1; j<numSeg; j++){
      seg2 = (*segments)[j];
      
      thetaDiff = fabs(seg1.theta - seg2.theta);
      if (thetaDiff > 3.141592654) thetaDiff = fabs(thetaDiff - 6.283185307);
      if (thetaDiff > 1.570796327) thetaDiff = fabs(thetaDiff - 3.141592654);
      
      if (thetaDiff < CORNER_THETA) continue;
      
      det = cos_array[i]*sin_array[j] - sin_array[i]*cos_array[j];
      x = (seg1.radius * sin_array[j] - seg2.radius * sin_array[i]) / det;
      y = (seg2.radius * cos_array[i] - seg1.radius * cos_array[j]) / det;
      
      dx = seg1.startX - x;
      dy = seg1.startY - y;
      start1 = (dx*dx + dy*dy) < CORNER_DIST;
      dx = seg1.endX - x;
      dy = seg1.endY - y;
      end1 = (dx*dx + dy*dy) < CORNER_DIST;
      dx = seg2.startX - x;
      dy = seg2.startY - y;
      start2 = (dx*dx + dy*dy) < CORNER_DIST;
      dx = seg2.endX - x;
      dy = seg2.endY - y;
      end2 = (dx*dx + dy*dy) < CORNER_DIST;
      
      if ((start1 || end1) && (start2 || end2)){
        Feature f;
        f.x = x;
        f.y = y;
        featVec->push_back(f);
        count++;
      }
    }
  }
  
  return count;
}












