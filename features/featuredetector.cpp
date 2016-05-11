#include "featuredetector.h"

//constructor
FeatureDetector::FeatureDetector(ArSick* sick){
  this->sick = sick;
  this->hough = new HoughTransform();
}

//destructor
FeatureDetector::~FeatureDetector(){
  delete this->hough;
}


//main getFeatures function. Returns number of features found
int FeatureDetector::getFeatures(std::vector<Feature> *featVec, double* structCompass, double curPhi){
  //get current scan points
  sick->lockDevice();
    std::vector<ArSensorReading> *r = sick->getRawReadingsAsVector();
    std::vector<ArSensorReading> readings(*r);
    ArTime curTime = sick->getLastReadingTime();
  sick->unlockDevice();
  
  //make sure we have readings in the scan
  if (readings.size() == 0){
    (*structCompass) = NO_COMPASS;
    return 0;
  }
  
  //make sure this is a new scan
  if (curTime.isAt(Last_Time)){
    (*structCompass) = NO_COMPASS;
    return 0;
  }
  Last_Time = curTime;
  
  //get lines in scan from hough transform
  std::vector<struct houghLine> houghLines;
  this->hough->getLines(&readings, &houghLines);
  this->hough->clearHoughGrid();
  
  //fit points to lines to get line segments
  std::vector<struct lineSegment> lineSegments;
  this->fitLineSegments(&readings, &houghLines, &lineSegments);
  
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
  
  //extract corners from line segments
  int numf = this->extractCorners(featVec, &lineSegments);
  
//   for (int i=0; i<featVec->size(); i++){
//     std::cout << "Feature: ";
//     std::cout << (*featVec)[i].x << " ";
//     std::cout << (*featVec)[i].y << std::endl;;
//   }
  
  //get structural compass measurement
  (*structCompass) = getStructCompass(&houghLines, curPhi);
  return numf;
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
  
  //initialize arrays
  for (int i=0; i<lineSize; i++){
    theta = (*lines)[i].theta;
    sin_array[i] = sin(theta);
    cos_array[i] = cos(theta);    
    initialSegs.push_back(nullptr);
  }
  
  
  struct lineSegment* mergeSeg;
  
  //loop through all points in scan and add to line segments
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
        
    //check if point is close enough to be on line at all
    if (minDiff > POINT_DIST) continue;
    
    //merge point with line segments
    mergeSeg = initialSegs[mindex];
    
    //horizontalish line merging
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
    
    //verticalish line merging
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
  
  for (int i=0; i<initialSegs.size(); i++){
    mergeSeg = initialSegs[i];
    
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


//pull corner features from line segments
int FeatureDetector::extractCorners(std::vector<Feature> *featVec, std::vector<struct lineSegment> *segments){
  int numSeg = segments->size();
  
  float sin_array[numSeg];
  float cos_array[numSeg];
  
  //initialize arrays
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
      
      //find intersection point of line segments
      det = cos_array[i]*sin_array[j] - sin_array[i]*cos_array[j];
      x = (seg1.radius * sin_array[j] - seg2.radius * sin_array[i]) / det;
      y = (seg2.radius * cos_array[i] - seg1.radius * cos_array[j]) / det;
      
      //get distance between line segments and intersection point
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
      
      //if segments are close enough to intersection, save intersectino as new corner
      if ((start1 || end1) && (start2 || end2) && ((x*x + y*y) > MIN_DIST)){
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



//find and return the estimated angle of the robot based on structual compass
double FeatureDetector::getStructCompass(std::vector<struct houghLine> *lines, double curPhi){
  std::vector<struct compassgroup> groups;
    
  //merge all lines into compassgroups
  for (int i=0; i<lines->size(); i++){
    double curTheta = (*lines)[i].theta - 1.570796327 * floor((*lines)[i].theta / 1.570796327);
    double curWeight = (*lines)[i].weight;
    
    bool merge = false;
    
    //check to see if we can merge line with existing group
    for (int j=0; j<groups.size(); j++){
      double mergeTheta = groups[j].theta / groups[j].weight;
      double thetaDiff = fabs(curTheta - mergeTheta);
      
      if (thetaDiff < COMPASS_THRESH){
        groups[j].theta += curTheta*curWeight;
        groups[j].weight += curWeight;
        merge = true;
      }
      
    }

    //if we can't merge, make a new compass group
    if (!merge){
      struct compassgroup g;
      g.theta = curTheta*curWeight;
      g.weight = curWeight;
      groups.push_back(g);
    }
  }
  
  //get compassgroup with largest weight
  struct compassgroup max;
  double maxweight = 0.0;
  for (int i=0; i<groups.size(); i++){
    if (groups[i].weight > maxweight){
      max = groups[i];
      maxweight = max.weight;
    }
  }
  
  //if no groups, can't find compass value
  if (maxweight == 0.0) return NO_COMPASS;
  
  //get estimated angle (based on structural compass) of robot mod 90 degrees
  double cardinal = -(max.theta / maxweight);
  if (COMPASS_OFFSET == 100.0) COMPASS_OFFSET = cardinal;
  cardinal -= COMPASS_OFFSET;
  cardinal -= 1.570796327 * floor(cardinal / 1.570796327);
  
  //get robot angle mod 360 degrees
  curPhi -= 6.283185307 * floor(curPhi / 6.283185307);
  
  //get errors for all possible directions of robot (0, 90, 180, 360 degrees plus rollovers)
  double e1 = fabs(curPhi - cardinal);
  double e2 = fabs(curPhi - cardinal - 1.570796327);
  double e3 = fabs(curPhi - cardinal - 3.141592654);
  double e4 = fabs(curPhi - cardinal - 4.71238898);
  double e5 = fabs(curPhi - cardinal - 6.283185307);
  double e6 = fabs(curPhi - cardinal + 1.570796327);
  
  //return best estimate of robot's angle. If the error between the robot's angle and the
    //structual compass value is more than 45 degrees, this WILL return bad values. There
    //isn't any way to correct for this, though, so we have to hope we never get that much error
  if ((e1 <= e2) && (e1 <= e3) && (e1 <= e4) && (e1 <= e5) && (e1 <= e6)) return cardinal;
  else if ((e2 <= e3) && (e2 <= e4) && (e2 <= e5) && (e2 <= e6)) return cardinal + 1.570796327;
  else if ((e3 <= e4) && (e3 <= e5) && (e3 <= e6)) return cardinal + 3.141592654;
  else if ((e4 <= e5) && (e4 <= e6)) return cardinal + 4.71238898;
  else if (e5 <= e6) return cardinal;
  else return cardinal + 4.71238898;
}

