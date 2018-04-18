/**	\file gridmap.h
 * \brief Implementation an occupancymap for mobile robot. 
 *
 * \author Hjalte Bested Møller
 * \date 4. April 2018	
 *
*/

#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define ASTAR_USE_OPENCV
#include "astar.h"

using namespace std;
using namespace Eigen;
using namespace cv;


#ifndef USE_CV_DISTANCE_TRANSFORM
#define USE_CV_DISTANCE_TRANSFORM
#endif

class LaserScanner {
public:
    // Create Laser Scanner 
    LaserScanner() { 
        // Laser Parameters
        this->maxDistance = 4.0f;
        this->resolDeg = 0.36f;
        this->fov = 180.0f;
        this->init();
    } 

    float maxDistance;  /// Max Distance (Laser Measurement Range) in meters.
    float resolDeg;     /// Laser Scanner Resolution in Deg
    float resolRad;     /// Laser Scanner Resolution in Rad
    float fov;          /// Laser Scanner Field-of-view in Deg

    int nScanLines;     /// Number of scanning lines (deduced from scan width and resolution)
    MatrixXf scanData;  /// Matrix for holding the laser scan data format is scanData(2,nScanLines), where phi(i) = scanData(0,i), rho(i) = scanData(1,i)

    /** Initialise LaserScanner: Calculate the resolution in radian and determine the number of scanning lines (nScanLines). */
    void init(){
        this->resolRad = resolDeg*M_PI/180.0;
        this->nScanLines = round(fov/resolDeg+1);
    }

    Vector3f pose;  /// Scan Pose in world coordinates (x,y,theta)

    /// Set Scan Pose in world coordinates (x,y,theta)
    void setPose(Vector3f newpose){ 
        pose = newpose;
    }
    /// Set Scan Pose in world coordinates (x,y,theta)
    void setPose(float x, float y, float th){ 
        pose << x,y,th; 
    }

    /// Return pose
    Vector3f getPose(){ 
        return pose; 
    }

    /** Convert polar coordinates to cartesian coordinates */
    MatrixXf polarToCart(MatrixXf const& scanPolar){
        int nScanLines = scanPolar.cols();
        MatrixXf scanCart(2,nScanLines);
        for(int i=0; i<nScanLines; i++){
            double phi = scanPolar(0,i);
            double rho = scanPolar(1,i);
            double x = rho * cos(phi);
            double y = rho * sin(phi);
            scanCart(0,i) = x;
            scanCart(1,i) = y;
        }
        return scanCart;
    }

    /** Convert cartesian coordinates in the scanner frame to cartesian coordinates in the world frame */
    MatrixXf cartToWorld(MatrixXf const& scanCart, Vector3f const& pose){
        int nScanLines = scanCart.cols();
        MatrixXf scanWorld(2,nScanLines);
        double x = pose(0);
        double y = pose(1);
        double th = pose(2);
        double costh = cos(th);
        double sinth = sin(th);

        // Transform all the coordinates and store them in scanWorld
        for(int i=0; i<nScanLines; i++){
            double xs = scanCart(0,i);
            double ys = scanCart(1,i);
            double xw = xs*costh - ys*sinth + x;
            double yw = ys*costh + xs*sinth + y;
            scanWorld(0,i) = xw;
            scanWorld(1,i) = yw;
        }
        return scanWorld;
    }

    /** Convert polar coordinates in the scanner frame to cartesian coordinates in the world frame directly */
    inline MatrixXf polarToWorld(MatrixXf const& scanPolar, Vector3f const& pose){
        int nScanLines = scanPolar.cols();
        MatrixXf scanWorld(2,nScanLines);
        double x = pose(0);
        double y = pose(1);
        double th = pose(2);
        double costh = cos(th);
        double sinth = sin(th);

        for(int i=0; i<nScanLines; i++){
            double phi = scanPolar(0,i);
            double rho = scanPolar(1,i);
            double xs = rho * cos(phi);
            double ys = rho * sin(phi);
            double xw = xs*costh - ys*sinth + x;
            double yw = ys*costh + xs*sinth + y;
            scanWorld(0,i) = xw;
            scanWorld(1,i) = yw;
        }
        return scanWorld;
    }

    /** Overloaded convinience method */
    inline MatrixXf cartToWorld(MatrixXf const& scanCart){
        return cartToWorld(scanCart, this->pose);
    }

    /** Overloaded convinience method */
    inline MatrixXf polarToWorld(MatrixXf const& scanPolar){
        return polarToWorld(scanPolar,pose);
    }

    /// Return scan in polar (default) coordinates
    inline MatrixXf scanPolar(){
        return scanData;
    }

    /// Return scan in Laser local cartesian coordinates
    inline MatrixXf scanCart(){
        return polarToCart(scanData);
    }

    /// Return scan in world cartesian coordinates, based on the internal pose
    inline MatrixXf scanCartWorld(){
        return polarToWorld(scanData, pose);
    }

    void resize(int nScanLines){
        this->nScanLines = nScanLines;
        scanData.resize(2,nScanLines);
    }

    void setScanPoint(int i, float phi, float r){
        scanData(0,i) = phi;
        scanData(1,i) = r;
    }

    /** Simulate a laser scan. The lines are defined as a matrix where each column represents a line and the rows are [x1; y1; x2; y2] */
    MatrixXf simScan(Vector3f const& pose, MatrixXf const& lines){
        int nLines = lines.cols();
        double x = pose(0);
        double y = pose(1);
        double th = pose(2);
        double costh = cos(th);
        double sinth = sin(th);
        // cout << "th=" << th << endl;

        /* ----------------------------------------
         * ---- Pre-allocate the memory needs -----
         * --------------------------------------*/

        // Global system coordinates
        VectorXd xStart(nLines);
        VectorXd yStart(nLines);
        VectorXd xEnd(nLines);
        VectorXd yEnd(nLines);

        // Laser scanner local system coordinates
        VectorXd xStartTrans(nLines);
        VectorXd yStartTrans(nLines);
        VectorXd xEndTrans(nLines);
        VectorXd yEndTrans(nLines);

        VectorXd a(nLines);
        VectorXd b(nLines);
        VectorXd c(nLines);

        // Rezise the scanData matrix, if the size is correct already, Eigen 
        // ensures that this is a no-operation
        scanData.resize(2,nScanLines); 


        /* ------------------------------------------------------------------------
         * --- Conversion of the lines from global frame to scanner local frame ---
         * --------------------------------------------------------------------- */

        // For each line:
        // cout << "nLines" << nLines << endl;

        for(int i=0; i<nLines; i++){
            // Start and end values for the lines ending points.
            xStart(i) = lines(0,i);
            yStart(i) = lines(1,i);
            xEnd(i)   = lines(2,i);
            yEnd(i)   = lines(3,i);
            // cout << "Line = (" << xStart(i) << "," << yStart(i) << "," << xEnd(i) << "," << yEnd(i) << ")" << endl;

            // Transformation of the lines to the laser scanner's coordinate system.
            // Lines are converted to the new coordinate system.
            xStartTrans(i) = (xStart(i)-x)*costh + (yStart(i)-y)*sinth;
            yStartTrans(i) = (yStart(i)-y)*costh - (xStart(i)-x)*sinth; 
            xEndTrans(i) = (xEnd(i)-x)*costh   + (yEnd(i)-y)*sinth;
            yEndTrans(i) = (yEnd(i)-y)*costh   - (xEnd(i)-x)*sinth;
            // cout << "LineTrans = (" << xStartTrans(i) << "," << yStartTrans(i) << "," << xEndTrans(i) << "," << yEndTrans(i) << ")" << endl;

            // Starting and ending points are swapped if the x value of the 
            // starting point is bigger than the x value of the ending point.
            if( xStartTrans(i) > xEndTrans(i)){
                double temp = xStartTrans(i);
                xStartTrans(i) = xEndTrans(i);
                xEndTrans(i) = temp;
                temp = yStartTrans(i);
                yStartTrans(i) = yEndTrans(i);
                yEndTrans(i) = temp;
            }

            if( xStartTrans(i) == xEndTrans(i)){
                xEndTrans(i) += 1e-6;
            }
            // cout << "LineTrans = (" << xStartTrans(i) << "," << yStartTrans(i) << "," << xEndTrans(i) << "," << yEndTrans(i) << ")" << endl;

            // The line equations are calculated 
            // a*x+ b*y=c - IMPORTANT: Variables related to the line equations must 
            // be double precition to avoid numerical issues!
            a(i) = yStartTrans(i)-yEndTrans(i);
            b(i) = xEndTrans(i)-xStartTrans(i);
            double a2 = a(i)*a(i);
            double b2 = b(i)*b(i);
            double l = sqrt(a2+b2);
            a(i) = a(i)/l;
            b(i) = b(i)/l;
            c(i) = a(i)*xStartTrans(i)+b(i)*yStartTrans(i);
            // cout << "Line " << i << ": " << a(i) << "x + " << b(i) << "y = " << c(i)<< endl;
        }

        /* -------------------------------------------------------------------
         * Conversion of what the laser scanner sees to polar coordinates.
         * -------------------------------------------------------------------*/

        // For each laser scanner angle:
        for(int i=0; i<nScanLines; i++){
            // Laser scanner maximum measured distance maxDistance(in meters)
            // Closest distance from the laser scanner.
            double min_dist = maxDistance;

            // Current laser scanner angle 
            double phi = i*resolRad-fov/2.0*M_PI/180.0; 
            // Find distance from the lines to laser scanner in the current angle
            double cosphi = cos(phi);
            double sinphi = sin(phi);
            // cout << "cos(phi)=" << cosphi << ", sin(phi)=" << sinphi << endl;
            for(int j=0; j<nLines; j++){
                double temp = a(j)*cosphi + b(j)*sinphi;
                if(abs(temp) > 1e-8){
                    double t = c(j)/temp;
                    if(t>0 && t<min_dist){
                        if(abs(xStartTrans(j)-xEndTrans(j)) > 1e-8){
                            if(t*cosphi < xEndTrans(j) && t*cosphi > xStartTrans(j)) min_dist=t;
                        else 
                            if(yEndTrans(j) > yStartTrans(j)){
                                if(t*sinphi < yEndTrans(j) && t*sinphi > yStartTrans(j)) min_dist=t; 
                            } else {
                                if(t*sinphi > yEndTrans(j) && t*sinphi < yStartTrans(j)) min_dist=t; 
                            }
                            
                        }
                    }
                }    
            }

            // The polar coordinates returned
            scanData(0,i) = phi;
            scanData(1,i) = min_dist;
        }
        return scanData;
    }

    inline MatrixXf simScan(MatrixXf const& lines){
        return simScan(pose,lines);
    }
};


class GridMap {
public:
    Mat mapData;
    Mat dialatedMap;
    Mat strel;

    Mat invDialatedMap;
    Mat distanceMap_32F;
    Mat distanceMap;
    bool wrapMap = true;

    // Mapsize
    float cellSize = 1;
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;
    float xoffset = 0;
    float yoffset = 0;

    int fillMode = 0;
    int clearMode = 0;
    Astar astar;

    float widthInMeters() {
        return width*cellSize;
    }
    float heightInMeters() {
        return height*cellSize;
    }

    void setOffset(float xoffset, float yoffset){
        this->xoffset = xoffset;
        this->yoffset = yoffset;
    }

    void resize(float widthInMeters, float heightInMeters, float cellSize){
        this->cellSize = cellSize;
        this->height = ceil(heightInMeters/cellSize);
        this->width = ceil(widthInMeters/cellSize);
        this->size = width * height;
        this->setOffset(widthInMeters/2,heightInMeters/2);
        mapData.create(height,width,CV_8SC1);
        dialatedMap.create(height,width,CV_8SC1);
        invDialatedMap.create(height,width,CV_8SC1);
        distanceMap_32F.create(height,width,CV_32F);
        distanceMap.create(height,width,CV_8SC1);
        astar = Astar(height, width);
        astar.wrapMap = wrapMap;
        this->clear();
    }

    void makeStrel(float width){
        int dilationSize = ceil(0.5*width/cellSize);
        strel = getStructuringElement( 
            MORPH_RECT,
            Size( 2*dilationSize+1, 2*dilationSize+1 ),
            Point( dilationSize, dilationSize ) );
    }

    GridMap() { }
    GridMap(float widthInMeters, float heightInMeters, float cellSize) {
        this->resize(widthInMeters, heightInMeters, cellSize);
    }

    void clear(){
        mapData.setTo(-1);
        dialatedMap.setTo(0);
        astar.clear();
    }
    
    /// An implementation of Bresenham's line algorithm, see http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    vector<Point> bresenhamPoints(int x0, int y0, int x1, int y1){
        Point point;
        vector<Point> points;
        int ystep = -1;

        bool steep=abs(y1-y0)>abs(x1-x0);

        if(steep){ // slope more than 1
            int tempx0=x0; x0 = y0; y0 = tempx0; // Swapping
            int tempx1=x1; x1 = y1; y1 = tempx1; // Swapping
        }
        if(x0>x1){ // line goes to the left
            int tempx0=x0; x0=x1; x1=tempx0; // Swapping
            int tempy0=y0; y0=y1; y1=tempy0; // Swapping
        }

        int deltaX = x1-x0;
        int deltaY = abs(y1-y0);
        int N = deltaX+1;
        float error=-N/2;

        int x=x0;
        int y=y0;

        if(y0<y1) ystep=1;

        for(int i=0; i<N; i++){
            if(steep){
                point.x = y;
                point.y = x;
            } else {
                point.x = x;
                point.y = y;
            }
            x++;
            error=error+deltaY;
            if(error >= 0){
                y=y+ystep;
                error=error-deltaX;
            }
            points.push_back(point);
        }   
        return points;
    }
    
    /// Round towards zero
    int fix(float val){
        if(val > 0) return floor(val);
        if(val < 0) return ceil(val);
        return val;
    }

    inline Point findCell(float x, float y){
        int& rows = mapData.rows;
        x = x+xoffset;
        y = y+yoffset;
        Point cell;
        cell.x = x/cellSize;
        cell.y = rows - floor(y/cellSize + 1);
        return cell;
    }

    inline Point worldToMap(float x, float y){
        int& rows = mapData.rows;
        x = x+xoffset;
        y = y+yoffset;
        Point cell;
        cell.x = x/cellSize;
        cell.y = rows - floor(y/cellSize + 1);
        return cell;
    }

    inline Point2f mapToWorld(int const& x, int const& y){
        int& rows = mapData.rows;
        Point2f world;
        // Translate from map coordinates to world coordinates
        world.x =  (x+0.5) * cellSize - xoffset;
        world.y =  (rows-y-0.5) * cellSize - yoffset;
        return world;
    }

    inline Point worldToMap(Point2f const& world){
        return mapToWorld(world.x,world.y);
    }

    inline Point2f mapToWorld(Point const& cell){
        return mapToWorld(cell.x,cell.y);
    }


    inline Point wrap(Point cell){
        int& rows = mapData.rows;
        int& cols = mapData.cols;

        // Wrapping the map
        while(cell.x < 0)       cell.x += cols;
        while(cell.x >= cols)   cell.x -= cols;
        while(cell.y < 0)       cell.y += rows;    
        while(cell.y >= rows)   cell.y -= rows;
        return cell;
    }

    inline int wx(int x){
        // Wrapping the map
        if(wrapMap){
            int cols = mapData.cols;
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
        }   
        return x;
    }
    inline int wy(int y){
        if(wrapMap){
            // Wrapping the map
            int rows = mapData.rows;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
        }
        return y;
    } 

    inline void resetMapNode(int const& x, int const& y, char const& value){
        MapNode*node = astar.mapAt(x,y);
        if(node != NULL){
            node->x = x;
            node->y = y;
            node->clear();
            node->type = value;
        }
    }

    void setCell(int const& x, int const& y, char const& value, int const& mode){
        int& rows = mapData.rows;
        int& cols = mapData.cols;
        int xm = wx(x);
        int ym = wy(y);

        if((0 <= xm && xm < cols) && (0 <= ym && ym < rows)){
            resetMapNode(x,y,value);
            mapData.at<char>(ym,xm) = value;

            int xm1; int xp1; int ym1; int yp1;
            if(mode > 0){
                xm1=wx(xm-1); xp1=wx(xm+1); ym1=wy(ym-1); yp1=wy(ym+1);
                if(0 <= xm1){    mapData.at<char>(ym,xm1) = value; resetMapNode(x-1,y,value);}
                if(xp1 < cols){  mapData.at<char>(ym,xp1) = value; resetMapNode(x+1,y,value);}
                if(0 <= ym1){    mapData.at<char>(ym1,xm) = value; resetMapNode(x,y-1,value);}
                if(yp1 < rows){  mapData.at<char>(yp1,xm) = value; resetMapNode(x,y+1,value);}
            }
            if(mode > 1){
                if(0 <= xm1 && 0 <= ym1){    mapData.at<char>(ym1,xm1)=value;  resetMapNode(x-1,y-1,value);}
                if(0 <= xm1 && yp1 < rows){  mapData.at<char>(yp1,xm1)=value;  resetMapNode(x-1,y+1,value);}
                if(xp1 < cols && 0 <= ym1){  mapData.at<char>(ym1,xp1)=value;  resetMapNode(x+1,y-1,value);}
                if(xp1 < cols && yp1 < rows){mapData.at<char>(yp1,xp1)=value;  resetMapNode(x+1,y+1,value);}
            }
            // cout << "SetCell (" << x << "," << y << ") = " << int(value) << ",mode=" << mode << endl;
        }
    }

    char mapAt(int x, int y){
        int& rows = mapData.rows;
        int& cols = mapData.cols;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
        }

        if((0 <= x && x < cols) && (0 <= y && y < rows)){
            return mapData.at<char>(y,x);
        }

        return 0;
    }

    uchar dialatedMapAt(int x, int y){
        int& rows = mapData.rows;
        int& cols = mapData.cols;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
        }

        if((0 <= x && x < cols) && (0 <= y && y < rows)){
            return dialatedMap.at<uchar>(y,x);
        }

        return 0xFF;
    }

    uchar distanceMapAt(int x, int y){
        int& rows = mapData.rows;
        int& cols = mapData.cols;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
        }

        if((0 <= x && x < cols) && (0 <= y && y < rows)){
            return distanceMap.at<uchar>(y,x);
        }

        return 0;
    }

    // store the converted scans globally as they are used by several funcitons,
    // maybe store them in LaserScanner ?
    MatrixXf scanPolar;
    MatrixXf scanWorld;
    Point scanPosCell;

    void updateMap(LaserScanner *scan, float maxLSDist){
        float& x = scan->pose(0);
        float& y = scan->pose(1);

        // Cell Laser Scanner Position
        scanPosCell = findCell(x,y);
        // cout << "scanPosCell = (" << scanPosCell.x << "," << scanPosCell.y << ")" << endl;

        // Convert polar scan coordinates to laser cartesian coordinates
        scanPolar = scan->scanPolar();
        // Convert laser cartesian coordinates to world cartesian coordinates
        scanWorld = scan->scanCartWorld();

        vector<Point> pointsToFill;

        // Convert scan from world cartesian coordinates to cell position
        for(int i=0; i<scan->nScanLines; i++){
            float& xs = scanWorld(0,i);
            float& ys = scanWorld(1,i);
            float& rho = scanPolar(1,i);
            Point cell = findCell(xs, ys);
            
            // Clear cells using bresenham's algorithm
            // cout << "ScanPosCell: (" << scanPosCell.x << "," << scanPosCell.y << "), cell: (" << cell.x << "," << cell.y << ")" << endl; 
            vector<Point> pointsToClear = bresenhamPoints(scanPosCell.x, scanPosCell.y, cell.x, cell.y);
            // cout << "Number of Points To Clear = " << pointsToClear.size() << endl; 

            for(uint ii=0; ii<pointsToClear.size(); ii++){
                setCell(pointsToClear[ii].x, pointsToClear[ii].y, 0, clearMode);
            }

            if(rho <= maxLSDist) pointsToFill.push_back(cell);
        }

        for(uint ii=0; ii<pointsToFill.size(); ii++){
            setCell(pointsToFill[ii].x, pointsToFill[ii].y, 1, fillMode);
        }

        // cout << "scanWorldCell:\n" << scanWorldCell.transpose() << endl;
    }

    void transform(){
        dilate( mapData > 0, dialatedMap, strel);
        #ifdef USE_CV_DISTANCE_TRANSFORM
        bitwise_not(dialatedMap, invDialatedMap);
        distanceTransform(invDialatedMap, distanceMap_32F, CV_DIST_L2, 5, CV_32F);
        distanceMap_32F.convertTo(distanceMap,dialatedMap.type());
        #endif
   }

    /** Determine next waypoint defined as the furthest point seen by the laserscanner which is
    not occupied in the dialated obstacle map. The search has a preference for scanlines with small angles, which is achieved by 
    iterating from the middle and out and only replacing if a longer distance is found. Thus, small angles are only used in case of ties.*/
    Point determineNextWaypointCell(LaserScanner *scan){
        float biggestDistanceInCells = 0;
        Point mostDistantCell;
        int nScanLinesHalf = scan->nScanLines/2;

        // Convert scan from world cartesian coordinates to cell position
        for(int i=0; i<=nScanLinesHalf; i++){
            int k;
            for(int j=0; j<2; j++){
                if(j==0)   k = nScanLinesHalf-i;
                else       k = nScanLinesHalf+i;
                if(k >= 0 && k < scan->nScanLines){
                    float& xs  = scanWorld(0,k);
                    float& ys  = scanWorld(1,k);
                    // float& phi = scanPolar(0,k);
                    float& rho = scanPolar(1,k);
                    if(rho < (biggestDistanceInCells+1)*cellSize) continue;
                    Point cell = findCell(xs, ys);
                    // int obstdist = distanceMapAt(cell.x,cell.y);

                    // Clear search for furthest available point using Bresenham's algorithm
                    Point closestOccupiedPoint;
                    vector<Point> points = bresenhamPoints(scanPosCell.x, scanPosCell.y, cell.x, cell.y);                    

                    // cout << "Diff:" <<scanPosCell-cell << endl;
                    for(uint ii=0; ii<points.size(); ii++){
                        Point point = points[ii];
                        float dx = point.x - scanPosCell.x;
                        float dy = point.y - scanPosCell.y;
                        float celldist = sqrt(dx*dx+dy*dy);
                        if(celldist > biggestDistanceInCells && dialatedMapAt(point.x,point.y) == 0){
                            biggestDistanceInCells = celldist;
                            mostDistantCell = point;
                        }
                    }
                }
            }
        }
  
        return mostDistantCell;
    }

    /** Determine next waypoint defined as the furthest point seen by the laserscanner, 
    not occupied in the dialated map, with preference for scanlines with small angles, achieved by 
    iterating from the middle and out and only replacing if a longer distance is found. */
    inline Point2f determineNextWaypoint(LaserScanner *scan){
        return mapToWorld(determineNextWaypointCell(scan));
    }

    /** This function will find the minimal cost path from the starting cell to the target cell. 
    The function uses the A* search algorithm with tie-breaker and an additional cost related to 
    the obstacle distance. */
    vector<MapNode *> findpath(int const& xStart, int const& yStart, int const& xTarget, int const& yTarget, unsigned long const& maxIter = 10000){
        // vector<MapNode *> path;
        astar.clear();
        for (int y = 0; y < mapData.rows; y++) {
            for (int x = 0; x < mapData.cols; x++) {
                MapNode *node = astar.mapAt(x,y);
                node->type = NODE_TYPE_ZERO;
                if(dialatedMapAt(x,y) > 0) node->type = NODE_TYPE_OBSTACLE;
                #ifdef USE_CV_DISTANCE_TRANSFORM
                node->obstdist = distanceMapAt(x,y);
                #endif
            }
        }

        return astar.findpath(xStart, yStart, xTarget, yTarget, maxIter);
    }

    inline vector<MapNode *> simplifyPath(vector<MapNode *> const& path){
        return astar.simplifyPath(path);
    }

    vector<Point2f> pathToWorld(vector<MapNode *> const& path){
        vector<Point2f> waypoints;
        for(int i=0; i<path.size(); i++){
            waypoints.push_back(mapToWorld(path[i]->x,path[i]->y));
        }
        
        /*
        cout << "wayPoints:";
        for(int i=0; i<waypoints.size(); i++){
            cout << "->(" << waypoints[i].x << "," << waypoints[i].y << ")";
        }
        cout << endl;
        */
        return waypoints;
    }

    inline vector<Point2f> findpathAsWaypoints(int const&  xStart, int const& yStart, int const& xTarget, int const& yTarget){
        return pathToWorld(simplifyPath(findpath(xStart, yStart, xTarget, yTarget)));
    }

};



#endif

