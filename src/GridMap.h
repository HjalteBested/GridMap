/**	\file gridmap.h
 * \brief Implementation of the A* search Algorithm.
 *
 * \author Hjalte Bested Møller
 * \date 4. April 2018	
 *
*/

#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace cv;

const int ALLOW_DIAGONAL_PASSTHROUGH = 1;
const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const int G_DIRECT_COST   = 100; 
const int G_DIAGONAL_COST = 141;	// ≈ 100 sqrt(2)
const int H_AMITGAIN = 0;			// Zero means it is disabled completely

class MapSize {
public:
    float cellSize = 1;
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    MapSize() { }
    MapSize(float widthInMeters, float heightInMeters, float cellSize) {
        this->cellSize = cellSize;
        this->height = ceil(heightInMeters/cellSize);
        this->width = ceil(widthInMeters/cellSize);
        this->size = width * height;
    }

    float widthInMeters() {
        return width*cellSize;
    }
    float heightInMeters() {
        return height*cellSize;
    }
};

class MapNode {
public:
    int x = -1;	// x-position
    int y = -1; // y-position
    int h = 0;	// Heuristic cost
    int g = 0;  // Cost from start to node
    int c = 0;  // Terrain Cost
    int type = NODE_TYPE_ZERO;
    int flag = NODE_FLAG_UNDEFINED;
    MapNode *parent = 0;

    MapNode() { }

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = 0){
        this->x = x;
        this->y = y;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    int f() {
        return g + h + c;
    }
};

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

    void init(){
        this->resolRad = resolDeg*M_PI/180.0;
        this->nScanLines = round(fov/resolDeg+1);
    }

    /// Scan Pose in world coordinates (x,y,theta);
    Vector3f pose;      
    void setPose(Vector3f newpose){ 
        pose = newpose;
    }
    void setPose(float x, float y, float th){ 
        pose << x,y,th; 
    }
    Vector3f getPose(){ 
        return pose; 
    }

    /** Convert polar coordinates to cartesian coordinates */
    MatrixXf polarToCart(MatrixXf scanPolar){
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
    MatrixXf cartToWorld(MatrixXf scanCart, Vector3f pose){
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

    inline MatrixXf cartToWorld(MatrixXf scanCart){
        return cartToWorld(scanCart, this->pose);
    }

    /** Convert polar coordinates in the scanner frame to cartesian coordinates in the world frame directly */
    inline MatrixXf polarToWorld(MatrixXf scanPolar, Vector3f pose){
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
    inline MatrixXf polarToWorld(MatrixXf scanPolar){
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

    /** Simulate a laser scan. The lines are defined as a matrix where each column represents a line and the rows are [x1; y1; x2; y2] */
    MatrixXf simScan(Vector3f pose, MatrixXf lines){
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

    inline MatrixXf simScan(MatrixXf lines){
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
    float xoffset = 5;
    float yoffset = 5;

    int fillMode = 2;
    int clearMode = 1;

    float widthInMeters() {
        return width*cellSize;
    }
    float heightInMeters() {
        return height*cellSize;
    }

    void resize(float widthInMeters, float heightInMeters, float cellSize){
        this->cellSize = cellSize;
        this->height = ceil(heightInMeters/cellSize);
        this->width = ceil(widthInMeters/cellSize);
        this->size = width * height;
        mapData.create(height,width,CV_8SC1);
        dialatedMap.create(height,width,CV_8SC1);
        invDialatedMap.create(height,width,CV_8SC1);
        distanceMap_32F.create(height,width,CV_32F);
        distanceMap.create(height,width,CV_8SC1);
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
                // point << y, x;
                point.x = y;
                point.y = x;
            } else {
                // point << x, y;
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
        int rows = mapData.rows;
        int cols = mapData.cols;
        x = x+xoffset;
        y = y+yoffset;
        Point cell;
        cell.x = x/cellSize;
        cell.y = rows - floor(y/cellSize + 1);
        return cell;
    }

    /*
    inline Point wrap(Point cell){
        int rows = mapData.rows;
        int cols = mapData.cols;
        // Wrapping the map
        while(cell.x < 0)       cell.x += cols;
        while(cell.x >= cols)   cell.x -= cols;
        while(cell.y < 0)       cell.y += rows;    
        while(cell.y >= rows)   cell.y -= rows;
        return cell;
    }
    */

    void setCell(int x, int y, char value, int mode){
        int rows = mapData.rows;
        int cols = mapData.cols;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
        }
        

        if((0 <= x && x < cols) && (0 <= y && y < rows)){
            mapData.at<char>(y,x) = value;
            if(mode > 0){
                if(0 <= x-1)    mapData.at<char>(y,x-1) = value; 
                if(x+1 < cols)  mapData.at<char>(y,x+1) = value; 
                if(0 <= y-1)    mapData.at<char>(y-1,x) = value; 
                if(y+1 < rows)  mapData.at<char>(y+1,x) = value; 
            }
            if(mode > 1){
                if(0 <= x-1 && 0 <= y-1)    mapData.at<char>(y-1,x-1)=value;
                if(0 <= x-1 && y+1 < rows)  mapData.at<char>(y+1,x-1)=value; 
                if(x+1 < cols && 0 <= y-1)  mapData.at<char>(y-1,x+1)=value; 
                if(x+1 < cols && y+1 < rows)mapData.at<char>(y+1,x+1)=value;
            }
            // cout << "SetCell (" << x << "," << y << ") = " << int(value) << ",mode=" << mode << endl;
        }
    }

    char mapAt(int x, int y){
        int rows = mapData.rows;
        int cols = mapData.cols;

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
        int rows = mapData.rows;
        int cols = mapData.cols;

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

        return 0;
    }

    // store the converted scans globally as they are used by several funcitons,
    // maybe store them in LaserScanner ?
    MatrixXf scanPolar;
    MatrixXf scanWorld;
    Point scanPosCell;

    void updateMap(LaserScanner *scan, float maxLSDist){
        float x = scan->pose(0);
        float y = scan->pose(1);

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
            float xs = scanWorld(0,i);
            float ys = scanWorld(1,i);
            float rho = scanPolar(1,i);
            Point cell = findCell(xs, ys);
            
            // Clear cells using bresenham's algorithm
            // cout << "ScanPosCell: (" << scanPosCell.x << "," << scanPosCell.y << "), cell: (" << cell.x << "," << cell.y << ")" << endl; 
            vector<Point> pointsToClear = bresenhamPoints(scanPosCell.x, scanPosCell.y, cell.x, cell.y);
            // cout << "Number of Points To Clear = " << pointsToClear.size() << endl; 

            for(uint ii=0; ii<pointsToClear.size(); ii++){
                setCell(pointsToClear[ii].x, pointsToClear[ii].y, 0, clearMode);
            }
            
            /* LineIterator is commented out because it cannot be used for wrapping the map
            // Clear cells using bresenham's algorithm
            LineIterator it(mapData,scanPosCell,cell,8);
            for(int j=0; j< it.count; j++, ++it){
                Point pt = it.pos();
                setCell(pt.x, pt.y, 0, clearMode);
            }
            */
            
            if(rho <= maxLSDist) pointsToFill.push_back(cell);
        }

        for(uint ii=0; ii<pointsToFill.size(); ii++){
            setCell(pointsToFill[ii].x, pointsToFill[ii].y, 1, fillMode);
        }

        // cout << "scanWorldCell:\n" << scanWorldCell.transpose() << endl;
    }

    void transform(){
        dilate( mapData>0, dialatedMap, strel);
        bitwise_not(dialatedMap, invDialatedMap);
        distanceTransform(invDialatedMap, distanceMap_32F, DIST_L2, 5, CV_32F);
        distanceMap_32F.convertTo(distanceMap,dialatedMap.type());
    }

    /** Determine next waypoint defined as the furthest point seen by the laserscanner, 
    not occupied in the dialated map, with preference for scanlines with small angles, achieved by 
    iterating from the middle and out and only replacing if a longer distance is found. */
    Vector2f determineNextWaypoint(LaserScanner *scan){
        Vector2f waypoint; // Output
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
                    float xs = scanWorld(0,k);
                    float ys = scanWorld(1,k);
                    float rho = scanPolar(1,k);
                    if(rho < biggestDistanceInCells*cellSize) continue;
                    Point cell = findCell(xs, ys);

                    // Clear search for furthest available point using Bresenham's algorithm
                    vector<Point> points = bresenhamPoints(scanPosCell.x, scanPosCell.y, cell.x, cell.y);
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

        // Translate from map coordinates to world coordinates
        waypoint << mostDistantCell.x * cellSize - cellSize/2 - xoffset,
                    dialatedMap.rows * cellSize - (mostDistantCell.y * cellSize - cellSize/2) - yoffset;

        return waypoint;
    }
    




};



#endif

