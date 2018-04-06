/**	\file gridmap.h
 * \brief Implementation of the A* search Algorithm.
 *
 * \author Hjalte Bested Møller
 * \date 4. April 2018	
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
        return g + h;
    }
};

class LaserScanner {
public:
    // Vector3f pose; /// Scan Pose in world coordinates (x,y,theta);
    float maxDistance; /// Max Distance (Laser Measurement Range) in meters.
    float resolDeg;    /// Laser Scanner Resolution in Deg
    float resolRad;    /// Laser Scanner Resolution in Rad
    float fov;         /// Laser Scanner Field-of-view in Deg

    int nScanLines; /// Number of scanning lines (deduced from scan width and resolution)
    MatrixXf scanData;

    void init(){
        this->resolRad = resolDeg*M_PI/180.0;
        this->nScanLines = round(fov/resolDeg+1);
    }

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

    MatrixXf polarToWorld(MatrixXf scanPolar, Vector3f pose){
        return cartToWorld(polarToCart(scanPolar),pose);
    }

    /** Simulate a laser scan */
    MatrixXf simScan(Vector3f pose, MatrixXf lines){
        int nLines = lines.cols();
        float x = pose(0);
        float y = pose(1);
        float theta = pose(2);
        cout << "theta=" << theta << endl;
        // Global system coordinates
        VectorXf xStart(nLines);
        VectorXf yStart(nLines);
        VectorXf xEnd(nLines);
        VectorXf yEnd(nLines);

        // Laser scanner local system coordinates
        VectorXf xStartTrans(nLines);
        VectorXf yStartTrans(nLines);
        VectorXf xEndTrans(nLines);
        VectorXf yEndTrans(nLines);

        VectorXd a(nLines);
        VectorXd b(nLines);
        VectorXd c(nLines);
        scanData.resize(2,nScanLines);
        // scanData.setZero();

        // Start and end values for the lines ending points.
        for(int i=0; i<nLines; i++){
            xStart(i) = lines(0,i);
            yStart(i) = lines(1,i);
            xEnd(i)   = lines(2,i);
            yEnd(i)   = lines(3,i);
            // cout << "Line = (" << xStart(i) << "," << yStart(i) << "," << xEnd(i) << "," << yEnd(i) << ")" << endl;

            // Transformation of the lines to the laser scanner's coordinate system.

            // Lines are converted to the new coordinate system.
            float costh = cos(theta);
            float sinth = sin(theta);
            xStartTrans(i) = (xStart(i)-x)*costh + (yStart(i)-y)*sinth;
            yStartTrans(i) = (yStart(i)-y)*costh - (xStart(i)-x)*sinth; 
            xEndTrans(i) = (xEnd(i)-x)*costh   + (yEnd(i)-y)*sinth;
            yEndTrans(i) = (yEnd(i)-y)*costh   - (xEnd(i)-x)*sinth;
            
            // cout << "LineTrans = (" << xStartTrans(i) << "," << yStartTrans(i) << "," << xEndTrans(i) << "," << yEndTrans(i) << ")" << endl;

            // Starting and ending points are swapped if the x value of the 
            // starting point is bigger than the x value of the ending point.
            if( xStartTrans(i) > xEndTrans(i)){
                float temp = xStartTrans(i);
                xStartTrans(i) = xEndTrans(i);
                xEndTrans(i) = temp;
                temp = yStartTrans(i);
                yStartTrans(i) = yEndTrans(i);
                yEndTrans(i) = temp;
                cout << "Reversed" << endl;
            }
            // cout << "LineTrans = (" << xStartTrans(i) << "," << yStartTrans(i) << "," << xEndTrans(i) << "," << yEndTrans(i) << ")" << endl;

            // The line equations are calculated 
            //  a*x+ b*y=c
            a(i) = yStartTrans(i)-yEndTrans(i);
            b(i) = xEndTrans(i)-xStartTrans(i);
            double a2 = a(i)*a(i);
            double b2 = b(i)*b(i);
            double l = sqrt(a2+b2);
            a(i) = a(i)/l;
            b(i) = b(i)/l;
            c(i) = a(i)*xStartTrans(i)+b(i)*yStartTrans(i);
            cout << "Line " << i << ": " << a(i) << "x + " << b(i) << "y = " << c(i)<< endl;
        }

        /* -------------------------------------------------------------------
         * Conversion of what the laser scanner sees to polar coordinates.
         * -------------------------------------------------------------------*/
        // For each laser scanner angle
        for(int i=0; i<nScanLines; i++){
            // Laserscanerens maksimale måleafstand.
            // Laser scanner maximum measured distance maxDistance(in meters)
            // Closest distance from the laser scanner.
            double min_dist = maxDistance;

            // Aktuelle vinkel laserscanner.
            // Current laser scanner angle 
            double phi = i*resolRad-fov/2.0*M_PI/180.0; 
            // linierne gennemgås for at finde deres afstand til laserscanneren
            // Find distance from the lines to laser scanner in the current angle
            double cosphi = cos(phi);
            double sinphi = sin(phi);
            // cout << "cos(phi)=" << cosphi << ", sin(phi)=" << sinphi << endl;
            for(int j=0; j<nLines; j++){
                double temp = a(j)*cosphi + b(j)*sinphi;
                // cout << "cos(phi)=" << cosphi << ", sin(phi)=" << sinphi << endl;
                // cout << "temp=" << temp << endl;
                if(abs(temp) > 1e-9){
                    double t = c(j)/temp;
                    if(t>0 && t<min_dist){
                        if(abs(xStartTrans(j)-xEndTrans(j)) > 1e-9){
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

    LaserScanner() { 
        // Laser Parameters
        this->maxDistance = 4.0f;
        this->resolDeg = 0.36f;
        this->fov = 180.0f;
        this->init();
    } 

};

class GridMap {
public:
    GridMap() { }

    vector<Vector2i> bresenhamPoints(int x0, int y0, int x1, int y1){
        Vector2i point;
        vector<Vector2i> points;
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
                point << y, x;
            } else {
                point << x, y;
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

};



#endif

