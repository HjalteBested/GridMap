#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <Eigen/Dense>

#include "GridMap.h"
using namespace std;
using namespace Eigen;
using namespace cv;


MapSize mapSize;
GridMap map;
LaserScanner laserScanner;

int main(){
    cout << "Hello Dude!" << endl;
    float w = 10;
    float h = 10;
    float cellSize = 0.03;
    float maxLSDist = 3.5;


	// Define Lines in cartesian space
    MatrixXf lines(4,5);
    //lines.col(0) << 2, 1, -2, 1;
    //lines.col(1) << -2, 0.5, -2, -1;

	lines.col(0) << 0,    0.25, 18.5,  0.25;
    lines.col(1) << 0,   -0.25, 18,   -0.25;
    lines.col(2) << 18.5,  0.25, 18.5, -2;
	lines.col(3) << 18,   -0.25, 18,   -1;
	lines.col(4) << 24,   2, 	24,   -2;

	cout << "lines:\n" << lines << endl;
    
    // Setup the map
    map.resize(w,h,cellSize);
    cout << "Map:  H:" << map.heightInMeters() << "m W:" << map.widthInMeters() << "m with cellSize:" << map.cellSize << "m^2 ---> H:" << map.height << " W:" << map.width << " Size:" << map.size  << endl;

    // Make structuring element for map dialation - based on robot width, or 2*radius, or som safety margin
    float robotWidth = 0.28;
    map.makeStrel(robotWidth);


	// Create OpenCV Windows
    namedWindow( "mapData", WINDOW_AUTOSIZE );
    namedWindow( "mapDialated", WINDOW_AUTOSIZE );
    namedWindow( "mapDistance", WINDOW_AUTOSIZE );
	moveWindow("mapData", 0,0);
	moveWindow("mapDialated", map.width,0);
	moveWindow("mapDistance", 2*map.width,0);

    // Simulation
    for(int ii=0; ii<400; ii++){
	    // Set pose of the scanner
		laserScanner.setPose(0.26+ii*0.2,0,0);
		// cout << "pose:\n" << laserScanner.pose << endl;    

	    // Simumate a scan
		laserScanner.simScan(lines);

	    // Call this whenever new scandata is available
	    map.updateMap(&laserScanner, maxLSDist);
	    
	    // This following needs only to be done when a new route should be planned
	    map.transform();	// Dialate Map and Compute Distance Transform
	    Vector2f wayPoint = map.determineNextWaypoint(&laserScanner);

	    // cout << "wayPoint:" << wayPoint << endl;

	    // Show Data
	    imshow( "mapData", 	   map.mapData*127);
	    imshow( "mapDialated", map.dialatedMap);
	    imshow( "mapDistance", map.distanceMap);
	    waitKey(1);
	}
    /*
    Mat dialatedMap = map.dialate();
	Mat invDialatedMap;
	Mat distanceMap;

	cout << "dialatedMap: " << "Type: " << dialatedMap.type() << endl;
	bitwise_not(dialatedMap, invDialatedMap);
	distanceTransform(invDialatedMap, distanceMap, DIST_L2, 3, CV_32F);


	distanceMap.convertTo(distanceMap,dialatedMap.type());




	cout << "dialatedMap: " << "Type: " << dialatedMap.type() << endl;
	
	cout << "weightmap: " << "Data:\n" << distanceMap << endl;

	*/

	waitKey(0);


	

    return 0;
}
