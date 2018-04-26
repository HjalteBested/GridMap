

#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>

//#include <iostream>
//#include <Eigen/Dense>
// #define USE_CV_DISTANCE_TRANSFORM

#include "GridMap.hpp"
#include "RobotMPC.h"
// #include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;


// MapSize mapSize;
GridMap gridMap;
LaserScanner laserScanner;
RobotPathFollowMPC pf = RobotPathFollowMPC();

// Mat mapToPlot;
Mat mapToDraw;

void drawPoseHist(Mat &mapToDraw, vector<Vector3f> poseHist){
}



int main(){

    // Design MPC - Compute controller gains for the horizon
    int N = 100;
    float Qth = 0.04;
    float Qdu = 0.0005;
    float Qu =  0.0008;
    pf.initMPC(N);
    pf.design(Qth, Qdu, Qu);

    // Print That Shit !
    pf.printRobot();
    pf.sys.printSys();

    pf.ks=0;
    pf.ka=0;

    // GridMap Stuff!!
    cout << "Hello Dude!" << endl;
    float w = 10;
    float h = 10;
    float cellSize = 0.03;
    float maxLSDist = 3.8f;

	// Define Lines in cartesian space
    // MatrixXf lines(4,5);
    //lines.col(0) << 2, 1, -2, 1;
    //lines.col(1) << -2, 0.5, -2, -1;
    /*
	lines.col(0) << 0,   0.25, 3.5,  0.25;
    lines.col(1) << 0,  -0.25, 3,   -0.25;
    lines.col(2) << 3.5, 0.25, 3.5, -2;
	lines.col(3) << 3,  -0.25, 3,   -1;
    lines.col(4) << 24,  3, 24,   -3;
	cout << "lines:\n" << lines << endl;
    */

    // Walls
    /*
    MatrixXf lines(4,36);
    lines.col(0) << 0,    0.25, 3.5,    0.25;
    lines.col(1) << 0,   -0.25,   3,   -0.25;
    lines.col(2) << 3.5,  0.25, 3.5,   -2;
    lines.col(3) << 3,   -0.25,   3,   -1;
    lines.col(4) << 3,   -0.25,   3,   -1;
    lines.col(5) << 3,   -2,    3.5,   -2;
    lines.col(6) << 3,   -2,      3,   -1.8;
    lines.col(7) << 0,   -1,      3,   -1;
    lines.col(8) << 1,   -1.8,    3,   -1.8;
    lines.col(9) << 0,   -1,      0,   -4.0;
    lines.col(10) << 1,  -1.8,    1,   -2.5;
    lines.col(11) << 1,  -2.5,    3,   -2.5;
    lines.col(12) << 3,  -2.5,    3,   -4;
    lines.col(13) << 1.7,-3.45, 1.7,   -3.55;
    lines.col(14) << 0,  -3.45, 1.7,   -3.45;
    lines.col(15) << 0,  -3.55, 1.7,   -3.55;
    lines.col(16) << 2.8,-3.45,   3,   -3.45;
    lines.col(17) << 2.8,-3.45, 2.8,   -3.55;
    lines.col(18) << 2.8,-3.55,   3,   -3.55;
    lines.col(19) << 2,     -4,   3,   -4;
    lines.col(20) << 0,     -4, 1.5,   -4;
    lines.col(21) << 1.5,   -4, 1.5,   -6;
    lines.col(22) << 2,     -4,   2,   -6;
    lines.col(23) << 1,  -3.55,   1,   -4;
    lines.col(24) << 3,  -3.55,   3,   -4;
    // Obstacle1
    lines.col(25) << 0.1, -1.2,  0.4, -1.05;
    lines.col(26) << 0.4, -1.05, 0.6, -1.2;
    lines.col(27) << 0.6, -1.2,  0.5, -1.5;
    lines.col(28) << 0.5, -1.5,  0.1, -1.2;
    // Obstacle2
    lines.col(29) << 1, -3, 0.5, -2.6; 
    lines.col(30) << 1, -3, 1.2, -2.7 ; 
    lines.col(31) << 0.5, -2.6,  1.2, -2.7; 
    // Obstacle1
    lines.col(32) << 2.4, -2.8,  2.7, -2.6;
    lines.col(33) << 2.7, -2.6, 2.9, -2.8;
    lines.col(34) << 2.9, -2.8,  2.8, -3.1;
    lines.col(35) << 2.8, -3.1,  2.4, -2.8;

    */

    MatrixXd lines(4,31);
    lines.col(0) << 0,    0.5,  8,      0.5;
    lines.col(1) << 0,   -0.45, 5,      -0.45;
    lines.col(2) << 0,   -0.55, 7,      -0.55;
    lines.col(3) << 2,    0.1,  2,      -0.45;
    lines.col(4) << 2.3,  0.1,  2.3,    -0.45;
    lines.col(5) << 2,    0.1,  2.3,    0.1;
    lines.col(6) << 3,    0.5,  3,      0.1;
    lines.col(7) << 3.3,  0.5,  3.3,    0.1;
    lines.col(8) << 3,    0.1,  3.3,    0.1;
    lines.col(9) << 4,    0.1,  4,      -0.45;
    lines.col(10) << 4.3,  0.1,  4.3,    -0.45;
    lines.col(11) << 4,    0.1,  4.3,    0.1;
    // Gate
    lines.col(12) << 5,    0.5,  5,      0.3;
    lines.col(13) << 5,    0.3,  7.5,    0.3;
    lines.col(14) << 7.5,  0.5,  7.5,    0.3;
    lines.col(15) << 5,   -0.45, 5,      -0.3;
    lines.col(16) << 5,   -0.3,  7,      -0.1;
    lines.col(17) << 7,   -0.55, 7,      -0.1;
    lines.col(18) << 8,    0.5,  8,      -1;
    lines.col(19) << 1,   -1,    8,      -1;
    lines.col(20) << 0,   -0.45, 0,      -3;
    lines.col(21) << 1,   -1,    1,      -2;
    lines.col(22) << 0.1, -1.2,  0.4,    -0.65;
    lines.col(23) << 0.4, -0.65, 0.5,    -1.2;
    lines.col(24) << 0.5, -1.2,  0.4,    -1.5;
    lines.col(25) << 0.4, -1.5,  0.1,    -1.2;
    lines.col(26) << 0.4, -1.5,  0.1,    -1.2;
    lines.col(27) << 0,   -3,    7,      -3;
    lines.col(28) << 1,   -2,    8,      -2.5;
    lines.col(29) << 8,   -2.5,  8,      -4;
    lines.col(30) << 7,   -3,    7,      -4;
    
    cout << "lines:\n" << lines << endl;

    // Setup the map
    gridMap.resize(w,h,cellSize);
    cout << "Map:  H:" << gridMap.heightInMeters() << "m W:" << gridMap.widthInMeters() << "m with cellSize:" << gridMap.cellSize << "m^2 ---> H:" << gridMap.height << " W:" << gridMap.width << " Size:" << gridMap.size  << endl;
    gridMap.setOffset(0,5);


    // Make structuring element for map dialation - based on robot width, or 2*radius, or som safety margin
    float robotWidth = 0.28;
    gridMap.makeStrel(robotWidth);


	// Create OpenCV Windows
    namedWindow( "Path", WINDOW_AUTOSIZE );
    moveWindow("Path", 0,0);

    Vector3f pose; pose << 0, 0, 0;
    vector<Vector3f> postHist;
    //pose << 1.53779,-2.33605,-0.0965727;
    Vector3f scanPoseR; scanPoseR << 0.255, 0, 0;

    int nstp=2400;
    float time=0;

    gridMap.astar.unknownAsObstacle = true;

    // Simulation
    for(int ii=0; ii<nstp; ii++){
        pose(2) = pf.wrapToPi(pose(2));
        postHist.push_back(pose);

        // Set new position
        float& x  = pose(0);
        float& y  = pose(1);
        float& th = pose(2);
        float cs = cos(th);
        float sn = sin(th);

        if(ii % 5 == 0){
        // Set Laserpose in world coordinates
        laserScanner.setPose(
            x + cs*scanPoseR(0) - sn*scanPoseR(1),
            y + cs*scanPoseR(1) + sn*scanPoseR(0),
            th+scanPoseR(2));

        cout << "scanpose: (" << laserScanner.pose(0) << "," << laserScanner.pose(1) << "," << laserScanner.pose(2) << ")" << endl;    
        
        // Simumate a scan
        laserScanner.simScan(lines);

        // Convert robot world coordinate to cell coordinate
        Point robotCell = gridMap.worldToCell(x,y);
        Point scanCell  = gridMap.worldToCell(laserScanner.pose(0),laserScanner.pose(1));

	    // Call this whenever new scandata is available
        gridMap.updateMap(&laserScanner, maxLSDist);
        gridMap.setLine(robotCell.x, robotCell.y, scanCell.x, scanCell.y, 0, 1);

	    // This following needs only to be done when a new route should be planned
        gridMap.transform();	// Dialate Map and Compute Distance Transform
        Point wayPointCell = gridMap.determineNextWaypointCell(&laserScanner,3.0f);
        // Point wayPointCell = gridMap.worldToCell(3.5,-4);
        // Vector2f wayPoint = map.determineNextWaypoint(&laserScanner);
        // cout << "wayPointCell = " << wayPointCell << endl;

        vector<MapNode *> path = gridMap.findpath(robotCell.x, robotCell.y, wayPointCell.x, wayPointCell.y, 1e5);
        bool newPathFound = gridMap.astar.reachedTarget;
        vector<MapNode *> newpath;
        vector<Point2f> waypoints;
        if(newPathFound){
            newpath = gridMap.simplifyPath(path);
            waypoints = gridMap.pathToWorld(newpath);
        
            int nWayPoints = waypoints.size();
            // if(nWayPoints > 5) nWayPoints = 5;
            MatrixX2f wp(nWayPoints,2);
            for(int i=0; i<nWayPoints; i++){
                wp(i,0) = waypoints[i].x;
                wp(i,1) = waypoints[i].y;
            }

            pf.setWaypoints(wp);
            pf.currentLine = 0;        
        }

        if(ii % 10 == 0 && newPathFound){
            mapToDraw = (gridMap.mapData+1);
            mapToDraw.convertTo(mapToDraw,CV_8UC3);
            mapToDraw *= 127;
            mapToDraw += gridMap.dialatedMap * 0.15;
            
            cvtColor(mapToDraw, mapToDraw, COLOR_GRAY2BGR);
            gridMap.astar.drawPath(mapToDraw);
            gridMap.astar.drawNodes(mapToDraw, newpath);

            resize(mapToDraw, mapToDraw, Size(gridMap.height*3, gridMap.width*3), 0, 0, INTER_NEAREST);                    
            imshow( "Path", mapToDraw);
            waitKey(1);
        }

        
        }


        // Controller
        pf.compute(pose(0),pose(1),pose(2));

        // Robot Simulation
        pose = pf.kinupdate(pose, pf.uw, pf.T);
        time += pf.T;

	}
    cout << "Number of steps in simulation = " << nstp << endl;

	waitKey(0);
	

    return 0;
}



