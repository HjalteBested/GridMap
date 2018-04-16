

//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>

//#include <iostream>
//#include <Eigen/Dense>

#include "GridMap.h"
#include "RobotMPC.h"
// #include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;


// MapSize mapSize;
GridMap gridMap;
LaserScanner laserScanner;
RobotPathFollowMPC pf = RobotPathFollowMPC();

Mat mapToPlot;
Mat mapToDraw;
void drawPath(Mat &map, vector<MapNode *> path);
void drawExaminedNotes(Mat &mapToDraw);
vector<MapNode *> simplifyPath(vector<MapNode *> path);


int main(){

    // Design MPC - Compute controller gains for the horizon
    int N = 100;
    float Qth = 0.05;
    float Qdu = 0.005;
    float Qu =  0.001;
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
    float maxLSDist = 3.5f;

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

    /*

    MatrixXf lines(4,31);
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
    */
    cout << "lines:\n" << lines << endl;

    // Setup the map
    gridMap.resize(w,h,cellSize);
    cout << "Map:  H:" << gridMap.heightInMeters() << "m W:" << gridMap.widthInMeters() << "m with cellSize:" << gridMap.cellSize << "m^2 ---> H:" << gridMap.height << " W:" << gridMap.width << " Size:" << gridMap.size  << endl;
    gridMap.setOffset(0,5);


    // Make structuring element for map dialation - based on robot width, or 2*radius, or som safety margin
    float robotWidth = 0.28;
    gridMap.makeStrel(robotWidth);


	// Create OpenCV Windows
    namedWindow( "mapData", WINDOW_AUTOSIZE );
    namedWindow( "Path", WINDOW_AUTOSIZE );
	moveWindow("mapData", 0,0);
    moveWindow("Path", 2*gridMap.width,0);

    Vector3f pose; pose << 0.26, 0, 0;
    

    int nstp=2800;
    float time=0;


    // Simulation
    for(int ii=0; ii<nstp; ii++){
        pose(2) = pf.wrapToPi(pose(2));

        if(ii % 10 == 0){
        // Set pose of the scanner
        laserScanner.pose = pose;
        cout << "scanpose: (" << pose(0) << "," << pose(1) << "," << pose(2) << ")" << endl;    


	    // Simumate a scan
		laserScanner.simScan(lines);

	    // Call this whenever new scandata is available
<<<<<<< HEAD
	    map.updateMap(&laserScanner, maxLSDist);
	    mapToPlot = map.mapData*127;
=======
        gridMap.updateMap(&laserScanner, maxLSDist);
        Mat mapToPlot = gridMap.mapData*127;
>>>>>>> 3d94735bdfc1c9e7e8a5485652c1f8ade18df679
	    // This following needs only to be done when a new route should be planned
        gridMap.transform();	// Dialate Map and Compute Distance Transform
        Point wayPointCell = gridMap.determineNextWaypointCell(&laserScanner);
        // Vector2f wayPoint = map.determineNextWaypoint(&laserScanner);
        // cout << "wayPointCell = " << wayPointCell << endl;

        Point robotCell = gridMap.worldToMap(pose(0),pose(1));

        vector<MapNode *> path    = gridMap.findpath(robotCell.x, robotCell.y, wayPointCell.x, wayPointCell.y, 6000);
        if(path.size() > 1){
            vector<MapNode *> newpath = gridMap.simplifyPath(path);
            vector<Point2f> waypoints = gridMap.pathToWorld(newpath);
        
            int nWayPoints = waypoints.size();
            // if(nWayPoints > 5) nWayPoints = 5;
            MatrixX2f wp(nWayPoints,2);
            for(int i=0; i<nWayPoints; i++){
                wp(i,0) = waypoints[i].x;
                wp(i,1) = waypoints[i].y;
            }

            pf.setWaypoints(wp);
            pf.currentLine = 0;
        
        // cout << "waypoints:\n" << waypoints << endl;
        }
<<<<<<< HEAD
        
        map.dialatedMap.convertTo(mapToDraw,CV_8UC3);
=======

        gridMap.dialatedMap.convertTo(mapToDraw,CV_8UC3);
>>>>>>> 3d94735bdfc1c9e7e8a5485652c1f8ade18df679
        cvtColor(mapToDraw, mapToDraw, COLOR_GRAY2BGR);
        drawExaminedNotes(mapToDraw);
        drawPath(mapToDraw, path);

        // imshow( "mapDistance", 100/map.distanceMap);
<<<<<<< HEAD
        resize(mapToPlot, mapToPlot, Size(map.height*2, map.width*2), 0, 0, INTER_NEAREST);
        resize(mapToDraw, mapToDraw, Size(map.height*2, map.width*2), 0, 0, INTER_NEAREST);
        if(ii % 30 == 0){
            imshow( "mapData", mapToPlot);
            imshow( "Path", mapToDraw);
            waitKey(1);
        }

=======
        resize(mapToPlot, mapToPlot, Size(gridMap.height*2, gridMap.width*2), 0, 0, INTER_NEAREST);
        resize(mapToDraw, mapToDraw, Size(gridMap.height*2, gridMap.width*2), 0, 0, INTER_NEAREST);

        imshow( "mapData", mapToPlot);
        imshow( "Path", mapToDraw);
        waitKey(1);
>>>>>>> 3d94735bdfc1c9e7e8a5485652c1f8ade18df679
        }

        // Controller
        pf.compute(pose(0),pose(1),pose(2));

        // Robot Simulation
        pose = pf.kinupdate(pose, pf.uw, pf.T);
        time += pf.T;
<<<<<<< HEAD

=======
>>>>>>> 3d94735bdfc1c9e7e8a5485652c1f8ade18df679
	}
    cout << "Number of steps in simulation = " << nstp << endl;


	waitKey(0);
	

    return 0;
}



MapNode wrapNode(MapNode node, int cols, int rows){
    while(node.x < 0)       node.x += cols;
    while(node.x >= cols)   node.x -= cols;
    while(node.y < 0)       node.y += rows;
    while(node.y >= rows)   node.y -= rows;
    return node;
}

void drawPath(Mat &mapToDraw, vector<MapNode *> path) {
    cvtColor(mapToDraw, mapToDraw, COLOR_BGR2HSV);

    // DrawPath
    for (int i=0; i<path.size(); i++) {
        MapNode node = wrapNode(*path[i],gridMap.mapData.cols,gridMap.mapData.rows);
        mapToDraw.at<Vec3b>(node.y, node.x) = Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
    }

    cvtColor(mapToDraw, mapToDraw, COLOR_HSV2BGR);
    
    // Draw StartNode
    MapNode *startNode = gridMap.astar.startNode;
    if(startNode && startNode->y >= 0 && startNode->y < mapToDraw.rows && startNode->x >= 0 && startNode->x < mapToDraw.cols){
        mapToDraw.at<Vec3b>(startNode->y, startNode->x) = Vec3b(255, 0, 0);
    }

    // Draw EndNode
    MapNode *targetNode = gridMap.astar.targetNode;
    if(targetNode && targetNode->y >= 0 && targetNode->y < mapToDraw.rows && targetNode->x >= 0 && targetNode->x < mapToDraw.cols){
        mapToDraw.at<Vec3b>(targetNode->y, targetNode->x) = Vec3b(0, 0, 255);
    }

}

void drawExaminedNotes(Mat &mapToDraw) {
    // Draw Open List
    for (uint i = 0; i < gridMap.astar.openList.size(); i++) {
        MapNode node = wrapNode(*gridMap.astar.openList[i],mapToDraw.cols,mapToDraw.rows);
        mapToDraw.at<Vec3b>(node.y, node.x) = Vec3b(210, 210, 210);
    }
    // Draw Closed List
    for (uint i = 0; i < gridMap.astar.closedList.size(); i++) {
        MapNode node = wrapNode(*gridMap.astar.closedList[i],mapToDraw.cols,mapToDraw.rows);
        mapToDraw.at<Vec3b>(node.y, node.x) = Vec3b(210, 210, 210);
    }
}



