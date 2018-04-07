#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

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
    float w = 6;
    float h = 6;
    float cellSize = 0.03;
    float maxLSDist = 4;
    int pointsPerCell = 2;
    float accuracy_factor = (1/cellSize)*pointsPerCell;

    mapSize = MapSize(w,h,cellSize);
    cout << mapSize.height << "," << mapSize.width << "," << mapSize.size << "," << mapSize.cellSize << "," << mapSize.heightInMeters() << "," << mapSize.widthInMeters() << endl;

    Vector3f pose; pose << 0,0,M_PI/2;
    cout << "pose:\n" << pose << endl;

    MatrixXf lines(4,1);
    lines << 2, 1, -2, 1;
    cout << "lines:\n" << lines << endl;

    MatrixXf scanPol   = laserScanner.simScan(pose,lines);
    MatrixXf scanCart  = laserScanner.polarToCart(scanPol);
    MatrixXf scanWorld = laserScanner.polarToWorld(scanPol,pose);



    vector<Vector2i> points = map.bresenhamPoints(0, 0, 30, 10);
    
    /*
	for(int i=0; i<points.size(); i++){
		cout << points[i].transpose() << endl;
	}
	*/

	cout << scanPol.rows() << "," << scanPol.cols() << endl;
	cout << "scanPol:\n" << scanPol.transpose() << endl;
	cout << "scanCart:\n" << scanCart.transpose() << endl;
	cout << "scanWorld:\n" << scanWorld.transpose() << endl;


    return 0;
}
