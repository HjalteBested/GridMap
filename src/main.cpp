#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
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
    vector<Vector2i> points = map.bresenhamPoints(0, 0, 30, 10);

    MatrixXf lines(4,1);
    Vector3f pose; pose << 0,0,M_PI/2;
    cout << "pose:\n" << pose << endl;
    lines << 2, 1, -2, 1;
    cout << "lines:\n" << lines << endl;

    laserScanner.simScan(pose,lines);
    /*
	for(int i=0; i<points.size(); i++){
		cout << points[i].transpose() << endl;
	}
	*/

	cout << laserScanner.scanData.rows() << "," << laserScanner.scanData.cols() << endl;
	cout << "scanData:\n" << laserScanner.scanData.transpose() << endl;


    return 0;
}
