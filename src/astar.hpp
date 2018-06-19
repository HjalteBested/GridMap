 /**	\file astar.h
 * \brief Implementation of the A* search Algorithm.
 *
 * \author Hjalte Bested Møller
 * \date 4. April 2018	
*/

#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

#ifdef ASTAR_USE_OPENCV
#include <opencv2/core/core.hpp>
using namespace cv;
#endif

const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_UNKNOWN = -1;
const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const bool DEBUG = false;			// Zero means it is disabled completely

/** Astar Data Structure that maintains the size of a grid-map */
class MapSize {
public:
    unsigned long width = 0;	//!< The width of the map in cells
    unsigned long height = 0;	//!< The height of the map in cells
    unsigned long size = 0;		//!< The size of the map in cells: size = width * height

    MapSize() { }
    MapSize(unsigned long width, unsigned long height) {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }
};

/** Astar Data Structure representing a graph node */
class MapNode {
public:
	/** The x location of the node */
    int x = -1;	
    /** The y location of the node */
    int y = -1;
    /** Heuristic cost */
    uint h = 0;
    /** Cost from start to node  */
    uint g = 0; 
    /** Distance to closest obstacle - if the value is -1 during node expansion, the obstacle distance is calculated locally */
    int obstdist = -1;	
    /** Node type: -1:NODE_TYPE_UNKNOWN, 0: NODE_TYPE_ZERO, 1: NODE_TYPE_OBSTACLE, 2: NODE_TYPE_START, 3:NODE_TYPE_END */
    int type = NODE_TYPE_ZERO;
    /** Node flag: -1: NODE_FLAG_CLOSED, 0:NODE_FLAG_UNDEFINED, 1:NODE_FLAG_OPEN */
    int flag = NODE_FLAG_UNDEFINED;
    /** Pointer to the parent node - used for reconstructing the path */
    MapNode *parent = NULL;

    /** Construct an unitialized instance */
    MapNode() { }
    /** Construct an itialized instance */
    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int obstdist = -1, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = NULL){
        this->x = x;
        this->y = y;
        this->obstdist = obstdist;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    /** Compute the total cost estimate: f(n) = g(n)+h(n) */
    int f(){
        return g + h;
    }

    /** Clear the node */
    inline void clear(int type=NODE_TYPE_ZERO){
    	this->h = 0;
    	this->g = 0;
        this->obstdist = -1;
    	this->type = type;
    	this->flag = NODE_FLAG_UNDEFINED;
    	this->parent = NULL;
    }

    /** Print the node */
    void print(){
    	cout << "MapNode(x=" << x << ",y=" << y << ",type=" << type << ",obstdist=" << obstdist << ",flag=" << flag << ",parent=" << parent << ")" << endl;
    }
};

/** Implementation of A* pathfinding algorithm for grid-maps, featured with tie-breaker and obstacle repulsive potential options */
class Astar {
public:
	int G_DIRECT_COST   = 1000; //!< Cost of moving to a direct neighbor cell 
	int G_DIAGONAL_COST = 1414;	//!< Cost of moving to a diagonal neighbor cell, i.e. ≈ G_DIRECT_COST * sqrt(2);
	int G_UNKNOWN_COST  = 0;	//!< Cost of moving to an unknown neighbor cell
    int H_AMITGAIN = 0;			//!< Gain for the tie-breaker. Zero means it is disabled completely.
	int G_OBST_COST = 1000;		//!< The obstacle distance cost is: G_OBST_COST/obstdist.
	int G_OBST_THRESH = 12;		//!< Obstacel distance cost is only active when: obstdist < G_OBST_THRESH
    bool ALLOW_DIAGONAL_PASSTHROUGH = true; //!< Is diagonal passthrough allowed
	bool unknownAsObstacle = false; //!< If true: Treat unknown area as obstacle, else: Treat unknown area as free
	float krep = 20.0; //!< Scaling factor for the obstacle repulsive potential
	float p=1.5; //!< Parameter for controlling the slope of the obstacle repulsive potantial

	MapSize mapSize;				//!< Class for storing the size of the Map
	vector<MapNode> mapData;		//!< The actual map data
	vector<MapNode *> openList; 	//!< The list of open nodes (the frontier)
	vector<MapNode *> closedList; 	//!< The closedList is not really needed for the algorithm to run, but is handy for visualising which nodes was examined in the path determination
	vector<MapNode *> path;			//!< The resulting path

	MapNode *startNode=NULL;		//!< Pointer to the start node
	MapNode *targetNode=NULL;		//!< Pointer to the goal node
	bool wrapMap = false;			//!< If wrapMap = true, the map can wrap around the edges such that the map can be used locally

    bool reachedTarget=false;		//!< Did the search ever reach the target ?
	vector<MapNode *> neighborNodes; //!< Preallocated storage for the neighbors of the current node

	/** Construct uninitialized instance */
	Astar() { }

	/** Construct initialized instance */
    Astar(unsigned long width, unsigned long height) {
        this->resize(width,height);
    }

    /** Resize the internal map and all the related data structures */
    void resize(unsigned long width, unsigned long height){
        this->mapSize = MapSize(width,height);
        this->mapData.resize(mapSize.size);
        for(int y=0; y<mapSize.height; y++){
            for(int x=0; x<mapSize.width; x++){
                MapNode * node = this->mapAt(x,y);
                node->x = x;
                node->y = y;
            }
        }
		neighborNodes.reserve(8);
		openList.reserve(mapSize.size);
		closedList.reserve(mapSize.size);
        if(DEBUG) cout << "MapSize(" << mapSize.width << ", " << mapSize.height << ", " << mapSize.size << ")" << endl;
    }

	/**	The standard heuristic is the Manhattan distance: 
	 *	Look at your cost function and see what the least cost is 
	 *	for moving from one space to another. 
	 *	The heuristic should be cost times manhattan distance: */
	inline int manhattan_distance(MapNode* node1, MapNode* node2){
		return abs(node2->x - node1->x) + abs(node2->y - node1->y);
	}

	/**	If on your map you allow diagonal movement, then you need a different heuristic. 
	 *	The Manhattan distance for (4 east, 4 north) will be 8. 
	 *	However, you could simply move (4 northeast) instead, so the heuristic should be 4. 
	 *	This function handles diagonals: */
	inline int diagonal_distance(MapNode* node1, MapNode* node2){
		return max(abs(node2->x - node1->x), abs(node2->y - node1->y));
	}

	/**	If your units can move at any angle (instead of grid directions), 
	 *	then you should probably use a straight line distance: */
	inline double straight_distance(MapNode* node1, MapNode* node2){
		double dx = (node2->x - node1->x);
		double dy = (node2->y - node1->y);
		return sqrt(dx*dx + dy*dy);
	}

	/**	One thing that can lead to poor performance is ties in the heuristic. 
	 *	When several paths have the same f value, they are all explored, even 
	 *	though we only need to explore one of them. To solve this problem, we 
	 *	can add a small tie-breaker to the heuristic. This tie breaker will assign 
	 *  lower cost along the straight line from the starting node to the goal node and
	 *	can give us nicer looking paths. However, it can produce bad results in case of obstacles.
	 *	node1 = start position
	 *	node2 = current position
	 *	node3 = target position */
	inline int amits_modifier(MapNode* node1, MapNode* node2, MapNode* node3){
		int dx1 = node2->x - node3->x;
		int dy1 = node2->y - node3->y;
		int dx2 = node1->x - node3->x;
		int dy2 = node1->y - node3->y;
		int cross = dx1*dy2 - dx2*dy1;
		if( cross<0 ) return -cross;
		return cross;
	}


	/** Compute the obstacle repulsive potential Urep for a given node */
	inline float computeUrep(MapNode *node) {
		if(node->obstdist <= 0 || node->obstdist >= G_OBST_THRESH || krep <= 0) return 0.0f;
		return  krep * pow((1.0f/node->obstdist - 1.0f/G_OBST_THRESH),p);
	}

	/** Compute the heuristic cost, or the estimated cost from current node (node1) to goal node (node2). */
	inline int computeH(MapNode *node1, MapNode *node2){
	    if (ALLOW_DIAGONAL_PASSTHROUGH) {
	        return diagonal_distance(node1, node2) * G_DIAGONAL_COST;
	    } else {
	        return manhattan_distance(node1, node2) * G_DIRECT_COST;
	    }
	}

	/** Compute the cost of going from node1 to node2 */
	inline int computeG(MapNode *node1, MapNode *node2) {
		int cost = 0;
	    if(node1->x != node2->x && node1->y != node2->y) 
	    	cost = G_DIAGONAL_COST; 	// if diagonal movement
	    else 
	    	cost = G_DIRECT_COST;		// if direct movement

		if(node2->type == NODE_TYPE_UNKNOWN) cost += G_UNKNOWN_COST;
	    return cost;
	}

	/** Clear all Nodes in the open and closed list, empty the lists, set startNode=NULL and targetNode = NULL. 
	 *	This is a cheap way of preparing the object to compute a totally new route as no memory reallocation is needed. */
	void clear(){		
		// Clear all nodes in openList
		for(int i=0; i<openList.size(); i++){
			openList[i]->clear();
		}
		openList.clear();

		// Clear all nodes in closedList
		for(int i=0; i<closedList.size(); i++){
			closedList[i]->clear();
		}		
		closedList.clear();

		// Set pointers to zero
		startNode  = NULL;
	    targetNode = NULL;
	}

	/** Clear all Nodes , empty the lists, set startNode=NULL and targetNode = NULL */	
	void clearAll(){
		for(int y=0; y<mapSize.height; y++){
            for(int x=0; x<mapSize.width; x++){
                MapNode * node = this->mapAt(x,y);
                node->clear();
                node->x = x;
                node->y = y;
            }
        }
        openList.clear();
        closedList.clear();

        // Set pointers to zero
		startNode  = NULL;
	    targetNode = NULL;
	}

	/** Return a pointer to the MapNode at (x,y) */
	inline MapNode *mapAt(int x, int y){
        // Wrapping the map
        if(wrapMap){
            while(x < 0)       			x += mapSize.width;
            while(x >= mapSize.width)   x -= mapSize.width;
            while(y < 0)       			y += mapSize.height;
            while(y >= mapSize.height)  y -= mapSize.height;
            return &mapData[y * mapSize.width + x];
        }

    	if (x < 0 || y < 0 || x >= mapSize.width || y >= mapSize.height) return NULL;    	
    	return &mapData[y * mapSize.width + x];
	}

	/** Put a note into the map, the location is defined by the node itself! */
	void putNode(MapNode node){
		const int& cols = mapSize.width;
		const int& rows = mapSize.height;
		int x = node.x;
		int y = node.y;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
            mapData[y * mapSize.width + x] = node;
        } else if(x >= 0 && x < cols && y >= 0 && y < rows){
			mapData[y * mapSize.width + x] = node;
		}
	}

	/** Call clear() on the node at (x,y) */
	void clearNodeAt(int x, int y){
		MapNode * node = mapAt(x,y);
		if(node != NULL) node->clear();
	}

	/** Set the node type: 
	 *	NODE_TYPE_UNKNOWN = -1;
	 *	NODE_TYPE_ZERO = 0;
	 *	NODE_TYPE_OBSTACLE = 1;
	 *	NODE_TYPE_START = 2;
	 *	NODE_TYPE_END = 3; */
	void setNodeTypeAt(int x, int y, int type){
		MapNode * node = mapAt(x,y);
		if(node != NULL) node->type = type;
	}

	/** Set the distance to the nearest obstacle cell */
	void setObstDistAt(int x, int y, int obstdist){
		MapNode * node = mapAt(x,y);
		if(node != NULL) node->obstdist = obstdist;
	}


	/** Find the path with the minimal total cost. The actual A* search!  */
	vector<MapNode *> findpath(unsigned long const& maxIter = 1e9) {
	    path.clear();
        reachedTarget = false;

	    if(DEBUG) cout << "Finding started!" << endl;
	    int iteration = 0;
	    MapNode *node;
	    MapNode *reversedPtr = 0;

	    while (openList.size() > 0) {
	        node = openList.at(0);

	        // Should be changed to a PriorityQueue !!
	        for (int i=0; i<openList.size(); i++){
	        	if (openList[i]->f() <= node->f()){
	                node = openList[i];
	            } 
	        }
	        openList.erase(remove(openList.begin(), openList.end(), node), openList.end());
	        node->flag = NODE_FLAG_CLOSED;
	        closedList.push_back(node);

	        if(DEBUG) { 
	        	cout << iteration++ << endl;
	        	cout << "   Current node " << node->x << ", " << node->y << " ..." << endl;
	        }
	        if (node->parent != 0)
	             if(DEBUG)  cout << "       ... parent " << node->parent->x << ", " << node->parent->y << endl;
	        if (node == targetNode) {
	            if(DEBUG) cout << "Reached the target node." << endl;
	            reversedPtr = node;
                reachedTarget = true;
	            break;
	        }
	        if(iteration++ > maxIter) {
	        	cout << "MaxIteration Reached. Returning" << endl;
	        	// return path;
	            reversedPtr = node;
	            break;
	        }

	        neighborNodes = neighbors(node);
	         if(DEBUG) cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
	        for (uint i = 0; i < neighborNodes.size(); i++) {
	            MapNode *_node = neighborNodes[i];
	            if ( _node->type == NODE_TYPE_OBSTACLE) continue;
	            else if(_node->type == NODE_TYPE_UNKNOWN && unknownAsObstacle) continue;

	            if(_node->obstdist < 0 && G_OBST_THRESH > 0) _node->obstdist = computeObstDist(_node);

	            float urepmul = (1.0f + computeUrep(_node));
	            int g = node->g + computeG(node, _node) * urepmul;
	            if (_node->flag == NODE_FLAG_UNDEFINED || g < _node->g) {
	                _node->g = g;
	                _node->h = computeH(_node, targetNode);
	                if(H_AMITGAIN > 0) _node->h += amits_modifier(startNode,_node,targetNode)*H_AMITGAIN*urepmul;
	                _node->parent = node;
	                if (_node->flag != NODE_FLAG_OPEN) {
	                    _node->flag = NODE_FLAG_OPEN;
	                    openList.push_back(_node);
	                }

	            }
	        }
	      	if (openList.size() <= 0){ 
	      		reversedPtr = node;
	      		break;
	      	}

	    }
	    if (reversedPtr == 0) {
	        if(DEBUG) cout << "Target node is unreachable." << endl;
	    } else {
	        MapNode *_node = reversedPtr;
	        while (_node->parent != 0) {
	            path.push_back(_node);
	            _node = _node->parent;
	        }
	        path.push_back(startNode); // optional
	        reverse(path.begin(), path.end());
	    }
	    return path;
	}

	/** Find the path with the minimal total cost. The actual A* search!  */
	vector<MapNode *> findpath(int const& xStart, int const& yStart, int const& xTarget, int const& yTarget, unsigned long const& maxIter=1e9){
		// Set StartNode
        putNode(MapNode(xStart, yStart, 0, NODE_TYPE_START));
        startNode = mapAt(xStart, yStart);
        // cout << "startNode" << astar.startNode << endl;

        // Set TargetNode
        putNode(MapNode(xTarget, yTarget, 0, NODE_TYPE_END));
        targetNode = mapAt(xTarget, yTarget);
        // cout << "targetNode" << astar.targetNode << endl;

        // Push the startnode to the open list !
        openList.push_back(startNode);

        return findpath(maxIter);
	}

	/** Simplify the path by removing all nodes that are on a straight line, i.e. keep only the corner nodes. */
	vector<MapNode *> simplifyPath(vector<MapNode *> const& path){
		const bool DEBUG = false;
        if(path.size() < 3) return path;
        vector<MapNode *> newpath;

        // Make New Path - points are lying on the same line as the previous and next, throw it away! 
        newpath.push_back(path.front());
        for(int i=1; i<(path.size()-1); i++){
            MapNode * P0 = path[i-1];
            MapNode * P1 = path[i];
            MapNode * P2 = path[i+1];
            int dx01 = P1->x - P0->x;
            int dy01 = P1->y - P0->y;
            int dx12 = P2->x - P1->x;
            int dy12 = P2->y - P1->y;
            if(dx01 == 0 && dy01 == 0) continue;
            if( !(dx01 == dx12 && dy01 == dy12) ) newpath.push_back(P1);
        }
        newpath.push_back(path.back());

		if(DEBUG && false){
			cout << "path:--";
	        for(int i=0; i<path.size(); i++){
	            cout << "->(" << path[i]->x << "," << path[i]->y << ")";
	        }
	        cout << endl;
	    }

	    if(DEBUG){
	        cout << "newpath:--";
	        for(int i=0; i<newpath.size(); i++){
	            cout << "->(" << newpath[i]->x << "," << newpath[i]->y << ")";
	        }
	        cout << endl;
    	}

		
        return newpath;
    }

	/** Find all the neighbors to the node and return a vector of MapNode pointers */
	vector<MapNode *> neighbors(MapNode *node){
	    vector<MapNode *> available;
	    MapNode *_node;
	    if(wrapMap){
		    if ((_node = mapAt(node->x - 1, node->y)) != 0 && node->x-1 == _node->x && node->y   == _node->y) available.push_back(_node); // L
		    if ((_node = mapAt(node->x, node->y - 1)) != 0 && node->x 	== _node->x && node->y-1 == _node->y) available.push_back(_node); // T
		    if ((_node = mapAt(node->x + 1, node->y)) != 0 && node->x+1 == _node->x && node->y   == _node->y) available.push_back(_node); // R
		    if ((_node = mapAt(node->x, node->y + 1)) != 0 && node->x 	== _node->x && node->y+1 == _node->y) available.push_back(_node); // B
		    if (ALLOW_DIAGONAL_PASSTHROUGH) {
		        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0 && node->x-1 == _node->x && node->y-1 == _node->y) available.push_back(_node); // LT
		        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0 && node->x+1 == _node->x && node->y-1 == _node->y) available.push_back(_node); // RT
		        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0 && node->x+1 == _node->x && node->y+1 == _node->y) available.push_back(_node); // RB
		        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0 && node->x-1 == _node->x && node->y+1 == _node->y) available.push_back(_node); // LB
		    }
		} else {
			if ((_node = mapAt(node->x - 1, node->y)) != 0) available.push_back(_node); // L
		    if ((_node = mapAt(node->x, node->y - 1)) != 0) available.push_back(_node); // T
		    if ((_node = mapAt(node->x + 1, node->y)) != 0) available.push_back(_node); // R
		    if ((_node = mapAt(node->x, node->y + 1)) != 0) available.push_back(_node); // B
		    if (ALLOW_DIAGONAL_PASSTHROUGH) {
		        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0) available.push_back(_node); // LT
		        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0) available.push_back(_node); // RT
		        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0) available.push_back(_node); // RB
		        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0) available.push_back(_node); // LB
			}
		}
	    return available;
	}

	/** Compute the diagonal distance from node n to the nearest obstacle */
	uint computeObstDist(MapNode *node){
	    MapNode *_node;
		uint dist = 1; 		
		int x,y;
		if(node->type == NODE_TYPE_OBSTACLE) return 0;
	    while(dist < G_OBST_THRESH){   
		    x = node->x-dist;
		    for(y=node->y-dist; y<=node->y+dist; y++){
		    	if ((_node = mapAt(x,y)) != 0 && _node->x==x && _node->y==y && (_node->type == NODE_TYPE_OBSTACLE)){ _node->obstdist=dist; return dist; }
		    }
		    
		   	x = node->x+dist;
		    for(y=node->y-dist; y<=node->y+dist; y++){
		    	if ((_node = mapAt(x,y)) != 0 && _node->x==x && _node->y==y && (_node->type == NODE_TYPE_OBSTACLE)){ _node->obstdist=dist; return dist; }
		    }

		    y = node->y-dist;
		    for(x=node->x-dist+1; x<=node->x+dist-1; x++){
		    	if ((_node = mapAt(x,y)) != 0 && _node->x==x && _node->y==y && (_node->type == NODE_TYPE_OBSTACLE)){ _node->obstdist=dist; return dist; }
		    }

		    y = node->y+dist;
		    for(x=node->x-dist+1; x<=node->x+dist-1; x++){
		    	if ((_node = mapAt(x,y)) != 0 && _node->x==x && _node->y==y && (_node->type == NODE_TYPE_OBSTACLE)){ _node->obstdist=dist; return dist; }
		    }
		    dist++;
		}
		return -1;
	}

	/** Print the map */
	void printMap(){
	    for (int y = 0; y < mapSize.height; y++) {
	        for (int x = 0; x < mapSize.width; x++) {
	            cout << mapAt(x, y)->type << " ";
	        }
	        cout << endl;
	    }
	}

	/** Return a node with wrapped coordinates */
	MapNode wrapNode(MapNode node, int cols, int rows){
	    while(node.x < 0)       node.x += cols;
	    while(node.x >= cols)   node.x -= cols;
	    while(node.y < 0)       node.y += rows;
	    while(node.y >= rows)   node.y -= rows;
	    return node;
	}
	
	/** Construct a vector with all nodes that are an edge of an obstacle */
	vector<MapNode *> obstacelEdgeNodes(){
		vector<MapNode *> obstnodes;
		// Find all obstacle nodes:
		for(int i=0; i<mapData.size(); i++){
			MapNode * node = &mapData[i];
			if(node->type == NODE_TYPE_OBSTACLE){
				node->obstdist = 0;

				// Check if obstacle is an edgepoint
				neighborNodes = neighbors(node);
				if(neighborNodes.size() > 0){
					bool isEdge=false;
					for(int n=0; n<neighborNodes.size(); n++){
						if(neighborNodes[n]->type == NODE_TYPE_ZERO) isEdge=true;
					}
					if(isEdge) obstnodes.push_back(node);
				}
			}
		}
		return obstnodes;
	}


	#ifdef ASTAR_USE_OPENCV // To allow using astar without having opencv included
	/** Draw nodes as red circles. Intended for showing the simplified path nodes, enabled if ASTAR_USE_OPENCV is defined */
	void drawNodes(Mat& mapToDraw, vector<MapNode *> path, const Scalar& color=Scalar(0,0,255), int thickness=-1){
		int const& N = path.size();
		vector<Point> points(N);
		Point point;
		MapNode node;
        for(int i=0; i<N; i++){
        	node = wrapNode(*path[i], mapToDraw.cols, mapToDraw.rows);
            point.x = node.x;
            point.y = node.y;
            circle(mapToDraw, point, 1, color, thickness);
        }
	}

	/** Drawing is enabled if ASTAR_USE_OPENCV is defined */
	void drawPath(Mat &mapToDraw, bool drawExaminedNotes=true) {
	    if(drawExaminedNotes){
	        // Draw Open List
	        for (uint i = 0; i < openList.size(); i++) {
	            MapNode node = wrapNode(*openList[i], mapToDraw.cols, mapToDraw.rows);
	            mapToDraw.at<Vec3b>(node.y, node.x) = cv::Vec3b(210, 210, 210);
	        }
	        // Draw Closed List
	        for (uint i = 0; i < closedList.size(); i++) {
	            MapNode node = wrapNode(*closedList[i], mapToDraw.cols, mapToDraw.rows);
	            mapToDraw.at<Vec3b>(node.y, node.x) = cv::Vec3b(210, 210, 210);
	        }
	    }

	    // Convert to HSV colerspace
	    cvtColor(mapToDraw, mapToDraw, COLOR_BGR2HSV);

	    // DrawPath
	    for (int i=0; i<path.size(); i++) {
	        MapNode node = wrapNode(*path[i], mapToDraw.cols, mapToDraw.rows);
	        mapToDraw.at<Vec3b>(node.y, node.x) = Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
	    }

	    // Convert back to BGR colerspace
	    cvtColor(mapToDraw, mapToDraw, COLOR_HSV2BGR);
	    
	    // Draw Start Node (BLUE)
	    if(startNode){
	    	MapNode node = wrapNode(*startNode, mapToDraw.cols, mapToDraw.rows);
	        mapToDraw.at<Vec3b>(node.y, node.x) = Vec3b(255, 0, 0);
	    }

	    // Draw Target Node (RED)
	    if(targetNode){
	    	MapNode node = wrapNode(*targetNode, mapToDraw.cols, mapToDraw.rows);
	        mapToDraw.at<Vec3b>(node.y, node.x) = Vec3b(0, 0, 255);
	    }
	}
	#endif
};

#endif

