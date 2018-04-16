 /**	\file astar.h
 * \brief Implementation of the A* search Algorithm.
 *
 * \author Hjalte Bested Møller
 * \date 4. April 2018	
*/

#ifndef ASTAR_H
#define ASTAR_H

using namespace std;

const int ALLOW_DIAGONAL_PASSTHROUGH = 1;
const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const bool DEBUG = false;			// Zero means it is disabled completely


class MapSize {
public:
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    MapSize() { }
    MapSize(unsigned long width, unsigned long height) {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }
};

class MapNode {
public:
    int x = -1;	// x-position
    int y = -1; // y-position
    int h = 0;	// Heuristic cost
    int g = 0;  // Cost from start to node
    int obstdist = 0;	// Terrain cost
    int type = NODE_TYPE_ZERO;
    int flag = NODE_FLAG_UNDEFINED;
    MapNode *parent = NULL;

    MapNode() { }

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int obstdist = 0, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = NULL){
        this->x = x;
        this->y = y;
        this->obstdist = obstdist;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    int f(){
        return g + h;
    }

    void clear(){
    	this->h = 0;
    	this->g = 0;
        this->obstdist = 0;
    	this->type = NODE_TYPE_ZERO;
    	this->flag = NODE_FLAG_UNDEFINED;
    	this->parent = NULL;
    }

    void print(){
    	cout << "MapNode(x=" << x << ",y=" << y << ",type=" << type << ",obstdist=" << obstdist << ",flag=" << flag << ",parent=" << parent << ")" << endl;
    }
};

class Astar {
public:

	int G_DIRECT_COST   = 1000; /// Cost of moving to a direct neighbor cell 
	int G_DIAGONAL_COST = 1414;	/// Cost of moving to a diagonal neighbor cell, i.e. ≈ G_DIRECT_COST * sqrt(2);
	int H_AMITGAIN = 2;			/// Gain for the tie-breaker. Zero means it is disabled completely.
	int G_OBST_COST = 1000;		/// The obstacle distance cost is: G_OBST_COST/obstdist.
	int G_OBST_THRESH = 5;		/// Obstacel distance cost is only active when: obstdist < G_OBST_THRESH


	MapSize mapSize;				/// Class for storing the size of the Map
	vector<MapNode> mapData;		/// The actual map data
	vector<MapNode *> openList; 	
	vector<MapNode *> closedList; 	/// The closedList is not really needed for the algorithm to run, but is handy for visualising which nodes was examined in the path determination
	MapNode *startNode;
	MapNode *targetNode;
	bool wrapMap = true;
	
	Astar() { }

    Astar(unsigned long width, unsigned long height) {
    	this->mapSize = MapSize(width,height);
    	this->mapData.resize(mapSize.size);
    	for(int y=0; y<mapSize.height; y++){
    		for(int x=0; x<mapSize.width; x++){
    			MapNode * node = this->mapAt(x,y);
    			node->x = x;
    			node->y = y;
    		}
    	}
    	if(DEBUG) cout << "MapSize(" << mapSize.width << ", " << mapSize.height << ", " << mapSize.size << ")" << endl;
    }

	/**	The standard heuristic is the Manhattan distance. 
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
		return max(abs(node2->x - node1->x),abs(node2->y - node1->y));
	}

	/**	If your units can move at any angle (instead of grid directions), 
	 *	then you should probably use a straight line distance: */
	inline int straight_distance(MapNode* node1, MapNode* node2){
		int dx = (node2->x - node1->x);
		int dy = (node2->y - node1->y);
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

	/** Compute the heuristic cost, or the estimated cost from 
	*	current node (node1) to goal node (node2). */
	inline int computeH(MapNode *node1, MapNode *node2){
	    if (ALLOW_DIAGONAL_PASSTHROUGH) {
	        return diagonal_distance(node1, node2)  * G_DIAGONAL_COST;
	    } else {
	        return manhattan_distance(node1, node2) * G_DIRECT_COST;
	    }
	}

	/** Compute the cost from startnode (node1) to current node (node2). */
	inline int computeG(MapNode *node1, MapNode *node2) {
		int obstCost=0;
		if(node2->obstdist > 0 && node2->obstdist < G_OBST_THRESH){
			obstCost = ceil(G_OBST_COST/node2->obstdist);
		}
	    if(node1->x != node2->x && node1->y != node2->y) 
	    	return G_DIAGONAL_COST	+ 2*obstCost;
	    else 
	    	return G_DIRECT_COST	+ obstCost;
	}

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

	/** Return a pointer to the MapNode at (x,y) */
	MapNode *mapAt(int x, int y){
		int cols = mapSize.width;
		int rows = mapSize.height;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;
            while(y >= rows)   y -= rows;
        }

    	if (x < 0 || y < 0 || x >= mapSize.width || y >= mapSize.height) return 0;
    	return &mapData[y * mapSize.width + x];
	}

	/** Put a note into the map, the location if defined by the node itself! */
	void putNode(MapNode node){
		int cols = mapSize.width;
		int rows = mapSize.height;
		int x = node.x;
		int y = node.y;

        // Wrapping the map
        if(wrapMap){
            while(x < 0)       x += cols;
            while(x >= cols)   x -= cols;
            while(y < 0)       y += rows;    
            while(y >= rows)   y -= rows;
        }

		if(x >= 0 && x < cols && y >= 0 && y < rows){
			mapData[y * mapSize.width + x] = node;
		}
	}

	void clearNodeAt(int x, int y){
		MapNode * node = mapAt(x,y);
		if(node != 0) node->clear();
	}

	void setNodeTypeAt(int x, int y, int type){
		MapNode * node = mapAt(x,y);
		if(node != 0) node->type = type;
	}

	void setObstDistAt(int x, int y, int obstdist){
		MapNode * node = mapAt(x,y);
		if(node != 0) node->obstdist = obstdist;
	}

	/** Find the path with the minimal total cost. The actual A* search!  */
	vector<MapNode *> findpath(unsigned long maxIter = 1e4) {
	    vector<MapNode *> path;
	    // if(startNode !=0 || targetNode != 0 || startNode != targetNode) return path;

	    if(DEBUG) cout << "Finding started!" << endl;
	    int iteration = 0;
	    MapNode *node;
	    MapNode *reversedPtr = 0;

	    while (openList.size() > 0) {
	        node = openList.at(0);

	        for (int i = 0, max = openList.size(); i < max; i++) {
	            if (openList[i]->f() <= node->f()) {
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
	            break;
	        }
	        if(iteration++ > maxIter) {
	        	cout << "MaxIteration Reached. Returning" << endl;
	            reversedPtr = node;
	            break;
	        }

	        vector<MapNode *> neighborNodes = neighbors(node);
	         if(DEBUG) cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
	        for (uint i = 0; i < neighborNodes.size(); i++) {
	            MapNode *_node = neighborNodes[i];
	            if ( _node->type == NODE_TYPE_OBSTACLE) {
	                continue;
	            }
	            int g = node->g + computeG(node, _node);
	            if (_node->flag == NODE_FLAG_UNDEFINED || g < _node->g) {
	                _node->g = g;
	                _node->h = computeH(_node, targetNode);
	                if(H_AMITGAIN > 0){
	             	   _node->h += amits_modifier(startNode,_node,targetNode)*H_AMITGAIN;
	            	}
	                _node->parent = node;
	                if (_node->flag != NODE_FLAG_OPEN) {
	                    _node->flag = NODE_FLAG_OPEN;
	                    openList.push_back(_node);
	                }
	            }
	        }

	        if (openList.size() <= 0) break;

	    }
	    if (reversedPtr == 0) {
	        if(DEBUG) cout << "Target node is unreachable." << endl;
	    } else {
	        MapNode *_node = reversedPtr;
	        while (_node->parent != 0) {
	            path.push_back(_node);
	            _node = _node->parent;
	        }
	        path.push_back(startNode);
	        reverse(path.begin(), path.end());
	    }
	    return path;
	}

	vector<MapNode *> findpath(int xStart, int yStart, int xTarget, int yTarget, unsigned long maxIter=1e4){
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

	vector<MapNode *> simplifyPath(vector<MapNode *> path){
        // if(path.size() < 3) return path;
        vector<MapNode *> newpath;

        // Make New Path - points are lying on the same line as the previous and next, throw it away! 
        newpath.push_back(path.front());
        for(int i=1; i<(path.size()-1); i++){
            MapNode * P0 = path[i-1];
            MapNode * P1 = path[i];
            MapNode * P2 = path[i+1];
            int dx1 = P1->x - P2->x;
            int dy1 = P1->y - P2->y;
            int dx2 = P0->x - P2->x;
            int dy2 = P0->y - P2->y;
            int cross = dx1*dy2 - dx2*dy1;
            if( cross<0 ) cross = -cross;
            if( cross > 0) newpath.push_back(P1);
        }
        newpath.push_back(path.back());
        
        /*
        cout << "simplifyPath:\n";
        for(int i=0; i<newpath.size(); i++){
            cout << "->(" << newpath[i]->x << "," << newpath[i]->y << ")";
        }
        cout << endl;
        
        for(int i=0; i<path.size(); i++){
            cout << "->(" << path[i]->x << "," << path[i]->y << ")";
        }
        cout << endl;
        */

        return newpath;
    }

	/** Find all the neighbors to the node and return a vector of MapNode pointers */
	vector<MapNode *> neighbors(MapNode *node){
	    vector<MapNode *> available;
	    MapNode *_node;
	    if ((_node = mapAt(node->x - 1, node->y)) != 0 && node->x-1 == _node->x && node->y == _node->y) available.push_back(_node); // L
	    if ((_node = mapAt(node->x, node->y - 1)) != 0 && node->x == _node->x   && node->y-1 == _node->y) available.push_back(_node); // T
	    if ((_node = mapAt(node->x + 1, node->y)) != 0 && node->x+1 == _node->x && node->y == _node->y) available.push_back(_node); // R
	    if ((_node = mapAt(node->x, node->y + 1)) != 0 && node->x == _node->x   && node->y+1 == _node->y) available.push_back(_node); // B

	    if (ALLOW_DIAGONAL_PASSTHROUGH) {
	        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0 && node->x-1 == _node->x && node->y-1 == _node->y) available.push_back(_node); // LT
	        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0 && node->x+1 == _node->x && node->y-1 == _node->y) available.push_back(_node); // RT
	        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0 && node->x+1 == _node->x && node->y+1 == _node->y) available.push_back(_node); // RB
	        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0 && node->x-1 == _node->x && node->y+1 == _node->y) available.push_back(_node); // LB
	    }
	    return available;
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


};


#endif

