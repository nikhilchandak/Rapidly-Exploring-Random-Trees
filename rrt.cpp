#include <iostream>
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp>
#include "geometry.h"

using namespace std ; 

const int WIDTH = 800 ;
const int HEIGHT = 600 ;
const int RADIUS = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const double JUMP_SIZE = (WIDTH/100.0 * HEIGHT/100.0)/1.5;
const double DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found 

int whichRRT = 3 ; 

vector < Polygon > obstacles ; 
Point start, stop ; 
int obstacle_cnt = 1 ;

vector < Point > nodes ; 
vector < int > parent, nearby ; 
vector < double > cost, jumps ; 
int nodeCnt = 0, goalIndex = -1 ; 

vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ; 
bool pathFound = 0 ;

void getInput() {
	cout << "NOTE:" << endl ; 
	cout << "Height of screen: " << HEIGHT << " pixels." ;
	cout << " Width of screeen: " << WIDTH << " pixels." << endl ;
	cout << "Maximum distance by which algorithm jumps from one point to another: " << JUMP_SIZE << " units" << endl ;
	cout << "If you would like to change of any of these, please make modifications in code" << endl ; 
	cout << "Please provide your inputs keeping this in mind. " << endl << endl ;

	cout << "Which type of RRT would you like to watch? 1 for RRT, 2 for RRT*, 3 for Anytime RRT" << endl ;
	cin >> whichRRT ; 
	cout << "Input co-ordinates of starting and ending point respectively in this format X1 Y1 X2 Y2" << endl ;
	cin >> start.x >> start.y >> stop.x >> stop.y ;
	cout << "How many obstacles?" << endl ; 
	cin >> obstacle_cnt ; 
	
	obstacles.resize(obstacle_cnt); 
	int pnts = 0 ; Point pnt ; 
	vector < Point > poly ; 
	
	for(int i = 0; i < obstacle_cnt; i++) {
		poly.clear();
		cout << "How many points in " << i+1 << "th polygon?" << endl ; 
		cin >> pnts ; 
		poly.resize(pnts);

		cout << "Input co-ordinates of " << i+1 << "th polygon in clockwise order" << endl ;
		for(int j = 0; j < pnts; j++) {
			cin >> pnt.x >> pnt.y ; 
			obstacles[i].addPoint(pnt);
		}
	}
}

// Prepares SFML objects of starting, ending point and obstacles 
void prepareInput() {
	// Make starting and ending point circles ready 
	startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS); 
    startingPoint.setFillColor(sf::Color(208, 0, 240)); endingPoint.setFillColor(sf::Color::Blue);
    startingPoint.setPosition(start.x, start.y); endingPoint.setPosition(stop.x, stop.y);
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);

    // Prepare polygon of obstacles 
	polygons.resize(obstacle_cnt);
	for(int i = 0; i < obstacle_cnt; i++) {
		polygons[i].setPointCount(obstacles[i].pointCnt); 
		polygons[i].setFillColor(sf::Color(89, 87, 98)); 
		for(int j = 0; j < obstacles[i].pointCnt; j++) 
			polygons[i].setPoint(j, sf::Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
	}
}

void draw(sf::RenderWindow& window) {
	sf::Vertex line[2]; sf::CircleShape nodeCircle;

	// Uncomment if circular nodes are to be drawn 
	/*
	for(auto& node: nodes) {
		nodeCircle.setRadius(RADIUS/2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
		nodeCircle.setOrigin(RADIUS/2.5, RADIUS/2.5); 
		nodeCircle.setFillColor(sf::Color(0, 255, 171)); nodeCircle.setPosition(node.x, node.y); 
		window.draw(nodeCircle);
	}
	*/

	// Draw obstacles 
	for(auto& poly : polygons) window.draw(poly);

	// Draw edges between nodes 
	for(int i = (int)nodes.size() - 1; i; i--) {
		Point par = nodes[parent[i]] ; 
		line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
		line[1] = sf::Vertex(sf::Vector2f(nodes[i].x, nodes[i].y));
		window.draw(line, 2, sf::Lines);
	}

	window.draw(startingPoint); window.draw(endingPoint);

	// If destination is reached then path is retraced and drawn 
	if(pathFound) {
		int node = goalIndex; 
		while(parent[node] != node) {
			int par = parent[node];
			line[0] = sf::Vertex(sf::Vector2f(nodes[par].x, nodes[par].y));
			line[1] = sf::Vertex(sf::Vector2f(nodes[node].x, nodes[node].y));
			line[0].color = line[1].color = sf::Color::Red; // orange color 
			window.draw(line, 2, sf::Lines);
			node = par ;
		}
	}
}

template <typename T> // Returns a random number in [low, high] 
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Point a, Point b) {
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ; 
    return true ; 
}

// Returns a random point with some bias towards goal 
Point pickRandomPoint() {
    double random_sample = randomCoordinate(0.0, 1.0); 
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + Point(RADIUS, RADIUS) ;
	return Point(randomCoordinate(0, WIDTH), randomCoordinate(0, HEIGHT)); 
}

void checkDestinationReached() {
	sf::Vector2f position = endingPoint.getPosition(); 
	if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), RADIUS)) {
		pathFound = 1 ; 
		goalIndex = nodeCnt - 1;
		cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ; 
	}
}

/* Inserts nodes on the path from rootIndex till Point q such 
   that successive nodes on the path are not more than 
   JUMP_SIZE distance away */
void insertNodesInPath(int rootIndex, Point& q) {
	Point p = nodes[rootIndex] ; 
	if(!isEdgeObstacleFree(p, q)) return ;
	while(!(p == q)) {
		Point nxt = p.steer(q, JUMP_SIZE); 
		nodes.push_back(nxt); 
		parent.push_back(rootIndex);
		cost.push_back(cost[rootIndex] + distance(p, nxt));
		rootIndex = nodeCnt++ ; 
		p = nxt ; 
	}
}

/*  Rewires the parents of the tree greedily starting from 
	the new node found in this iterationsation as the parent */
void rewire() {
	int lastInserted = nodeCnt - 1 ; 
	for(auto nodeIndex: nearby) {
		int par = lastInserted, cur = nodeIndex;

		// Rewire parents as much as possible (greedily)
		while( ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS) { 
			int oldParent = parent[cur] ;
			parent[cur] = par; cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
			par = cur, cur = oldParent; 
		}
	}
}

/*	Runs one iteration of RRT depending on user choice 
	At least one new node is added on the screen each iteration. */
void RRT() {
	Point newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ; 
	int nearestIndex = 0 ; double minCost = INF; nearby.clear(); jumps.resize(nodeCnt); 

	while(!updated) {
		newPoint = pickRandomPoint(); 

		// Find nearest point to the newPoint such that the next node 
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free
		nearestPoint = *nodes.begin(); nearestIndex = 0;
		for(int i = 0; i < nodeCnt; i++) {
			if(pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while 
				cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);  

			// Make smaller jumps sometimes to facilitate passing through narrow passages 
			jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE ; 
			auto pnt = nodes[i] ; 
			if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
				nearestPoint = pnt, nearestIndex = i ; 
		}
		nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
		if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ; 

		if( (whichRRT == 1) or (!pathFound and whichRRT == 3)) {
			// This is where we don't do any RRT* optimization part 
			updated = true ;
			nodes.push_back(nextPoint); nodeCnt++;
			parent.push_back(nearestIndex);
			cost.push_back(cost[nearestIndex] + distance(nearestPoint, nextPoint));
			if(!pathFound) checkDestinationReached();
			continue ; 
		}

		// Find nearby nodes to the new node as center in ball of radius DISK_SIZE 
		for(int i = 0; i < nodeCnt; i++)
			if((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
				nearby.push_back(i);

		// Find minimum cost path to the new node 
		int par = nearestIndex; minCost = cost[par] + distance(nodes[par], nextPoint);
		for(auto nodeIndex: nearby) {
			if( ( (cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
				minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
		}

		parent.push_back(par); cost.push_back(minCost);
		nodes.push_back(nextPoint); nodeCnt++; 
		updated = true ; 
		if(!pathFound) checkDestinationReached(); 
		rewire();
	}
}

int main() {
	getInput(); prepareInput(); 
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Basic Anytime RRT");

	nodeCnt = 1; nodes.push_back(start); int iterations = 0 ; 
	parent.push_back(0); cost.push_back(0);
    sf::Time delayTime = sf::milliseconds(5);

    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
            	window.close();
            	return 0; exit(0);
            }
        }
        RRT(); iterations++;
        
		if(iterations % 500 == 0) {
			cout << "Iterations: " << iterations << endl ; 
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
			cout << endl ;
		}

		//sf::sleep(delayTime);
		window.clear();
		draw(window); 
        window.display();
    }
}

/* SOME SAMPLE INPUTS ARE SHOWN BELOW (only cin part) without any RRT preference */ 

/*
100 70
600 400
2
4
200 480
200 100
250 100
250 480
5
400 0
300 100
350 250
450 250
500 100
*/

/*
50 50
750 180
3
4
100 0
200 0
200 400
100 400
4
250 200
350 200
350 600
250 600
4
400 50
550 50
550 250
400 250
*/

/*
50 50
750 580
3
4
100 0
200 0
200 400
100 400
4
250 200
350 200
350 600
250 600
4
400 32
700 32
700 575
400 575
*/

/*
190 30
660 500
5
3
200 50
200 200
350 200
3
220 50
370 50
370 200
3
400 250
400 450
600 450
3
430 250
630 250
630 450
3
640 470
640 550
680 550
*/

/*
190 55
660 500
9
4
740 360 
750 350
680 540
670 530
3
710 290
780 350
630 380
3
520 450
620 540
349 580
4
450 70
700 70
700 240 
450 240
3
200 50
200 200
350 200
3
220 50
370 50
370 200
3
400 250
400 450
600 450
3
430 250
630 250
630 450
3
640 470
640 550
680 550
*/