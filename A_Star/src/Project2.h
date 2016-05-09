#ifndef Project2_hpp
#define Project2_hpp
#include <iterator>
#include <queue>
#include <algorithm>
#include <sstream>
#include <map>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <cfloat>
#include <cstdio>
#include "Robot.h"
#include "Vector2D.h"
#include "Simulator.h"

using namespace std;


class ChildPoint : public Point2D {
private:
    // none
    
public:
    bool boolean = false;
    string description = "";
    ChildPoint() {
    
    }
    ChildPoint(double x, double y): Point2D(x, y) {
        // initializing point with given x and y
    }
    ChildPoint(double x, double y, bool boolean): Point2D(x, y) {
        this->boolean = boolean;
    }
    ChildPoint(double x, double y, bool isObstacle, string description): Point2D(x, y) {
        this->description = description;
    }
};

class MapArea : public Point2D {
    int SIZE;
    int time;
    RobotAction action;
    string time_frame;
    
private:
    // nothing
public:
    MapArea(int size, RobotAction action){
        this->SIZE = size;
        this->action = action;
    }
    
};

struct compare
{
    bool operator()(const pair <ChildPoint, double> &l, const pair <ChildPoint, double> &r)
    {
        return l.second > r.second;
    }
};

struct reverseCompare
{
    bool operator()(const pair <ChildPoint, double> &l, const pair <ChildPoint, double> &r)
    {
        return r.second > l.second;
    }
};

struct traverse
{
    string a;
    string b;
    
    bool compareTwoStrings(string a, string b) {
        return a == b;
    };
};

class Project2 {
private:
   
public:
    Project2(Simulator* sim1);
    
    struct Child {
        Point2D location;
        bool ifObstacle;
        string repr;
    };
    
    // variables
    const double DIAGONAL_COST = 1.5;
    const double HORIZONTAL_COST = 1.0;
    vector<Point2D> knownObstacles;
    int counter;
    int H_FRAME;
    int W_FRAME;
    vector<ChildPoint> stored_things; 
    string stringify(ChildPoint);
    deque<ChildPoint> currentOptimalPath;
    deque<ChildPoint> nextOptimalPath;
    map<string, float> value_map;
    ChildPoint start;
    ChildPoint goal;
    bool started;
    vector<ChildPoint> visitedList;
    vector<ChildPoint> neighbors;
    vector<ChildPoint> open_closed;
    bool trigger;
    bool frame_rate_on;
    bool notFound = true;
    bool obstacle_found;
    ChildPoint ** existingObstacles;
    ChildPoint ** futureObstacles;
    double costCost(ChildPoint current, ChildPoint next);
    ChildPoint ** openList;
    vector <ChildPoint> futureOpenList;
    
    // methods
    void push_if_obstacle(map <string, ChildPoint> &push_store,  vector<ChildPoint> &neighbors);
    void markDistances(vector<Child> points);
    void initializeStuff(Simulator* sim1);
    map <string, ChildPoint> push_store;
    void printCurrentRobotLocation(ChildPoint location);
    void initBoundaries(Simulator* sim1);
    int triggerNewCounter(int &counter);
    vector<ChildPoint> dump_vector; // identify vector with the least values
    void initObstacles(vector<Point2D> &knownObstacles, Simulator* sim1);
    RobotAction getOptimalAction(Simulator* sim1, Robot* r1);
    float getHValue(ChildPoint location, ChildPoint goal);
    float computeHeuristics(ChildPoint location, ChildPoint goal);
    map<string, string> came_from;
    priority_queue<pair<ChildPoint, float>, vector<pair<ChildPoint, float>>, compare> successors;
    void AStar(Simulator* sim1, Robot* r1);
    void getPathFound();
    int trigerNewCounter(int &counter);
    void updateCurrentMap(vector <Point2D> discoveredLocations);
    void clearDumpVector(vector<ChildPoint> &dump_vector);
    void dumpThem(vector<ChildPoint> &dump_vector);
    RobotAction step(ChildPoint current, ChildPoint nextPoint,  vector<ChildPoint> &dump_vector);
    void getNeighbors(ChildPoint current);
    ChildPoint fromStringtoPoint(string);
};

#endif
