#ifndef Project3_hpp
#define Project3_hpp

#include <stdio.h>
#include <vector>
#include "Vector2D.h"
#include "Simulator.h"
#include <set>
#include <map>
#include <math.h>

using namespace std;

#define MAX_CHANGE 0.001

class ChildPoint : public Point2D {
private:
    // if obstacle of goal
    //bool isObstacle = false;
    
public:
    bool isObstacle = false;
    ChildPoint() {}
    ChildPoint(float r, float s): Point2D(r, s){}
    double previous_value;
    double value;
    void setIsObstacle() {
        this->isObstacle = true;
    }
    
    bool getIfObstacle() {
        return this->isObstacle;
    }
    RobotAction action;
};

class Point2DC: public Point2D {
    
private:
    int x;
    int y;
    bool isObstacle;
    double state;
    double prev_state;
    int cost;
    RobotAction prevAction;
    RobotAction currAction;
    
    
public:
    bool isTarget;
    Point2DC(){}
    
    
    Point2DC(float r, float s): Point2D(r, s){}
    
    Point2DC(int x, int y, bool isObstacle, double state, int cost) {
        this->x = x;
        this->y = y;
        this->isObstacle = isObstacle;
        this->state = state;
        this->cost = cost;
    }
    
    Point2DC(int x, int y, bool isObstacle) {
        this->x = x;
        this->y = y;
        this->isObstacle = isObstacle;
    }
    Point2DC(int x, int y, bool isObstacle, double state) {
        this->x = x;
        this->y = y;
        this->isObstacle = isObstacle;
        this->state = state;
        this->cost = 0;
    }
    
    void setCurrAction(RobotAction currAction) {
        this->currAction = currAction;
    }
    
    RobotAction getCurrAction() {
        return this->currAction;
    }
    
    void setPrevAction(RobotAction prevAction) {
        this->prevAction = prevAction;
    }
    
    RobotAction getPrevAction() {
        return this->prevAction;
    }
    
    void setIfObstacle(bool isObstacle) {
        this->isObstacle = isObstacle;
    }
    bool getIfObstacle(){
        return this->isObstacle;
    }
    void setState(float value) {
        this->state = value;
    }
    float getState() {
        return this->state;
    }
    void setCost(double cost) {
        this->cost = cost;
    }
    int getCost() {
        return this->cost;
    }
    int getX() {
        return this->x;
    }
    int getY() {
        return this->y;
    }
    void printPoint2DC() {
        cout << "Point: " << this->x << ", " << this->y << endl;
    }
    int getPrevState() {
        return this->prev_state;
    }
    void setPrevState(double prev_state) {
        this->prev_state = prev_state;
    }
};
class Project3 {
    
public:
    Project3(Simulator* sim1);
    RobotAction getaction(Point2D position);
    double getValue(Point2D position);
    ChildPoint ** map_g;
    ChildPoint ** pervious_points;
    float getMoreCostVal(ChildPoint loc);
    void determineNextStep(ChildPoint &current);
    ChildPoint start;
    ChildPoint goal;
    bool pointIsReached(vector<float> valueVector);
    bool isGoodEnough;
    
    // methods
    void showPolicies(ChildPoint ** map_g);
    int max_height, max_width;
    string down_str(Point2DC point);
    map<string, Point2DC> sim_map;
    vector <Point2DC> test_points;
    Point2D target;
    vector<Point2D> obstacles;
    vector <float> clear_vector;
    RobotAction getOptimalAction(Point2D loc);
    void updateOptimalAction(ChildPoint &current, float min_value, map<string, float> tackle);
    void initializeGrid(ChildPoint ** map_g, Simulator* sim);
    float calcLeft(ChildPoint &current,  ChildPoint ** map_g);
    float calcRight(ChildPoint &current,  ChildPoint ** map_g);
    float calcUp(ChildPoint &current,  ChildPoint ** map_g);
    float calcBottom(ChildPoint &current,  ChildPoint ** map_g);
    bool reachedLimit(vector<float> val_list, vector<double> clear_list);
    void calculateUtility(Point2DC point);
    void fillMap(map<string, ChildPoint> &sim_map);
    string stringify(Point2DC point);
    int getCostBasedOnState(Point2DC point);
    void fillMap(map<string, Point2DC> &sim_map);
    void markTarget(map<string, Point2DC> &sim_map, Point2D target);
    string up_str(Point2DC point);
    string bottom_str(Point2DC point);
    string right_str(Point2DC point);
    string left_str(Point2DC point);
    string stringify(Point2D point);
    void calcStoreValues(map<string, Point2DC> &sim_map);
    vector<Point2DC> opt_values;
};


#endif
