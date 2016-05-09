#include "Project3.h"
#include <algorithm>


Project3::Project3(Simulator* sim) {
    // 1. Query the simulator for dimensions of the map.
    obstacles.clear();
    obstacles = sim->getObstacleLocations();
    max_height = sim->getHeight();
    max_width = sim->getWidth();
    
    // 2. Construct MDP.
    obstacles = sim->getObstacleLocations();
    target = sim->getTargetLocation();
    for(int i = 0; i < obstacles.size(); i++) {
        string temp_p = to_string(obstacles[i].y) + "," + to_string(obstacles[i].x);
        sim_map[temp_p] = *new Point2DC(obstacles[i].y, obstacles[i].x, true, 0.0, 101); //Point2DC(int x, int y, bool isObstacle, double state, int cost) {
        sim_map[temp_p].setPrevState(10000);
    }
    
    
    map_g = new ChildPoint*[sim->getHeight()];
    for(int i = 0; i < sim->getHeight(); i++){
        map_g[i] = new ChildPoint[sim->getWidth()];
    }
    
    for( int i = 0; i < sim->getHeight(); i++){
        for(int j = 0; j < sim->getWidth(); j++){
            test_points.push_back(*new Point2DC(i, j, true, 0.0, 101)); //Point2DC
            map_g[i][j] = ChildPoint(i,j);
            map_g[i][j].value = 0.0;
            map_g[i][j].previous_value = 10000;
            
        }
    }
    string loc = "";
    ChildPoint obstacle;
    for(int i = 0; i < sim->getObstacleLocations().size(); i++){
        int x = obstacles[i].x;
        int y = obstacles[i].y;
        loc = stringify(ChildPoint(x, y));
        obstacle = ChildPoint(x, y);
        map_g[x][y] = obstacle;
        map_g[x][y].setIsObstacle();
    }
    
    fillMap(sim_map);
    
    vector<float> delta_change;
    int x_target = sim->getTargetLocation().x;
    int y_target = sim->getTargetLocation().y;
    int x_robot = sim->getRobotLocation().x;
    int y_robot = sim->getRobotLocation().y;
    goal = ChildPoint(x_target, y_target);
    goal.isObstacle = true;
    start = ChildPoint(x_robot, y_robot);
    
    
    // construct MDP here
    
    
    // ALGORITHM
    
    int itr = 0;
    while(!isGoodEnough)
    {
        delta_change.clear();
        clear_vector.clear();
        for(int i = 0; i < sim->getHeight(); i++){
            for(int j = 0; j < sim->getWidth(); j++){
                if(map_g[i][j].isObstacle)
                    continue;
                determineNextStep(map_g[i][j]);
                
                // calculate the difference
                float delta_a = fabs(map_g[i][j].previous_value - map_g[i][j].value);
                
                // for test purpuses: (check max)
                float max_point = max(map_g[i][j].previous_value, map_g[i][j].value);
                
                // store the difference to the vector
                delta_change.push_back(delta_a);
                
                // set the previous value be equal to current value
                map_g[i][j].previous_value = map_g[i][j].value;
                clear_vector.push_back(max_point);
            }
        }
        itr++;
        isGoodEnough = pointIsReached(delta_change);
    }
    cout << "\n";
    
    //showPolicies(map_g);
    
}

void initializemap_g(ChildPoint ** map_g) {
    
}


void Project3::showPolicies(ChildPoint ** map_g) {
    // showPolicies(map_g);
    for (int i = 0; i < max_height; i++) {
        for (int j = 0; j < max_width; j++) {
            if(!map_g[i][j].getIfObstacle()) {
                if(map_g[i][j].action == MOVE_UP) {
                    std::cout << "⇑";
                }
                if(map_g[i][j].action == MOVE_DOWN) {
                    cout << "⇓";
                }
                if(map_g[i][j].action == MOVE_LEFT) {
                    cout << "⇐";
                }
                if(map_g[i][j].action == MOVE_RIGHT) {
                    cout << "⇒";
                }
            }
        }
        cout<< "\n";
    }
    cout<< "\n";
}

bool Project3::pointIsReached(vector<float> list)
{
    double default_value = 0;
    if(list.empty())
        return false;
    
    if(list.empty() && clear_vector.empty())
        default_value = 1.0;
    
    for(auto itr : list) {
        if(itr > MAX_CHANGE) {
            return false;
        }
    }
    
    return true;
}


RobotAction Project3::getOptimalAction(Point2D loc) {
    // Here, you should return the best action for Wheelbot, assuming it is currently at 'loc'.
    cout << "getOptimalAction: " << map_g[loc.x][loc.y].value << endl; // same as get value
    return map_g[loc.x][loc.y].action;
    
}

void Project3::fillMap(map<string, Point2DC> &sim_map) {
    for(int i = 0; i < max_width; i++) {
        for(int j = 0; j < max_height; j++) {
            string key = to_string(i) + "," + to_string(j);
            if(!sim_map.count(key)) {
                sim_map[key] = *new Point2DC(i, j, false, 0.0, 1);// add point to the sim_map
                sim_map[key].setPrevState(1000);
            }
        }
    }
    
}

float Project3::getMoreCostVal(ChildPoint loc)
{
    int value = 1;
    
    if(loc.getIfObstacle()) {
        if(loc.x == goal.x && loc.y == goal.y)
            value = 0;
        value = 101;
    }
    
    return value;
}

void Project3::determineNextStep(ChildPoint &current){
    float up, bottom, left, right;
    
    up = calcUp(current, map_g);
    bottom = calcBottom(current, map_g);
    left = calcLeft(current, map_g);
    right = calcRight(current, map_g);
    
    map<string, float> tackle;
    tackle["up"] = up;
    tackle["right"] = right;
    tackle["left"] = left;
    tackle["bottom"] = bottom;
    
    updateOptimalAction(current, min({up, bottom, left, right}),  tackle);
    
}

void Project3::updateOptimalAction(ChildPoint &current, float min_value, map<string, float> tackle) {
    current.value = min_value;
    
    if(min_value == tackle["up"]){
        current.action = MOVE_UP;
    } if(min_value == tackle["bottom"]){
        current.action = MOVE_DOWN;
    } if(min_value == tackle["left"]) {
        current.action = MOVE_LEFT;
    } if(min_value == tackle["right"]) {
        current.action = MOVE_RIGHT;
    }
    
    clear_vector.clear();
    
}

float Project3::calcLeft(ChildPoint &current,  ChildPoint **map_g) {
    float val = 0.0;
    
    val = 0.2 * (map_g[current.x - 1][current.y].value + getMoreCostVal(map_g[current.x - 1][current.y]))
    + 0.2 * (map_g[current.x + 1][current.y].value + getMoreCostVal(map_g[current.x + 1][current.y]))
    + 0.5 * (map_g[current.x][current.y - 1].value + getMoreCostVal(map_g[current.x][current.y - 1]))
    + 0.1 * (map_g[current.x][current.y + 1].value + getMoreCostVal(map_g[current.x][current.y + 1]));
    
    return val;
    
}

float Project3::calcRight(ChildPoint &current,  ChildPoint ** map_g) {
    float val = 0.0;
    val = 0.2 * (map_g[current.x - 1][current.y].value + getMoreCostVal(map_g[current.x - 1][current.y]))
    + 0.2 * (map_g[current.x + 1][current.y].value + getMoreCostVal(map_g[current.x + 1][current.y]))
    + 0.1 * (map_g[current.x][current.y - 1].value + getMoreCostVal(map_g[current.x][current.y - 1]))
    + 0.5 * (map_g[current.x][current.y + 1].value + getMoreCostVal(map_g[current.x][current.y + 1]));
    
    return val;
}

float Project3::calcBottom(ChildPoint &current,  ChildPoint ** map_g) {
    float val = 0.0;
    val = 0.1 * (map_g[current.x - 1][current.y].value + getMoreCostVal(map_g[current.x - 1][current.y]))
    + 0.5 * (map_g[current.x + 1][current.y].value + getMoreCostVal(map_g[current.x + 1][current.y]))
    + 0.2 * (map_g[current.x][current.y - 1].value + getMoreCostVal(map_g[current.x][current.y - 1]))
    + 0.2 * (map_g[current.x][current.y + 1].value + getMoreCostVal(map_g[current.x][current.y + 1]));
    
    return val;
}

float Project3::calcUp(ChildPoint &current,  ChildPoint ** map_g) {
    float val = 0.0;
    val = 0.5 * (map_g[current.x - 1][current.y].value + getMoreCostVal(map_g[current.x - 1][current.y]))
    + 0.1 * (map_g[current.x + 1][current.y].value + getMoreCostVal(map_g[current.x + 1][current.y]))
    + 0.2 * (map_g[current.x][current.y - 1].value + getMoreCostVal(map_g[current.x][current.y - 1]))
    + 0.2 * (map_g[current.x][current.y + 1].value + getMoreCostVal(map_g[current.x][current.y + 1]));
    
    return val;
    
}

double Project3::getValue(Point2D loc) {
    // Here, you should return the value of the state corresponding to 'loc'.
    cout << "getValue: " << map_g[loc.x][loc.y].value << endl;
    int x = loc.x, y = loc.y;
    return map_g[x][y].value;
    
}

string Project3::up_str(Point2DC point) {
    string str = to_string(point.getX()) + "," + to_string(point.getY() - 1);
    return str;
}

string Project3::down_str(Point2DC point) {
    string str = to_string(point.getX()) + "," + to_string(point.getY() + 1);
    return str;
}

string Project3::right_str(Point2DC point) {
    string str = to_string(point.getX() + 1) + "," + to_string(point.getY());
    return str;
}

string Project3::left_str(Point2DC point) {
    string str = to_string(point.getX() - 1) + "," + to_string(point.getY());
    return str;
}

string Project3::stringify(Point2DC point) {
    string str_point = "";
    str_point = to_string(point.getX()) + "," + to_string(point.getY());
    return str_point;
}

string Project3::stringify(Point2D point) {
    string str_point = "";
    str_point = to_string(point.y) + "," + to_string(point.x); // (y,x) -> (x,y)
    return str_point;
}
