#include "Project2.h"
#include <cmath>
#include <algorithm>
#include <sstream>


using namespace std;

/**
* @brief default constructor
*/
Project2::Project2(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
    int init_size_x = sim1->getWidth();
    int init_size_y = sim1->getHeight();
    Child check;
    vector <Child> grid;
    
    for(int i = 0; i < init_size_x; i++) {
        for (int j = 0; j < init_size_y; j++) {
            Point2D point = *new Point2D(j, i);
            check.location = point;
            check.ifObstacle = false;
            check.repr = "."; // mockup grid
            grid.push_back(check);
        }
    }
    markDistances(grid);
    initBoundaries(sim1);
    initializeStuff(sim1);
    initObstacles(knownObstacles, sim1);
    trigger  = true;
    frame_rate_on = false;
}

void Project2::initObstacles(vector<Point2D> &knownObstacles, Simulator *sim1) {
    knownObstacles = sim1->getKnownObstacleLocations();
    for(int i = 0; i < knownObstacles.size(); i++){
        ChildPoint obstacle = ChildPoint((int)knownObstacles[i].x,(int)knownObstacles[i].y);
        existingObstacles[(int)knownObstacles[i].x][(int)knownObstacles[i].y] = obstacle;
        existingObstacles[(int)knownObstacles[i].x][(int)knownObstacles[i].y].boolean = true;
    }
}

void Project2::markDistances(vector<Child> point) {
    for (int i = 0; i < point.size(); i++) {
        cout << &point[i];
        
        if(i % 10)
            cout << "\n";
    }
    cout << "\n\n :::TESTED:::";
}

void Project2::initializeStuff(Simulator* sim1) {
    // Here, you should initialize the grid with all the known obstacles.
    const int grid_height = sim1->getHeight();
    const int grid_width = sim1->getWidth();
    
    existingObstacles = new ChildPoint*[grid_height];
    for(int i = 0; i < grid_height; i++){
        existingObstacles[i] = new ChildPoint[grid_width];
    }
    
    futureObstacles = new ChildPoint*[grid_width];
    for(int i = 0; i < grid_width; i++){
        futureObstacles[i] = new ChildPoint[grid_width];
        futureOpenList.push_back(*futureObstacles[i]); 
    }
    
    // new ones 
    openList = new ChildPoint*[grid_width];
    for(int i = 0; i < grid_width; i++){
        openList[i] = new ChildPoint[grid_width];
    }
    
    for( int i = 0; i < grid_height; i++){
        for(int j = 0; j < grid_width; j++){
            existingObstacles[i][j] = ChildPoint(i,j);
        }
    }
}

int Project2::trigerNewCounter(int &current) {
    return counter++;
}

void Project2::initBoundaries(Simulator* sim1) {
    Project2::H_FRAME = sim1->getWidth();
    Project2::W_FRAME = sim1->getHeight();
}

/**
 * @brief get optimal action
 * @param sim simulator pointer
 * @param r robot pointer
 * @return optimal action
 */
RobotAction Project2::getOptimalAction(Simulator* sim1, Robot* r1) {
    updateCurrentMap(r1->getLocalObstacleLocations());
    ChildPoint currentPos = ChildPoint(r1->getPosition().x, r1->getPosition().y);
    
    cout << "\nCURRENT ROBOT POSITION: " << r1->getPosition().x << ", " << r1->getPosition().y << endl << endl;
    
    if (!frame_rate_on) {
        cout << "Check open list: " << openList << endl;
    } else {
        // continue
    }
    
    if(trigger  == true) {
        AStar(sim1, r1);
        getPathFound();
        came_from.clear();
        value_map.clear();
        while(!successors.empty()) {
            successors.pop();
        }
        
        trigger  = false;
        frame_rate_on = true;
        notFound = false;
    }
    
    ChildPoint next = currentOptimalPath.front();
    currentOptimalPath.pop_front();
    dump_vector.front();
    //printPathFound();
    return step(currentPos, next, dump_vector);
}

void Project2::AStar(Simulator* sim1, Robot* r1){
    start = ChildPoint(r1->getPosition().x, r1->getPosition().y);
    goal = ChildPoint( sim1->getTarget().x, sim1->getTarget().y);
    
    came_from[stringify(start)] = stringify(start);
    value_map[stringify(start)] = 0;
    pair<ChildPoint, float> start_pair = make_pair(start, 0);
    successors.push(start_pair);
    dump_vector.push_back(*new ChildPoint());
    int m_cost;
    float newCost;
    float previousCost;

    while(!successors.empty())
    {
        ChildPoint current = successors.top().first; // get the first element from the queue
        successors.pop(); // remove it from the queue
        
        // a star search AStar
        if(current.x == goal.x && current.y == goal.y) {
            stored_things.clear();
            break;
        }
        stored_things.push_back(current);
        getNeighbors(current);
        for(auto next : neighbors) {
            if(abs(int(current.x - next.x)) == 1 && abs(int(current.y - next.y)) == 1){
                m_cost =  DIAGONAL_COST;
                cout << "GOTYAA" << endl;
            } else {
                m_cost = HORIZONTAL_COST;
            }
            //int w = 10;
            newCost = value_map[stringify(current)] + costCost(current, next);
            previousCost =  value_map[stringify(current)] + costCost(next, current);
            
            if(!value_map.count(stringify(next)) || newCost < value_map[stringify(next)]) {
                value_map[stringify(next)] = newCost;
                float newPriority = newCost + computeHeuristics(next, goal);
                came_from[stringify(next)] = stringify(current);
                pair<ChildPoint, float> next_pair = make_pair(next, newPriority);
                successors.push(next_pair);
            }
        }
        neighbors.clear();
        dump_vector.clear();
        
    }
}

double Project2::costCost(ChildPoint current, ChildPoint next){
    if(abs(int(current.x - next.x)) == 1 && abs(int(current.y - next.y)) == 1){
        return DIAGONAL_COST;
    }
    return HORIZONTAL_COST;
}

ChildPoint Project2::fromStringtoPoint(string pointString) {
    istringstream iss(pointString);
    vector<string> tokens;
    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(tokens));
    string x = tokens[0];
    string y = tokens[1];
    return ChildPoint(stof(x), stof(y));
}

void Project2::getPathFound(){
    ChildPoint current = goal;
    currentOptimalPath.push_back(current);
    while(current.x != start.x || current.y != start.y) {
        string pointString = came_from[stringify(current)];
        current = fromStringtoPoint(pointString);
        currentOptimalPath.push_back(current);
    }
    reverse(currentOptimalPath.begin(), currentOptimalPath.end());
    currentOptimalPath.pop_front();
}

void Project2::getNeighbors(ChildPoint current){
    
    int x = current.x;
    int y = current.y;
    int Fx = 0, Fy = 0, Fxx = 0, Fyy =0;
    
    Fx = x - 1; // x-1
    Fy = y - 1;
    Fxx = x + 1;
    Fyy = y + 1;
    ChildPoint p1 = existingObstacles[Fx][Fy];
    ChildPoint p2 = existingObstacles[Fx][y];
    ChildPoint p3 = existingObstacles[Fx][Fyy];
    ChildPoint p4 = existingObstacles[Fxx][Fy];
    ChildPoint p5 = existingObstacles[Fxx][y];
    ChildPoint p6 = existingObstacles[Fxx][Fyy];
    ChildPoint p7 = existingObstacles[x][Fyy];
    ChildPoint p8 = existingObstacles[x][Fy];
    dump_vector.push_back(p1);
    dump_vector.push_back(p2);
    dump_vector.push_back(p4);
    dump_vector.push_back(p5);
    dump_vector.push_back(p6);
    dump_vector.push_back(p7);
    dump_vector.push_back(p8);
    push_store["point1"] = p1;
    push_store["point2"] = p2;
    push_store["point3"] = p3;
    push_store["point4"] = p4;
    push_store["point5"] = p5;
    push_store["point6"] = p6;
    push_store["point7"] = p7;
    push_store["point8"] = p8;
    push_if_obstacle(push_store, neighbors);
}

string Project2::stringify(ChildPoint point){
    string point_x_str = to_string(point.x);
    string point_y_str = to_string(point.y);
    string whole = point_x_str + " " + point_y_str;
    return whole;
}

float Project2::getHValue(ChildPoint location, ChildPoint goal){
    return abs(goal.x - location.x) + abs(goal.y - location.y);
}

float Project2::computeHeuristics(ChildPoint location, ChildPoint goal)
{
    double dx = abs(location.x - goal.x);
    double dy = abs(location.y - goal.y);
    double octileDistance = 1.5 * min(dx, dy) + 1 * (max(dy, dy) - min(dx, dy));
    return octileDistance;
}

void Project2::updateCurrentMap(vector <Point2D> discoveredLocations){
    for(int i = 0; i < discoveredLocations.size(); i++){
        if(existingObstacles[(int)discoveredLocations[i].x][(int)discoveredLocations[i].y].boolean)
            continue;
        else {
            existingObstacles[(int)discoveredLocations[i].x][(int)discoveredLocations[i].y].boolean = true;
            currentOptimalPath.clear();
            open_closed.clear();
            trigger  = true;
        }
    }
}

void Project2::push_if_obstacle(map <string, ChildPoint> &push_store,  vector<ChildPoint> &neighbors) {
    if(!push_store["point1"].boolean){
        neighbors.push_back(push_store["point1"]);
    }if(!push_store["point2"].boolean) {
        neighbors.push_back(push_store["point2"]);
    } if(!push_store["point3"].boolean) {
        neighbors.push_back(push_store["point3"]);
    } if(!push_store["point4"].boolean) {
        neighbors.push_back(push_store["point4"]);
    }if(!push_store["point5"].boolean) {
        neighbors.push_back(push_store["point5"]);
    }if(!push_store["point6"].boolean) {
        neighbors.push_back(push_store["point6"]);
    } if(!push_store["point7"].boolean) {
        neighbors.push_back(push_store["point7"]);
    } if(!push_store["point8"].boolean) {
        neighbors.push_back(push_store["point8"]);
    }
}

RobotAction Project2::step(ChildPoint current, ChildPoint nextPoint,  vector<ChildPoint> &dump_vector){
    dump_vector.clear();
    
    RobotAction nextMove = STOP;
    int xNext = nextPoint.x;
    int yNext =  nextPoint.y;
    int xCurrent = current.x;
    int yCurrent = current.y;
    
    if(xNext - xCurrent == -1 && yNext - yCurrent == 0) {
        nextMove =  MOVE_UP;
    } if(xNext - xCurrent == 1 && yNext - yCurrent == 0) {
        nextMove =  MOVE_DOWN;
    } if(xNext- xCurrent == 0 && yNext - yCurrent == -1)
    {
        nextMove =  MOVE_LEFT;
    } if(xNext - xCurrent == 0 && yNext - yCurrent == 1)
    {
        nextMove =  MOVE_RIGHT;
    } if(xNext - xCurrent == -1 && yNext - yCurrent== -1)
    {
        nextMove =  MOVE_UP_LEFT;
    } if(xNext - xCurrent == -1 && yNext - yCurrent == 1)
    {
        nextMove =  MOVE_UP_RIGHT;
    } if(xNext - xCurrent== 1 && yNext - yCurrent == -1)
    {
        nextMove =  MOVE_DOWN_LEFT;
    } if(xNext - xCurrent == 1 && yNext - yCurrent == 1) {
        nextMove =  MOVE_DOWN_RIGHT;
    }
    return nextMove;
}