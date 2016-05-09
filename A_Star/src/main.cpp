#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <time.h>
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#define _USE_MATH_DEFINES
#include <cmath>
#endif
#include <vector>
#include <string>
#include "Vector2D.h"
#include "Robot.h"
#include "Simulator.h"
#include "Project2.h"


#define SIZEX 10
#define SIZEY 40

int main(int argc, char **argv)
{
    Simulator* sim1; // your environment
    Robot*  r1; // your robot
    int id;     // the id of your robot
    int sx = -1, sy = -1; // the size of your environment
    int ix, iy; // the initial location of your robot
    int tx, ty; // the target location of your robot
    int steps;  // the steps before found the target
    int waitCounter = 100; // amount to wait between steps (milliseconds)
    int numObstacles = 5; /**< number of obstacles */
    int numHiddenObstacles = 5; /**< number of hidden obstacles */
    int stoppedSteps = 0; /**< number of steps stopped for */
    int diagSteps = 0; /**< number of diagonal steps */
    int horVertSteps = 0; /**< number of horizontal or vertical steps */
    bool isRobotAlive = true; /**< robot begins by being alive */
    float pathLength = 0.0; /**< path length */
    Point2D pos; // a variable for 2D position
    bool debugShowHiddenSensorData = true; /**< debug show hidden */
    
    srand((unsigned int)time(NULL)); // initialize random seed
    
    printf("\n\n*** CS360 Project 2 Begin *** \n\n");
    
    if (argc==3 && (sx=std::stoi(argv[1])) && (sy=std::stoi(argv[2]))) {
        printf("Project 2 environment size = [%d,%d]\n", sx, sy);
    } else {
        sx = SIZEX;       // use SIZEX for your environment
        sy = SIZEY;       // use SIZEY for your environment
    }
    sim1 = new Simulator(sx,sy);    // create your environment
    printf("Simulator area [%d x %d] is created\n", sx, sy);
    
    tx = 6;	 // rand() % sx;     // random target x
    ty = 30; // rand() % sy;     // random target y
    sim1->setTarget(tx, ty);
    printf("Target is set at the location [%d,%d]\n", tx, ty);
    
    // sim1->createRandomObstacles(numObstacles);
    // sim1->createRandomHiddenObstacles(numHiddenObstacles);
    sim1->genMap1();
    id = 1;             // robot id
    r1 = new Robot(id, sim1);  // create your robot
    ix = 1;  // rand()%sx;     // random initial x
    iy = 1;  // rand()%sy;     // random initial y
    sim1->setRobot(r1, ix, iy);  // place your robot in the environment
    pos = r1->getPosition();    // get the location of your robot
    printf("Robot %d is created at the location [%d,%d]\n\n", r1->getID(), (int)pos.x, (int)pos.y);
    
    /*
     id = 1;						// robot id
     r1 = new Robot(id, sim1);	// create your robot
     ix = rand()%sx;				// random initial x
     iy = rand()%sy;				// random initial y
     sim1->setRobot(r1, ix, iy); // place your robot in the environment
     pos = r1->getPosition();    // get the location of your robot
     printf("Robot %d is created at the location [%d,%d]\n\n", r1->getID(), (int)pos.x, (int)pos.y);
     sim1->createRandomObstacles(numObstacles);
     sim1->createRandomHiddenObstacles(numHiddenObstacles);
     */
    steps = 0;
    //Illustration of how to get the target position
    std::cout<<"Target Position: ("<<sim1->getTarget().x<<","<<sim1->getTarget().y<<")"<<std::endl;
    //Illustration of how to get the target radiance. This will return -1 until you are sufficiently close to the target.
    std::cout<<"Target Radiance: "<<sim1->getTargetRadiance()<<std::endl;
    
    Project2 p2(sim1); // project 2 object
    
    while (!sim1->robotFoundTarget()) {  // loop until your robot find the target
        RobotAction a;
        r1->setRobotAction((a = p2.getOptimalAction(sim1, r1)));
        if (a == STOP) {
            pathLength  += 0;
            stoppedSteps++;
        }
        else if (a == MOVE_DOWN_LEFT || a == MOVE_DOWN_RIGHT || a == MOVE_UP_LEFT || a == MOVE_UP_RIGHT) {
            pathLength += 1.5;
            diagSteps++;
        } else {
            pathLength += 1.0;
            horVertSteps++;
        }
        if (debugShowHiddenSensorData) {
            std::vector<Point2D> hs = sim1->getLocalObstacleLocations(r1->getPosition(), true);
            if (hs.size() > 0) {
                std::cout<<"Hidden object at location(s): "<<std::endl;
                for (int i=0; i<hs.size(); i++) {
                    std::cout<<"("<<hs[i].x<<","<<hs[i].y<<")"<<std::endl;
                }
            }
        }
        
        // call the simulator to move your robot and count the steps
        
        sim1->moveRobot();
        sim1->display();
        if (sim1->testForRobotDeath(r1->getPosition().x, r1->getPosition().y)) {
            std::cout<<"Robot dead. Stepped on obstacle at ("<<
            r1->getPosition().x<<","<<r1->getPosition().y<<")"<<std::endl;
            isRobotAlive = false;
            break;
        }
        steps++;
#if defined(_WIN32) || defined(_WIN64)
        Sleep(waitCounter);
#else
        usleep(1000*waitCounter);
#endif
    }
    if (isRobotAlive) printf("My robot found the target in %d steps !!! \n\n", steps);
    else printf("Robot died in %d steps\n", steps);
    printf("Number of steps stopped: %d\n", stoppedSteps);
    printf("Number of steps horizontal/vertical: %d\n", horVertSteps);
    printf("Number of steps diagonally: %d\n", diagSteps);
    
    printf("Robot path length: %f\n", pathLength);
    //Illustration of how to get the target radiance. This will return -1 until you are sufficiently close to the target.
    std::cout<<"Target Radiance: "<<sim1->getTargetRadiance()<<std::endl;
    for (int i=0; i<numObstacles; i++) {
        std::cout<<"Obstacle "<<i + 1<<" at: ("<<sim1->getKnownObstacleLocations()[i].x<<","<<
        sim1->getKnownObstacleLocations()[i].y<<")"<<std::endl;
    }
    /*for (int i=0; i<numHiddenObstacles; i++) {
     std::cout<<"Hidden Obstacle "<<i + 1<<" at: ("<<sim1->getHiddenObstacleLocations()[i].x<<","<<
     sim1->getHiddenObstacleLocations()[i].y<<")"<<std::endl;
     }*/
    delete sim1;
    delete r1;
}

