#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <time.h>
//#include <pthread.h>
#include <vector>
#include <string>
#include "Vector2D.h"
#include "Robot.h"
#include "Simulator.h"

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#define SIZEX 10
#define SIZEY 40

using namespace std;


string findMin(int up, int left, int right, int down, int up_right, int up_left, int down_right, int down_left) {
    vector<int> list = {up, left, right, down, up_right, up_left, down_right, down_left};
    
    int min = 0;
    
    for(int i = 0; i < list.size(); i++) {
        min = list[i];
        for(int j = 0; j < list.size(); j++) {
            if(i !=j && min > list[j]) {
                min = list[j];
            }
        }
    }
    
    //cout << "DOWN_LEFT findMin: " << down_left << endl; // returns 0
    
//    cout << min << endl;
//    cout << up << endl;
//    cout << down << endl;
//    cout << right << endl;
//    cout << left << endl;
//    cout << down_right << endl;
//    cout << down_left << endl;
//    cout << up_left << endl;
//    cout << up_right << endl;
    
    if(min == up)
        return "UP";
    else if(min == left)
        return "LEFT";
    else if (min == right)
        return "RIGHT";
    else if (min == down)
        return "DOWN";
    else if (min == up_right)
        return "UP_RIGHT";
    else if (min == up_left)
        return "UP_LEFT";
    else if (min == down_left)
        return "DOWN_LEFT";
    else if (min == down_right)
        return "DOWN_RIGHT";
        
        
    return "";
    
}

string getNextStep(int xR, int yR, int xG, int yG) {
    string step = "";
    int Fdown = 0, Fright = 0, Fup = 0, Fleft = 0; // straight line distances
    int Vdown_right = 0, Vdown_left, Vup_left, Vup_right = 0; // curved line distances
    
    
    //1. *** (0, +) ~ DOWN
    int Dx = xR, Dy = yR + 1; // get location of the DOWN step
    
    cout << "xR: " <<  xR << endl;
    
    //<Fx, Fy> = <GoalX, GoalY> - <StepX, StepY>
    int Fx = xG - Dx;
    int Fy = yG - Dy;
    
    // finding distance
    Fdown = sqrt( pow(Fx, 2) + pow(Fy, 2) ); // mathflo
    
    //2. *** (+, 0) ~ RIGHT
    int Rx = xR + 1, Ry = yR;
    
    //<Fxx, Fyy> = <GoalX, GoalY> - <StepX, StepY>
    int Fxx = xG - Rx;
    int Fyy = yG - Ry;
    
    //finding distance
    Fright = sqrt( pow(Fxx, 2) + pow(Fyy, 2) ); // mathflo
    
    //3. *** (0, -) ~ UP
    int Ux = xR, Uy = yR - 1;
    
    //<Fxxx, Fyyy> = <GoalX, GoalY> - <StepX, StepY>
    int Fxxx = xG - Ux;
    int Fyyy = yG - Uy;

    // finding distance
    Fup = sqrt( pow(Fxxx, 2) + pow(Fyyy, 2) ); // mathflo
    
    //4. *** (-, 0) ~ LEFT
    int Lx = xR - 1, Ly = yR;
    
    //<Fxxxx, Fyyyy> = <GoalX, GoalY> - <StepX, StepY>
    int Fxxxx = xG - Lx;
    int Fyyyy = yG - Ly;
    
    // finding distance
    Fleft = sqrt( pow(Fxxxx, 2) + pow(Fyyyy, 2) ); // mathflo
    
    //5. *** (+, +) ~ DOWN_RIGHT
    int DRx = xR + 1, DRy = yR + 1;
    
    //<Fxxxx, Fyyyy> = <GoalX, GoalY> - <StepX, StepY>
    int Jx = xG - DRx;
    int Jy = yG - DRy;
    
    // finding distance
    Vdown_right = sqrt( pow(Jx, 2) + pow(Jy, 2) ); // mathflo
    
    //6. *** (-, -) ~ UP_LEFT
    int ULx = xR - 1, ULy = yR - 1;
    
    //<Fxxxx, Fyyyy> = <GoalX, GoalY> - <StepX, StepY>
    int Jxx = xG - ULx;
    int Jyy = yG - ULy;
    
    // finding distance
    Vup_left = sqrt( pow(Jxx, 2) + pow(Jyy, 2) ); // mathflo
    
    //7. *** (+, -) ~ UP_RIGHT
    int URx = xR + 1, URy = yR - 1;
    
    //<Fxxxx, Fyyyy> = <GoalX, GoalY> - <StepX, StepY>
    int Jxxx = xG - URx;
    int Jyyy = yG - URy;
    
    // finding distance
    Vup_right = sqrt( pow(Jxxx, 2) + pow(Jyyy, 2) ); // mathflo
    
    //8. *** (-, +) ~ DOWN_LEFT
    int DLx = xR - 1, DLy = yR + 1;
    
    //<Fxxxx, Fyyyy> = <GoalX, GoalY> - <StepX, StepY>
    int Jxxxx = xG - DLx;
    int Jyyyy = yG - DLy;
    
    // finding distance
    Vdown_left = sqrt( pow(Jxxxx, 2) + pow(Jyyyy, 2) ); // mathflo
    
    
    // F for stragiht, V for curved
    //int findMin(int up, int left, int right, int down, int up_right, int up_left, int down_right, int down_left)
    string min = findMin(Fup, Fleft, Fright, Fdown, Vup_right, Vup_left, Vdown_right, Vdown_left);
    
    step = min;
    return step;
}


int main(int argc, char **argv)
{
    Simulator* sim1; // your environment
    Robot*  r1; // your robot
    int id;     // the id of your robot
    int sx, sy; // the size of your environment
    int ix, iy; // the initial location of your robot
    int tx, ty; // the target location of your robot
    int steps;  // the steps before found the target
    int waitCounter = 100; // amount to wait between steps (milliseconds)
    Point2D pos; // a variable for 2D position
    
    srand(time(NULL)); // initialize random seed
    
    printf("\n\n*** CS360 Project 1 Begin *** \n\n");

    if (argc==3 && (sx=std::stoi(argv[1])) && (sy=std::stoi(argv[2]))) {
	printf("Project 1 environment size = [%d,%d]\n", sx, sy);
    } else {     
    	sx = SIZEX;       // use SIZEX for your environment
    	sy = SIZEY;       // use SIZEY for your environment
    }
    sim1 = new Simulator(sx,sy);    // create your environment
    printf("Simulator area [%d x %d] is created\n", sx, sy);

    tx = rand()%sx;     // random target x
    ty = rand()%sy;     // random target y
    sim1->setTarget(tx, ty);
    printf("Target is set at the location [%d,%d]\n", tx, ty);
    
    id = 1;             // robot id
    r1 = new Robot(id); // create your robot
    ix = rand()%sx;     // random initial x
    iy = rand()%sy;     // random initial y
    sim1->setRobot(r1, ix, iy); // place your robot in the environment
    pos = r1->getPosition();    // get the location of your robot
    printf("Robot %d is created at the location [%d,%d]\n\n", r1->getID(), (int)pos.x, (int)pos.y);
    steps = 0;
    //Illustration of how to get the target position
    std::cout<<"Target Position: ("<<sim1->getTarget().x<<","<<sim1->getTarget().y<<")"<<std::endl;
    //Illustration of how to get the target radiance. This will return -1 until you are sufficiently close to the target.
    std::cout<<"Target Radiance: "<<sim1->getTargetRadiance()<<std::endl;
    while (!sim1->robotFoundTarget()) {  // loop until your robot find the target
        
        /********************************************************************************/
        ///TODO: Below is the code you must modify for your robot to find the target quickly
        // You move your robot by giving it an action
        // The robot must decide its actions intelligently to find the target quickly
        // Below is an example of random actions
//        r1->setRobotAction((RobotAction)(rand()%8));
        
        
        // code starts here
        //cout << " (" <<r1->getPosition().x << ", " << r1->getPosition().y << ")" << endl;
        
        int xCurrentRobot = r1->getPosition().y;
        int yCurrentRobot = r1->getPosition().x;
        int xTarget = sim1->getTarget().y;
        int yTarget = sim1->getTarget().x;
        cout << "r1->GetPosition().x: " << xCurrentRobot << endl;
        
        string nextStep = getNextStep(xCurrentRobot, yCurrentRobot, xTarget, yTarget);
        
        cout << nextStep << endl;
        
        if(nextStep == "UP")
            r1->setRobotAction(MOVE_UP); // up
        else if(nextStep == "LEFT")
            r1->setRobotAction(MOVE_LEFT); // left
        else if (nextStep == "RIGHT")
            r1->setRobotAction(MOVE_RIGHT); // right
        else if (nextStep == "DOWN")
            r1->setRobotAction(MOVE_DOWN); // down
        else if (nextStep == "UP_RIGHT")
            r1->setRobotAction(MOVE_UP_RIGHT); // up_right
        else if (nextStep == "UP_LEFT")
            r1->setRobotAction(MOVE_UP_LEFT); // up_left
        else if (nextStep == "DOWN_LEFT")
            r1->setRobotAction(MOVE_DOWN_LEFT); // down_left
        else if (nextStep == "DOWN_RIGHT")
            r1->setRobotAction(MOVE_DOWN_RIGHT); // down_right
        
        //r1->setRobotAction(MOVE_UP_RIGHT);
        
        
        //r1->setForce(100,100);    // set the force for your robot
        ///END TODO: No more editing after this point
        /********************************************************************************/
        
        // call the simulator to move your robot and count the steps
        
        
        sim1->moveRobot();
        sim1->display();
        steps++;
    
        
        
        #if defined(_WIN32) || defined(_WIN64)
        Sleep(waitCounter);
        #else
        usleep(1000*waitCounter);
        #endif
    }
    printf("My robot found the target in %d steps !!! \n\n", steps);
    //Illustration of how to get the target radiance. This will return -1 until you are sufficiently close to the target.
    std::cout<<"Target Radiance: "<<sim1->getTargetRadiance()<<std::endl;
    delete sim1;
    delete r1;
}

