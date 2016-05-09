#ifndef Simulator_h
#define Simulator_h

#include <stdio.h>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <iostream>
#include "Vector2D.h"

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <stdlib.h>
#endif

#define EMPTY       0
#define OCCUPIED	1
#define TARGET      2
#define OBSTACLE    3
#define DEAD		4
#define MAX_X       200
#define MAX_Y       200

#define P_CORR 50
#define P_LEFT 20
#define P_RIGHT 20
#define P_BACK 10

enum RobotAction {
    MOVE_UP = 0,
    MOVE_LEFT = 1,
    MOVE_DOWN = 2,
    MOVE_RIGHT = 3,
	NO_ACTION = 4
};

/**
 * @class Simulator
 * @brief simulator object
 */
class Simulator
{
private:
	Point2D startLoc, robotLoc, targetLoc, crashLoc;
	
    int SX, SY;  /**< environment size */	
	int AREA[MAX_X][MAX_Y];

    std::vector<Point2D> obstacleLocations;
	bool robotDead;
	
public:
    
    Simulator(int sx, int sy, int* area) {
        SX = sx>MAX_X ? MAX_X : sx;
        SY = sy>MAX_Y ? MAX_Y : sy;
		
		obstacleLocations.clear();
				
		for (int x = 0; x < SX; x++) {
			for (int y = 0; y < SY; y++) {
				AREA[x][y] = area[x*sy+y];
				if (AREA[x][y] == OBSTACLE)
					obstacleLocations.push_back(Point2D(x,y));
			}
		}
		
		robotDead = false;
    }

	int getHeight() { return SX; }
	int getWidth() { return SY; }
    std::vector<Point2D> getObstacleLocations() {return obstacleLocations;}

    void setTargetLocation(Point2D loc) {targetLoc = loc;	AREA[loc.x][loc.y] = TARGET;}
    void setStartLocation(Point2D loc) {startLoc = loc; robotLoc = loc;}

    Point2D getTargetLocation() {return targetLoc;}
	Point2D getRobotLocation()	{return robotLoc;}
	
	bool isRobotDead()	{return robotDead;}
	Point2D getCrashLocation()	{return crashLoc;}

    bool robotFoundTarget() 		{return robotLoc == targetLoc;}

    void moveRobot(RobotAction a)
    {
		if (robotDead) {
			std::cout<<"Dead robots do not move.."<<std::endl;
			return;
		}
		
		RobotAction b;
		int r = rand() % 100;
		
		// Determine the actual action
		if (r < P_CORR)
			b = a;
		else if (r < P_CORR + P_LEFT)
			b = (RobotAction)((a + 3)%4);
		else if (r < P_CORR + P_LEFT + P_RIGHT)
			b = (RobotAction)((a + 1)%4);
		else
			b = (RobotAction)((a + 2)%4);
		
		// Move the robot
		Point2D prevLoc = robotLoc;
		
		if (b == MOVE_UP)
			robotLoc.x--;
		if (b == MOVE_DOWN)
			robotLoc.x++;
		if (b == MOVE_LEFT)
			robotLoc.y--;
		if (b == MOVE_RIGHT)
			robotLoc.y++;

		if (AREA[robotLoc.x][robotLoc.y] == OBSTACLE) {
			robotDead = true;
			crashLoc = robotLoc;
			//AREA[prevLoc.x][prevLoc.y] = DEAD;
			robotLoc = prevLoc;
		}
		//else {
			//AREA[prevLoc.x][prevLoc.y] = EMPTY;
			//AREA[robotLoc.x][robotLoc.y] = OCCUPIED;
		//}
    }

	void reset() {
		robotLoc = startLoc;
		robotDead = false;
	}
	
	//display the environment on the terminal
    void display() {	
		//printf("\033c"); // clear the terminal screen
#if defined(_WIN32) || defined(_WIN64)
        system("cls");
#else
        system("clear");
#endif
        for (int x=0; x<SX; x++) {
            for (int y=0; y<SY; y++) {
				if (Point2D(x,y) == robotLoc)
					if (robotDead)
						std::cout<<"X";
					else
						std::cout<<"O";
                else if (AREA[x][y] == EMPTY)
                    std::cout<<".";
                else if (AREA[x][y] == TARGET)
                    std::cout<<"$";
                else if (AREA[x][y] == DEAD)
                    std::cout<<"X";
                else if (AREA[x][y] == OBSTACLE)
                    std::cout<<"#";
                else std::cout<<".";
            }
            std::cout<<std::endl;
			std::cout<<std::flush;
        }
    }
};


#endif /* Simulator_h */
