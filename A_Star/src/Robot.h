#ifndef Robot_h
#define Robot_h
#include <stdio.h>
#include <vector>
#include "Vector2D.h"

class Simulator;

/**
 * @enum RobotAction
 * @brief enumeration of robot actions
 */
enum RobotAction {
    MOVE_UP,
    MOVE_DOWN,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UP_RIGHT,
    MOVE_UP_LEFT,
    MOVE_DOWN_RIGHT,
    MOVE_DOWN_LEFT,
    STOP
};
/**
 * @class Robot
 * @brief representation of robot on 2d grid
 */
class Robot
{
private:
	Simulator* sim;	/**< Simulator that the robot is running in */

public:
	int ID; /**< ID number */
    int X, Y; /**< Position */
    int VX, VY; /**< Velocity */
    int FX, FY; /**< Force on Robot */
    int SensorR; /**< Sensor radius */
    bool WallFollowing; /**< Am in wall following mode */
    double VNoise; /**< velocity noise */
    int Role; /**< role of robot */

    /**
     * @brief constructor
     * @param IDNum id number
     */
    Robot(int IDNum, Simulator* _sim) {
		sim = _sim;
		ID = IDNum;
    }
    /**
     * @brief constructor
     * @param IDNum id number
     * @param x x coordinate of robot position
     * @param y y coordinate of robot position
     * @param vx x component of robot vector
     * @param vy y component of  robot vector
     */
    Robot(int IDNum, int x, int y, int vx, int vy)
    {
        ID = IDNum;
        X = x;
        Y = y;
        VX = vx;
        VY = vy;
        FX = 0.0;
        FY = 0.0;
    }
    /**
     * @brief move robot position
     * @param x x coord of robot position
     * @param y y coord of robot position
     */
    void movePosition(int x, int y)
    {
        X = x;
        Y = y;
    }

	std::vector<Point2D> getLocalObstacleLocations();

    /**
     * @brief set velocity 
     * @param vx velocity x component
     * @param vy velocity y component
     */
    void setVelocity(int vx, int vy) {
        VX = vx;
        VY = vy;
    }
    /**
     * @brief get velocity 
     * @return velocity of robot
     */
    Vector2D getVelocity() {
        return Vector2D(VX, VY);
    }
    /**
     * @brief move robot
     * @param action robot action
     */
    void setRobotAction(RobotAction action) {
        if (action == MOVE_UP) {
            FX = -1; FY = 0;
        } else if (action == MOVE_DOWN) {
            FX = 1; FY = 0;
        } else if (action == MOVE_LEFT) {
            FX = 0; FY = -1;
        } else if (action == MOVE_RIGHT) {
            FX = 0; FY = 1;
        } else if (action == MOVE_UP_LEFT) {
            FX = -1; FY = -1;
        } else if (action == MOVE_UP_RIGHT) {
            FX = -1; FY = 1;
        } else if (action == MOVE_DOWN_LEFT) {
            FX = 1; FY = -1;
        } else if (action == MOVE_DOWN_RIGHT) {
            FX = 1; FY = 1;
        } else if (action == STOP) {
            FX = 0; FY = 0;
        }
    }
    /**
     * @brief set force 
     * @param fx force x component 
     * @param fy force y component
     */
    void setForce(int fx, int fy) {
        if (fx != 0 && fx != 1 && fx != -1) {
            return;
        }
        if (fy != 0 && fy != 1 && fy != -1) {
            return;
        }
        FX = fx;
        FY = fy;
    }
    /**
     * @brief get force on robot
     * @return force on robot
     */
    Vector2D getForce() {
        return Vector2D(FX, FY);
    }
    /**
     * @brief set nominal velocity
     */
    void setNominalVelocity() {
        if (VX != 0)
            VX = VX > 0 ? 1 : -1;
        if (VY != 0)
            VY = VY > 0 ? 1 : -1;
    }
    /**
     * @brief set robot id 
     * @param IDNum id number of robot
     */
    void setID(int IDNum) {
        ID = IDNum;
    }
    /**
     * @brief get robot id
     * @return id of robot
     */
    int getID() {
        return this->ID;
    }
    /**
     * @brief get robot position
     * @return robot position
     */
    Point2D getPosition() {
        return Point2D(X,Y);
    }
};


#endif /* Robot_h */
