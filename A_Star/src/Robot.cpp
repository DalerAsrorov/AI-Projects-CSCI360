#include "Robot.h"
#include "Simulator.h"
#include <vector>

std::vector<Point2D> Robot::getLocalObstacleLocations()
{
	return sim->getLocalObstacleLocations(getPosition());
}
