#ifndef Vector2D_h
#define Vector2D_h
#include <iostream>

#include <math.h>

class Point2D
{
public:
    int x, y;

    Point2D(int r = -1, int s = -1)
    {
        x = r;
        y = s;
	}
	
	bool operator==(const Point2D & rhs) const {
		return this->x == rhs.x && this->y == rhs.y;
	}
	
	friend std::ostream& operator<< (std::ostream& stream, const Point2D & p) {
		return stream<<"("<<p.x<<", "<<p.y<<")";
	}
};

#endif 
