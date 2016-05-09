#ifndef Vector2D_h
#define Vector2D_h

#include <math.h>

#ifndef M_PI
#define M_PI 3.14
#endif


/**
 * @class Vector2D
 * @brief a 2d vector
 */
class Vector2D
{
public:
    
    float   x; /**< x component */
    float   y; /**< y component */
    /**
     * @brief constructor
     */
    Vector2D() {}
    /**
     * @brief constructor
     * @param r x component
     * @param s y component
     */
    Vector2D(float r, float s)
    {
        x = r;
        y = s;
    }    
    /**
     * @brief operator /=
     * @param t float by which to divide vector
     * @return new vector divided by float
     */
    Vector2D& operator /=(float t)
    {
        float f = 1.0F / t;
        x *= f;
        y *= f;
        return (*this);
    }
    /**
     * @brief normalize vector
     * @return normalized vector
     */
    Vector2D& normalize(void)
    {
        return (*this /= sqrtf(x * x + y * y));
    }
    /**
     * @brief get theta
     * @return theta in degrees
     */
    double getTheta() {
        return (180.0 / M_PI) * atan2(y,x);
    }
};
/**
 * @class Point2D
 * @brief 2d point
 */
class Point2D : public Vector2D
{
public:
    /**
     * @brief constructor
     */
    Point2D() {}
    /**
     * @brief constructor
     * @param r x coordinate
     * @param s y coordinate
     */
    Point2D(float r, float s) : Vector2D(r, s) {}
    
};


#endif /* Vector2D_h */
