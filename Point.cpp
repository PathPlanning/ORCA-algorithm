#include "Point.h"
#include <vector>

Point::Point()
{
    x = 0.0;
    y = 0.0;
}


Point::Point(float x, float y)
{
    this->x = x;
    this->y = y;
}


float Point::GetX() const
{
    return x;
}


float Point::GetY() const
{
    return y;
}




Point::Point(const Point &obj)
{
    this->x = obj.x;
    this->y = obj.y;
}


std::pair<float, float> Point::GetPair()
{
    return {x, y};
}



