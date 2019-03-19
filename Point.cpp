#include "Point.h"
#include <vector>

Point::Point()
{
    x = 0;
    y = 0;
    sqEqNorm = 0;

}


Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
    sqEqNorm = (this->x * this->x + this->y * this->y);
}


double Point::GetX() const
{
    return x;
}


double Point::GetY() const
{
    return y;
}


double Point::ScalarProduct(const Point &another) const
{
    return this->x * another.x + this->y * another.y;
}


Point Point::operator - (const Point &another) const
{
    return {this->x - another.x, this->y - another.y};
}


Point Point::operator + (const Point &another) const
{
    return {this->x + another.x, this->y + another.y};
}


Point Point::operator * (double k) const
{
    return {this->x * k, this->y * k};
}


Point Point::operator /(double k) const
{
    return {this->x / k, this->y / k};
}


double Point::SquaredEuclideanNorm() const
{
    return sqEqNorm;
}


double Point::EuclideanNorm() const
{
    return std::sqrt(sqEqNorm);
}


Point::Point(const Point &obj)
{
    this->x = obj.x;
    this->y = obj.y;
    this->sqEqNorm = obj.sqEqNorm;
}


std::pair<double, double> Point::GetPair()
{
    return {x, y};
}


double Point::Det(Point another) const
{
    return (this->x * another.y - this->y * another.x);
}

bool Point::operator ==(const Point &another) const
{
    return (this->x == another.x) && (this->y == another.y);
}
