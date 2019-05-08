#ifndef ORCA_POINT_H
#define ORCA_POINT_H

#include <cmath>

class Point
{
    public:
        Point();
        Point(float x, float y);
        Point(const Point &obj);
        float GetX() const;
        float GetY() const;
        std::pair<float, float> GetPair();

         float ScalarProduct(const Point &another) const;
         float EuclideanNorm() const;
         float SquaredEuclideanNorm() const;
         float Det(Point another) const;

         Point operator - (const Point &another) const;
         Point operator + (const Point &another) const;
         bool operator == (const Point &another) const;
         Point operator * (float k) const;
         Point operator / (float k) const;
         Point operator - () const;


    private:
        float x;
        float y;
};

inline float Point::ScalarProduct(const Point &another) const
{
    return this->x * another.x + this->y * another.y;
}


inline Point Point::operator - (const Point &another) const
{
    return {this->x - another.x, this->y - another.y};
}


inline Point Point::operator + (const Point &another) const
{
    return {this->x + another.x, this->y + another.y};
}


inline Point Point::operator * (float k) const
{
    return {this->x * k, this->y * k};
}


inline Point Point::operator /(float k) const
{
    const float invK = 1.0f / k;
    return {this->x * invK, this->y * invK};
}


inline float Point::SquaredEuclideanNorm() const
{
    return this->ScalarProduct(*this);
}


inline float Point::EuclideanNorm() const
{
    return std::sqrt(this->ScalarProduct(*this));
}

inline float Point::Det(Point another) const
{
    return (this->x * another.y - this->y * another.x);
}

inline bool Point::operator ==(const Point &another) const
{
    return (this->x == another.x) && (this->y == another.y);
}

inline Point Point::operator-() const
{
    return Point(-this->x, -this->y);
}

#endif //ORCA_POINT_H
