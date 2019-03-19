#ifndef ORCA_POINT_H
#define ORCA_POINT_H

#include <cmath>

class Point
{
    public:
        Point();
        Point(double x, double y);
        Point(const Point &obj);
        double GetX() const;
        double GetY() const;
        std::pair<double, double> GetPair();

        double ScalarProduct(const Point &another) const;
        double EuclideanNorm() const;
        double SquaredEuclideanNorm() const;
        double Det(Point another) const;

        Point operator - (const Point &another) const;
        Point operator + (const Point &another) const;
        bool operator == (const Point &another) const;
        Point operator * (double k) const;
        Point operator / (double k) const;


    private:
        double x;
        double y;
        double sqEqNorm;
};


#endif //ORCA_POINT_H
