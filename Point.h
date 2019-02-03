//
// Created by Stepan on 03/02/2019.
//

#ifndef ORCA_POINT_H
#define ORCA_POINT_H


class Point
{
    public:
        double GetX();
        double GetY();
        void SetX(double x);
        void SetY(double y);
    private:
        double x;
        double y;
};


#endif //ORCA_POINT_H
