#include "../include/Geom.h"

int Node::convolution(int width, int height, bool withTime) const
{
    int res = withTime ? width * height * g : 0;
    return res + i * width + j;
}

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


float Point::X() const
{
    return x;
}


float Point::Y() const
{
    return y;
}


Point::Point(const Point &obj)
{
    this->x = obj.x;
    this->y = obj.y;
}


//std::pair<float, float> Point::GetPair()
//{
//    return {x, y};
//}
//
//std::string Point::ToString() const
//{
//    return "x: " + std::to_string(x) + "\ty: "
//           + std::to_string(y) + "\n";
//
//}

bool Vertex::IsConvex() const
{
    return convex;
}


void Vertex::SetConvex(bool cvx)
{
    convex = cvx;
}



bool Node::operator < (const Node &other) const
{
    return std::tuple<int, int, int, int>(F, -g, i, j) <
           std::tuple<int, int, int, int>(other.F, -other.g, other.i, other.j);
}


