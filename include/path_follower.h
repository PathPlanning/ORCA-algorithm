#ifndef ORCASTAR_PATH_FOLLOWER_H
#define ORCASTAR_PATH_FOLLOWER_H

class PathFollower
{
    public:
        virtual ~PathFollower() {};
        virtual bool GetNextTarget(const Point &curr, Point &next);
        // TODO
};

class RebuildPathFollower : public PathFollower
{
    // TODO
};






#endif //ORCASTAR_PATH_FOLLOWER_H
