#ifndef ORCA_XMLLOGGER_H
#define ORCA_XMLLOGGER_H

#include "tinyxml2.h"
#include <string>
#include <vector>

using namespace std;
using namespace tinyxml2;

class XmlLogger
{
    public:
        XmlLogger(int num, float r, float maxspeed, int neighborsNum, float timeBoundary, float sightradius, vector<pair<float, float>> start, vector<pair<float, float>> goal);
        void WriteAlgorithmParam(float timestep, float delta);
        void Save(vector<vector<pair<float, float>>> resultSteps, vector<pair<bool, int>> results ,float time);

    private:
        int num;
        float radius;
        XMLDocument *doc;
        XMLElement *root;
        std::vector<XMLElement*> agentsteps;

};


#endif //ORCA_XMLLOGGER_H
