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
        XmlLogger(int num, int r, double maxspeed, int neighborsNum, double timeBoundary, double sightradius, vector<pair<double, double>> start, vector<pair<double, double>> goal);
        void WriteAlgorithmParam(double timestep, double delta);
        void Save(vector<vector<pair<double, double>>> resultSteps, vector<pair<bool, int>> results ,double time);

    private:
        int num;
        int radius;
        XMLDocument *doc;
        XMLElement *root;
        XMLElement *steps;
        std::vector<XMLElement*> agentsteps;

};


#endif //ORCA_XMLLOGGER_H
