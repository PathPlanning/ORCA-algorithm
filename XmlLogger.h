#ifndef ORCA_XMLLOGGER_H
#define ORCA_XMLLOGGER_H

#include "tinyxml2.h"
#include <string>
#include <vector>

using namespace tinyxml2;
class XmlLogger
{
    public:
        XmlLogger(int num, int r,  std::vector<std::pair<double, double>> start, std::vector<std::pair<double, double>> goal);
        void WriteStep(int step, int agentnum, double x, double y);
        void Save();

    private:
        int num;
        int radius;
        XMLDocument *doc;
        XMLElement *root;
        XMLElement *steps;
        std::vector<XMLElement*> agentsteps;

};


#endif //ORCA_XMLLOGGER_H
