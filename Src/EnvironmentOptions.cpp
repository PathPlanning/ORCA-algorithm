#include "EnvironmentOptions.h"

EnvironmentOptions::EnvironmentOptions(const EnvironmentOptions &obj)
{
    this->metrictype = obj.metrictype;
    this->breakingties = obj.breakingties;
    this->allowsqueeze = obj.allowsqueeze;
    this->cutcorners = obj.cutcorners;
    this->timestep = obj.timestep;
    this->delta = obj.delta;
    this->hweight = obj.hweight;
    this->trigger = obj.trigger;
    this->MAPFNum = obj.MAPFNum;
}


EnvironmentOptions::EnvironmentOptions(int mt, bool bt, bool as, bool cc, float hw, float ts, float del, MAPFTriggers tr, int mn)
        :metrictype(mt), breakingties(bt), allowsqueeze(as), cutcorners(cc), hweight(hw), timestep(ts), delta(del), trigger(tr), MAPFNum(mn) {}