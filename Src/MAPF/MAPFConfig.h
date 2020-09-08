

#ifndef ORCASTAR_MAPFCONFIG_H
#define ORCASTAR_MAPFCONFIG_H

class MAPFConfig
{
    public:
        MAPFConfig()
        {
        }
        MAPFConfig(const MAPFConfig& orig) = default;
        ~MAPFConfig()
        {

        }

        int             maxTime = 1000;
        bool            withCAT = false;
        bool            withPerfectHeuristic = false;
        bool            parallelizePaths1 = false;
        bool            parallelizePaths2 = false;
        bool            withCardinalConflicts = false;
        bool            withBypassing = false;
        bool            withMatchingHeuristic = false;
        bool            storeConflicts = false;
        bool            withDisjointSplitting;
        bool            withFocalSearch = false;
        double          focalW = 1.0;

};

#endif //ORCASTAR_MAPFCONFIG_H


