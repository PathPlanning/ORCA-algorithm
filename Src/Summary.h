#include <ostream>
#include <string>

#ifndef ORCA_SUMMARY_H
#define ORCA_SUMMARY_H



class Summary
{
    public:
        Summary() = default;
        ~Summary() = default;

        Summary(float srate, float runtime, float flowtime, float makespan, int collisions, int collisionsObst)
            : successRate(srate), runTime(runtime), flowTime(flowtime), makeSpan(makespan), collisions(collisions), collisionsObst(collisionsObst){}
        Summary(const Summary &obj)
            : successRate(obj.successRate), runTime(obj.runTime), flowTime(obj.flowTime), makeSpan(obj.makeSpan), collisions(obj.collisions), collisionsObst(obj.collisionsObst){}


        std::string ToString() const
        {
            return std::to_string(successRate) + "\t"
                 + std::to_string(runTime) + "\t"
                 + std::to_string(makeSpan) + "\t"
                 + std::to_string(flowTime) + "\t"
                 + std::to_string(collisions) + "\t"
                 + std::to_string(collisionsObst) + "\n";
        }

        float successRate;
        float runTime;
        float flowTime;
        float makeSpan;
        int   collisions;
        int   collisionsObst;

};


#endif //ORCA_SUMMARY_H
