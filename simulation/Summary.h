#include <ostream>
#include <string>
#include <map>

#ifndef ORCA_SUMMARY_H
#define ORCA_SUMMARY_H



class Summary
{
    public:
        Summary() = default;
        ~Summary() = default;

//        Summary(float srate, float runtime, float flowtime, float makespan, int collisions, int collisionsObst)
//            : successRate(srate), runTime(runtime), flowTime(flowtime), makeSpan(makespan), collisions(collisions), collisionsObst(collisionsObst) {additionalFields = std::map<std::string, std::string>();}
//        Summary(const Summary &obj)
//            : successRate(obj.successRate), runTime(obj.runTime), flowTime(obj.flowTime), makeSpan(obj.makeSpan), collisions(obj.collisions), collisionsObst(obj.collisionsObst), additionalFields(obj.additionalFields){}



        std::string& operator[](std::string idx)
        {
            return fields[idx];
        }

    private:
        std::map<std::string, std::string> fields;


};


#endif //ORCA_SUMMARY_H
