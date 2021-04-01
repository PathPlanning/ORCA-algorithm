#ifndef ORCA_CONST_H
#define ORCA_CONST_H


#define Velocity Point
#define Vector Point

#define CN_EPS 0.00001f
#define CN_INFINITY    1000000000
#define CN_PI_CONSTANT 3.14159265359
#define CN_SQRT_TWO    1.41421356237

//Grid Cell
#define CN_GC_NOOBS 0
#define CN_GC_OBS   1

//Search Parameters
#define CN_SP_BT_GMIN 0
#define CN_SP_BT_GMAX 1

#define CN_SP_MT_DIAG   0
#define CN_SP_MT_MANH   1
#define CN_SP_MT_EUCL   2
#define CN_SP_MT_CHEB   3

#define CN_SP_ST_THETA  0
#define CN_SP_ST_DIR    1

#define CNS_SP_ST_THETA     "thetastar"
#define CNS_SP_ST_DIR       "direct"

#define CNS_AT_ST_ORCA          "orca"
#define CNS_AT_ST_ORCADD        "orca-dd"
#define CNS_AT_ST_ORCAPAR       "orca-par"
#define CNS_AT_ST_ORCAECBS      "orca-ecbs"
#define CNS_AT_ST_ORCAPARECBS   "orca-par-ecbs"
#define CNS_AT_ST_ORCARETURN    "orca-return"

#define CNS_AT_ST_COMMONPOINT   "common-point"
#define CNS_AT_ST_SPEEDBUFFER   "speed-buffer"

// Default values
#define CN_DEFAULT_SIZE                 1
#define CN_DEFAULT_MAX_SPEED            1
#define CN_DEFAULT_AGENTS_MAX_NUM       10
#define CN_DEFAULT_TIME_BOUNDARY        5.4
#define CN_DEFAULT_OBS_TIME_BOUNDARY    10
#define CN_DEFAULT_RADIUS_OF_SIGHT      10
#define CN_DEFAULT_METRIC_TYPE          CN_SP_MT_EUCL
#define CN_DEFAULT_BREAKINGTIES         0
#define CN_DEFAULT_ALLOWSQUEEZE         0
#define CN_DEFAULT_CUTCORNERS           0
#define CN_DEFAULT_HWEIGHT              1
#define CN_DEFAULT_TIME_STEP            0.25
#define CN_DEFAULT_DELTA                0.1
#define CN_DEFAULT_REPS                 0.15
#define CN_DEFAULT_MAPFNUM              3
#define CN_DEFAULT_MAPF_MAXTIME         1000
#define CN_DEFAULT_MAPF_TRIGGER         COMMON_POINT
#define CN_DEFAULT_ST                   CN_SP_ST_THETA

#define CNS_DEFAULT_ST                  CNS_SP_ST_THETA
#define CNS_DEFAULT_AGENT_TYPE          CNS_AT_ST_ORCA
#define CNS_DEFAULT_MAPF_TRIGGER        CNS_AT_ST_COMMONPOINT

#define CNS_MAPF_COMMON_TIME        "mapf_time"
#define CNS_MAPF_INIT_COUNT         "init_count"
#define CNS_MAPF_UNITE_COUNT        "unite_count"
#define CNS_MAPF_UPDATE_COUNT       "update_count"
#define CNS_MAPF_ECBS_COUNT         "ecbs_count"
#define CNS_MAPF_PAR_COUNT          "par_count"
#define CNS_MAPF_SUCCESS_COUNT      "success_count"
#define CNS_MAPF_UNSUCCESS_COUNT    "unsuccess_count"
#define CNS_MAPF_FLOWTIME           "mapf_flowtime"
#endif //ORCA_CONST_H
