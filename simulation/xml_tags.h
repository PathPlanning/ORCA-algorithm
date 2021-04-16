

#ifndef ORCASTAR_XML_TAGS_H
#define ORCASTAR_XML_TAGS_H

//XML tags
#define CNS_TAG_ROOT                    "root"

#define CNS_TAG_AGENTS                  "agents"
#define CNS_TAG_ATTR_NUM            "number"
#define CNS_TAG_ATTR_TYPE           "type"

#define CNS_TAG_DEF_PARAMS              "default_parameters"
#define CNS_TAG_ATTR_SIZE               "size"
#define CNS_TAG_ATTR_MAXSPEED           "movespeed"
#define CNS_TAG_ATTR_AGENTSMAXNUM       "agentsmaxnum"
#define CNS_TAG_ATTR_TIMEBOUNDARY       "timeboundary"
#define CNS_TAG_ATTR_SIGHTRADIUS        "sightradius"
#define CNS_TAG_ATTR_TIMEBOUNDARYOBST   "timeboundaryobst"
#define CNS_TAG_ATTR_REPS               "reps"

#define CNS_TAG_AGENT                   "agent"
#define CNS_TAG_ATTR_ID             "id"
#define CNS_TAG_ATTR_STX             "start.xr"
#define CNS_TAG_ATTR_STY             "start.yr"
#define CNS_TAG_ATTR_GX             "goal.xr"
#define CNS_TAG_ATTR_GY             "goal.yr"

#define CNS_TAG_ALG         "algorithm"
#define CNS_TAG_ST              "searchtype"
#define CNS_TAG_MT              "metrictype"
#define CNS_TAG_BT              "breakingties"
#define CNS_TAG_AS              "allowsqueeze"
#define CNS_TAG_CC              "cutcorners"
#define CNS_TAG_HW              "hweight"
#define CNS_TAG_TS              "timestep"
#define CNS_TAG_DEL             "delta"
#define CNS_TAG_TR              "trigger"
#define CNS_TAG_MN              "mapfnum"

#define CNS_TAG_MAP         "map"
#define CNS_TAG_CELLSIZE    "cellsize"
#define CNS_TAG_WIDTH       "width"
#define CNS_TAG_HEIGHT      "height"
#define CNS_TAG_GRID        "grid"
#define CNS_TAG_ROW         "row"

#define CNS_TAG_OBSTS       "obstacles"
#define CNS_TAG_OBST        "obstacle"
#define CNS_TAG_VERTEX      "vertex"
#define CNS_TAG_ATTR_X      "xr"
#define CNS_TAG_ATTR_Y      "yr"

#define CNS_TAG_LOG             "log"
#define CNS_TAG_SUM             "summary"
#define CNS_TAG_ATTR_SR         "successrate"
#define CNS_TAG_ATTR_RUNTIME    "runtime"
#define CNS_TAG_ATTR_FLOWTIME   "flowtime"
#define CNS_TAG_ATTR_MAKESPAN   "makespan"
#define CNS_TAG_ATTR_COL_AGNT   "collisions"
#define CNS_TAG_ATTR_COL_OBST   "collisionsobst"
#define CNS_TAG_PATH            "path"
#define CNS_TAG_ATTR_PATHFOUND  "pathfound"
#define CNS_TAG_ATTR_STEPS      "steps"
#define CNS_TAG_STEP            "step"



#define CNS_SUM_SUCCESS_RATE            "success_rate"
#define CNS_SUM_RUN_TIME                "run_time"
#define CNS_SUM_FLOW_TIME               "flow_time"
#define CNS_SUM_MAKESPAN                "makespan"
#define CNS_SUM_COLLISIONS              "collisions"
#define CNS_SUM_COLLISIONS_OBS          "collisions_obs"
#define CNS_SUM_MAPF_MEAN_TIME          "mapf_runtime"
#define CNS_SUM_MAPF_INIT_COUNT         "init_count"
#define CNS_SUM_MAPF_UNITE_COUNT        "unite_count"
#define CNS_SUM_MAPF_UPDATE_COUNT       "update_count"
#define CNS_SUM_MAPF_ECBS_COUNT         "ecbs_count"
#define CNS_SUM_MAPF_PAR_COUNT          "par_count"
#define CNS_SUM_MAPF_SUCCESS_COUNT      "success_count"
#define CNS_SUM_MAPF_UNSUCCESS_COUNT    "unsuccess_count"
#define CNS_SUM_MAPF_FLOWTIME           "mapf_flowtime"





#define CNS_TAG_OPT         "options"
#define CNS_TAG_LOGLVL      "loglevel"
#define CNS_TAG_LOGPATH     "logpath"
#define CNS_TAG_LOGFN       "logfilename"


#define CNS_TAG_MAPFN       "mapfilename"


#define CNS_TAG_LPLEVEL     "lplevel"
#define CNS_TAG_HPLEVEL     "hplevel"
#define CNS_TAG_SECTION     "section"
#define CNS_TAG_LOWLEVEL    "lowlevel"

#define CNS_TAG_OPEN        "open"
#define CNS_TAG_POINT       "node"
#define CNS_TAG_CLOSE       "close"

//XML tags' attributes
#define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
#define CNS_TAG_ATTR_NODESCREATED   "nodescreated"
#define CNS_TAG_ATTR_LENGTH         "length"
#define CNS_TAG_ATTR_LENGTH_SCALED  "length_scaled"
#define CNS_TAG_ATTR_TIME           "time"

#define CNS_TAG_ATTR_F              "F"
#define CNS_TAG_ATTR_G              "g"
#define CNS_TAG_ATTR_PARX           "parent_x"
#define CNS_TAG_ATTR_PARY           "parent_y"
#define CNS_TAG_ATTR_FINX           "finish.x"
#define CNS_TAG_ATTR_FINY           "finish.y"


//Log Configuration
#define CN_LP_LEVEL 0

#define CN_LP_LEVEL_NOPE_VALUE      "0"
#define CN_LP_LEVEL_NOPE_WORD       "nope"
#define CN_LP_LEVEL_TINY_VALUE      "0.5"
#define CN_LP_LEVEL_TINY_WORD       "tiny"
#define CN_LP_LEVEL_SHORT_VALUE     "1"
#define CN_LP_LEVEL_SHORT_WORD      "short"
#define CN_LP_LEVEL_MEDIUM_VALUE    "1.5"
#define CN_LP_LEVEL_MEDIUM_WORD     "medium"
#define CN_LP_LEVEL_FULL_VALUE      "2"
#define CN_LP_LEVEL_FULL_WORD       "full"

#define CN_LP_PATH 1
#define CN_LP_NAME 2


//Other
#define CNS_OTHER_PATHSELECTION     "*"
#define CNS_OTHER_MATRIXSEPARATOR   ' '




#endif //ORCASTAR_XML_TAGS_H
