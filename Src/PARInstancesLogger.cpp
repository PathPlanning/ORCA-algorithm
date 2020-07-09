#include "PARInstancesLogger.h"


PARInstancesLogger::PARInstancesLogger(std::string pathTempl)
{
    fileID = 0;
    pathTemplate = pathTempl;
}


bool PARInstancesLogger::SaveInstance(PARActorSet &agents, SubMap &map)
{
//    std::cout<<agents.getActorCount()<<"\n";
//    std::cout<< map.GetHeight()<<"x"<<map.GetWidth()<<"\n";

    XMLDocument *docMainXML, *docAgentsXML;
    std::string tmp1 = pathTemplate;
    std::string tmp2 = pathTemplate;


    size_t found = tmp1.find_last_of("/");
    std::string piece = "/MAP_" + std::to_string(fileID) + "_";
    tmp1.erase(found, 1);
    tmp1.insert(found, piece);


    found = tmp2.find_last_of("/");
    piece = "/AGENT_" + std::to_string(fileID) + "_";
    tmp2.erase(found, 1);
    tmp2.insert(found, piece);
//    std::cout<<tmp<<"\n";

    std::string mainFile = tmp1 + ".xml";
    std::string agentFile = tmp2  + "-" + std::to_string(1) + ".xml";

    /* MAIN ROOT */
    docMainXML = new XMLDocument();
    auto rootXML = docMainXML->NewElement(CNS_TAG_ROOT);
    docMainXML->InsertFirstChild(rootXML);

    /* MAP */
    auto mapXML = docMainXML->NewElement(CNS_TAG_MAP);
    auto widthXML = docMainXML->NewElement(CNS_TAG_WIDTH);
    auto heightXML = docMainXML->NewElement(CNS_TAG_HEIGHT);
    widthXML->SetText(map.GetWidth());
    heightXML->SetText(map.GetHeight());

    mapXML->InsertFirstChild(widthXML);
    mapXML->InsertFirstChild(heightXML);

    auto gridXML = docMainXML->NewElement(CNS_TAG_GRID);

    for(size_t i = 0; i < map.GetHeight(); i++)
    {
        auto rowXML = docMainXML->NewElement(CNS_TAG_ROW);

        std::string rowStr = "";
        for(size_t j = 0; j < map.GetWidth(); j++)
        {
            if(j != 0)
            {
                rowStr += " ";
            }

            if(map.CellIsObstacle(i, j))
            {
                rowStr += "1";
            }
            else
            {
                rowStr += "0";
            }
        }

        rowXML->SetText(rowStr.c_str());
        gridXML->InsertEndChild(rowXML);
    }
    mapXML->InsertEndChild(gridXML);
    rootXML->InsertFirstChild(mapXML);

    /* OPTIONS */
    auto optionsXML = docMainXML->NewElement(CNS_TAG_OPT);
//    auto algorithmXML = docMainXML->NewElement(CNS_TAG_ALG);
//    auto agentsXML = docMainXML->NewElement("agents_file");

    std::string tagNames[16] = {
            "algorithm",
            "low_level",
            "agents_file",
            "tasks_count",
            "maxtime",
            "with_cat",
            "with_perfect_h",
            "with_card_conf",
            "with_bypassing",
            "with_matching_h",
            "with_disjoint_splitting",
            "focal_w",
            "pp_order",
            "parallelize_paths_1",
            "parallelize_paths_2",
            "single_execution"
    };

    std::string tagText[16] = {
            "push_and_rotate",
            "astar",
            "",
            "1",
            "1000",
            "true",
            "false",
            "true",
            "true",
            "true",
            "true",
            "1.5",
            "2",
            "true",
            "true",
            "false"
    };

    tagText[2] = tmp2;

    for(size_t tag = 0; tag < 16; tag++)
    {
        auto optTagXML = docMainXML->NewElement(tagNames[tag].c_str());
        optTagXML->SetText(tagText[tag].c_str());
        optionsXML->InsertEndChild(optTagXML);
    }



    auto rangeXML = docMainXML->NewElement("agents_range");
    rangeXML->SetAttribute("min", agents.getActorCount());
    rangeXML->SetAttribute("max", agents.getActorCount());

    optionsXML->InsertEndChild(rangeXML);

    auto logpathXML = docMainXML->NewElement("logpath");
    auto logfilenameXML = docMainXML->NewElement("logfilename");
    optionsXML->InsertEndChild(logpathXML);
    optionsXML->InsertEndChild(logfilenameXML);


    rootXML->InsertEndChild(optionsXML);
    /* SAVE MAIN*/
    bool resMain = (docMainXML->SaveFile(mainFile.c_str()) == XMLError::XML_SUCCESS);


    docAgentsXML = new XMLDocument();
    rootXML = docAgentsXML->NewElement(CNS_TAG_ROOT);
    docAgentsXML->InsertFirstChild(rootXML);
    for(size_t agent = 0; agent < agents.getActorCount(); agent++ )
    {
        auto agentXML = docAgentsXML->NewElement("agent");
        agentXML->SetAttribute("id", (int)agent);
        agentXML->SetAttribute("start_i", agents.getActor(agent).getStart_i());
        agentXML->SetAttribute("start_j", agents.getActor(agent).getStart_j());
        agentXML->SetAttribute("goal_i", agents.getActor(agent).getGoal_i());
        agentXML->SetAttribute("goal_j", agents.getActor(agent).getGoal_j());

        rootXML->InsertEndChild(agentXML);
    }


    bool resAgents = (docAgentsXML->SaveFile(agentFile.c_str()) == XMLError::XML_SUCCESS);

    fileID++;
    return resMain && resAgents;

}