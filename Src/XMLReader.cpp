#include "XMLReader.h"


XMLReader::XMLReader(const std::string &fileName)
{
    this->fileName = fileName;
    this->doc = new XMLDocument();
    this->allAgents = new std::vector<Agent *>();

    this->map = nullptr;
    this->options = nullptr;
    this->grid = nullptr;
    this->obstacles = nullptr;
}


XMLReader::XMLReader()
{
    this->fileName = "";
    this->doc = nullptr;
    this->allAgents = nullptr;

    this->map = nullptr;
    this->options = nullptr;
    this->grid = nullptr;
    this->obstacles = nullptr;
}



XMLReader::XMLReader(const XMLReader &obj)
{

    fileName = obj.fileName;

    if(obj.doc != nullptr)
    {
        doc = new XMLDocument();
        obj.doc->DeepCopy(doc);
    }
    else
    {
        doc = nullptr;
    }

    allAgents = (obj.allAgents == nullptr) ? nullptr : new std::vector<Agent *>(*(obj.allAgents));
    map = (obj.map == nullptr) ? nullptr : new Map(*obj.map);
    options = (obj.options == nullptr) ? nullptr : new EnvironmentOptions(*obj.options);
    grid = (obj.grid == nullptr) ? nullptr : new  std::vector<std::vector<int>>(*obj.grid);
    obstacles = (obj.obstacles == nullptr) ? nullptr : new std::vector<std::vector<Point>>(*obj.obstacles);
}


XMLReader::~XMLReader()
{

    if(allAgents != nullptr)
    {
        delete allAgents;
        allAgents = nullptr;
    }

    if(options != nullptr)
    {
        delete options;
        options = nullptr;
    }

    if(map != nullptr)
    {
        delete map;
        map = nullptr;
    }
    if(grid != nullptr)
    {
        delete grid;
        grid = nullptr;
    }
    if(obstacles != nullptr)
    {
        delete obstacles;
        obstacles = nullptr;
    }

    if(doc != nullptr)
    {
        delete doc;
        doc = nullptr;
    }

}


bool XMLReader::GetMap(Map **map)
{
    if(this->map != nullptr)
    {
        *map = this->map;
        this->map = nullptr;
        return true;
    }

    return false;
}


bool XMLReader::GetEnvironmentOptions(EnvironmentOptions **envOpt)
{
    if(this->options != nullptr)
    {
        *envOpt = this->options;
        this->options = nullptr;
        return true;
    }
    return false;
}


bool XMLReader::GetAgents(std::vector<Agent *> &agents, const int &numThreshold)
{

    if(allAgents == nullptr || allAgents->size() >= numThreshold)
    {
        agents.clear();
        for(int i = 0; i < numThreshold; i++)
        {
            agents.push_back((*allAgents)[i]);
        }
        for(int i = numThreshold; i < allAgents->size(); i++)
        {
            delete (*allAgents)[i];
        }
        allAgents->clear();
        delete allAgents;
        allAgents = nullptr;

        return true;
    }
    return false;
}

bool XMLReader::ReadData()
{

    if(doc == nullptr || allAgents == nullptr)
    {
        return false;
    }

    bool tagFlag;
    XMLElement *tmpElement;

    // Чтение XML-файла с заданием / Reading task from XML-file //

    if(doc->LoadFile(fileName.c_str()) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening input XML file\n";
        return false;
    }

    XMLElement *root = doc->FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout <<CNS_TAG_ROOT <<" element not found in XML file\n";
        return false;
    }

    // Чтение информации о карте и препятствиях / Reading information about map and obstacles //

    XMLElement *mapTag = 0, *element = 0, *mapnode, *obsts = 0, *obstElem = 0;

    std::string value;
    std::stringstream stream;

    bool hasGridMem = false, hasGrid = false, hasHeight = false, hasWidth = false, hasCellSize = false;
    int height = -1, width = -1, rowIter = 0, gridI = 0, gridJ = 0, obstNum = -1, count = 0;
    float cellSize;

    // Чтение карты / Map reading //

    mapTag = root->FirstChildElement(CNS_TAG_MAP);
    if (!mapTag)
    {
        std::cout << CNS_TAG_MAP <<" element not found in XML file\n";
        return false;
    }

    for (mapnode = mapTag->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement())
    {
        element = mapnode->ToElement();
        value = mapnode->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        stream.str("");
        stream.clear();

        if(value != CNS_TAG_GRID)
        {
            stream << element->GetText();
        }

        if (!hasGridMem && hasHeight && hasWidth)
        {
            grid = new std::vector<std::vector<int>>(height);
            for (int i = 0; i < height; ++i)
                (*grid)[i] = std::vector<int>(width);
            hasGridMem = true;
        }

        if (value == CNS_TAG_HEIGHT)
        {
            if (hasHeight)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_HEIGHT << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_HEIGHT << "' =" << height << "will be used." << std::endl;
            }
            else
            {
                if (!((stream >> height) && (height > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_HEIGHT << "' tag should be an integer >=0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_HEIGHT
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                {
                    hasHeight = true;
                }
            }
        }
        else if (value == CNS_TAG_WIDTH)
        {
            if (hasWidth)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_WIDTH << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_WIDTH << "' =" << width << "will be used." << std::endl;
            }
            else
            {
                if (!((stream >> width) && (width > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_WIDTH << "' tag should be an integer AND >0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_WIDTH
                              << "' tag will be encountered later..." << std::endl;

                }
                else
                {
                    hasWidth = true;
                }
            }
        }
        else if (value == CNS_TAG_CELLSIZE)
        {
            if (hasCellSize)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_CELLSIZE << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_CELLSIZE << "' =" << cellSize << "will be used."
                          << std::endl;
            }
            else {
                if (!((stream >> cellSize) && (cellSize > 0))) {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_CELLSIZE
                              << "' tag encountered (or could not convert to double)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_CELLSIZE
                              << "' tag should be double AND >0. By default it is defined to '1'" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_CELLSIZE
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasCellSize = true;
            }
        }
        else if (value == CNS_TAG_GRID)
        {
            hasGrid = true;
            if (!(hasHeight && hasWidth))
            {
                std::cout << "Error! No '" << CNS_TAG_WIDTH << "' tag or '" << CNS_TAG_HEIGHT << "' tag before '"
                          << CNS_TAG_GRID << "'tag encountered!" << std::endl;
                return false;
            }
            element = mapnode->FirstChildElement();
            while (gridI < height)
            {
                if (!element) {
                    std::cout << "Error! Not enough '" << CNS_TAG_ROW << "' tags inside '" << CNS_TAG_GRID << "' tag."
                              << std::endl;
                    std::cout << "Number of '" << CNS_TAG_ROW
                              << "' tags should be equal (or greater) than the value of '" << CNS_TAG_HEIGHT
                              << "' tag which is " << height << std::endl;
                    return false;
                }
                std::string str = element->GetText();
                std::vector<std::string> elems;
                std::stringstream ss(str);
                std::string item;
                while (std::getline(ss, item, ' '))
                {
                    elems.push_back(item);
                }
                rowIter = gridJ = 0;
                int val;
                if (elems.size() > 0)
                {
                    for(gridJ = 0; gridJ < width; ++gridJ)
                    {
                        if(gridJ == elems.size())
                        {
                            break;
                        }
                        stream.str("");
                        stream.clear();
                        stream << elems[gridJ];
                        stream >> val;
                        (*grid)[gridI][gridJ] = val;
                    }
                }
                if (gridJ != width)
                {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << gridI + 1 << " " << CNS_TAG_ROW
                              << std::endl;
                    return false;
                }
                ++gridI;

                element = element->NextSiblingElement();
            }
        }
    }

    if (!hasGrid)
    {
        std::cout << CNS_TAG_GRID <<" element not found in XML file\n";
        return false;
    }


    // Чтение препятствий / Obstacles reading //

    obsts = root->FirstChildElement(CNS_TAG_OBSTS);
    if (!obsts)
    {
        std::cout << CNS_TAG_OBSTS <<" element not found in XML file\n";
        return false;
    }

    if (obsts->QueryIntAttribute(CNS_TAG_ATTR_NUM, &obstNum) != XMLError::XML_SUCCESS)
    {
        std::cout << CNS_TAG_ATTR_NUM <<" element not found at " << CNS_TAG_OBSTS << " tag in XML file\n";
        return false;
    }

    obstacles = new std::vector<std::vector<Point>>(obstNum);
    float oX,oY;
    obstElem = obsts->FirstChildElement(CNS_TAG_OBST);
    for(count = 0; obstElem && count < obstNum; obstElem = obstElem->NextSiblingElement(CNS_TAG_OBST), count++)
    {
        for(tmpElement = obstElem->FirstChildElement(CNS_TAG_VERTEX); tmpElement; tmpElement = tmpElement->NextSiblingElement(CNS_TAG_VERTEX))
        {
            if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_X, &oX) != XMLError::XML_SUCCESS)
            {
                std::cout << CNS_TAG_ATTR_X <<" element not found at " << CNS_TAG_OBST <<" #" << count + 1 << " tag in XML file\n";
                return false;
            }
            if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_Y, &oY) != XMLError::XML_SUCCESS)
            {
                std::cout << CNS_TAG_ATTR_Y <<" element not found at " << CNS_TAG_OBST <<" #" << count + 1 << " tag in XML file\n";
                return false;
            }
            (*obstacles)[count].push_back(Point(oX, oY));
        }

    }

    if(count != obstNum)
    {
        std::cout << "The number of obstacles does not match the stated in "<< CNS_TAG_ATTR_NUM << "attribute of " << CNS_TAG_OBSTS << " tag\n";
        return false;
    }

    map = new Map(cellSize, *grid, *obstacles);

    // Чтение информации о параметрах окружения и алгоритма// Reading options of the environment and algorithm //

    options = new EnvironmentOptions(CN_DEFAULT_METRIC_TYPE, CN_DEFAULT_BREAKINGTIES, CN_DEFAULT_ALLOWSQUEEZE, CN_DEFAULT_CUTCORNERS,
                                     CN_DEFAULT_HWEIGHT, CN_DEFAULT_TIME_STEP, CN_DEFAULT_DELTA);

    XMLElement *alg = root->FirstChildElement(CNS_TAG_ALG);
    if(!alg)
    {
        std::cout <<CNS_TAG_ALG <<" element not found in XML file\n";
    }

    // Нет необходимости в считывании типа эвристики / Metric type not necessary //

//    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_MT)) && (tmpElement->QueryIntText(&(options->metrictype)) == XMLError::XML_SUCCESS);
//    if(!tagFlag)
//    {
//        std::cout <<CNS_TAG_MT <<" element not found in XML file. It was defined to "<< CN_DEFAULT_METRIC_TYPE<<"\n";
//    }

    int plannertype = CN_DEFAULT_ST;
    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_ST)) && tmpElement->GetText();
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_ST <<" element not found in XML file. It was defined to "<< CNS_DEFAULT_ST <<"\n";
    }
    else
    {
        auto tmpst = std::string(tmpElement->GetText());
        if (tmpst == CNS_SP_ST_THETA)
        {
            plannertype = CN_SP_ST_THETA;
        }
        else if(tmpst == CNS_SP_ST_DIR)
        {
            plannertype = CN_SP_ST_DIR;
        }
        else
        {
            std::cout <<CNS_TAG_ST <<" element are incorrect. It was defined to "<< CNS_DEFAULT_ST <<"\n";
        }
    }

    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_BT)) && (tmpElement->QueryBoolText(&(options->breakingties)) == XMLError::XML_SUCCESS);
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_BT <<" element not found in XML file. It was defined to "<< CN_DEFAULT_BREAKINGTIES<<"\n";
    }

    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_AS)) && (tmpElement->QueryBoolText(&(options->allowsqueeze)) == XMLError::XML_SUCCESS);
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_AS <<" element not found in XML file. It was defined to "<< CN_DEFAULT_ALLOWSQUEEZE<<"\n";
    }

    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_CC)) && (tmpElement->QueryBoolText(&(options->cutcorners)) == XMLError::XML_SUCCESS);
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_CC <<" element not found in XML file. It was defined to "<< CN_DEFAULT_CUTCORNERS<<"\n";
    }

    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_HW)) && (tmpElement->QueryFloatText(&(options->hweight)) == XMLError::XML_SUCCESS);
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_HW <<" element not found in XML file. It was defined to "<< CN_DEFAULT_HWEIGHT<<"\n";
    }

    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_TS)) && (tmpElement->QueryFloatText(&(options->timestep)) == XMLError::XML_SUCCESS);
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_TS <<" element not found in XML file. It was defined to "<< CN_DEFAULT_TIME_STEP<<"\n";
    }

    tagFlag = (tmpElement = alg->FirstChildElement(CNS_TAG_DEL)) && (tmpElement->QueryFloatText(&(options->delta)) == XMLError::XML_SUCCESS);
    if(!tagFlag)
    {
        std::cout <<CNS_TAG_DEL <<" element not found in XML file. It was defined to "<< CN_DEFAULT_DELTA<<"\n";
    }

    // Чтение информации о агентах// Reading agents information //

    AgentParam defaultParam = AgentParam();
    int  agentsNumber;

    XMLElement *agents = root->FirstChildElement(CNS_TAG_AGENTS);
    if(!agents)
    {
        std::cout <<CNS_TAG_AGENTS <<" element not found in XML file\n";
        return false;
    }

    if(agents->QueryIntAttribute(CNS_TAG_ATTR_NUM, &agentsNumber) != XMLError::XML_SUCCESS)
    {
        std::cout <<CNS_TAG_ATTR_NUM <<" element not found at " << CNS_TAG_AGENTS << " tag in XML file\n";
        return false;
    }

    tmpElement = agents->FirstChildElement(CNS_TAG_DEF_PARAMS);
    if (!tmpElement)
    {
        std::cout <<CNS_TAG_DEF_PARAMS <<" element not found in XML file. The following default values was defined\n";
        std::cout << "Default " <<CNS_TAG_ATTR_SIZE << " = " << CN_DEFAULT_SIZE <<"\n";
        std::cout << "Default " <<CNS_TAG_ATTR_MAXSPEED << " = " << CN_DEFAULT_MAX_SPEED <<"\n";
        std::cout << "Default " <<CNS_TAG_ATTR_AGENTSMAXNUM << " = " << CN_DEFAULT_AGENTS_MAX_NUM <<"\n";
        std::cout << "Default " <<CNS_TAG_ATTR_TIMEBOUNDARY << " = " << CN_DEFAULT_TIME_BOUNDARY <<"\n";
        std::cout << "Default " <<CNS_TAG_ATTR_SIGHTRADIUS << " = " << CN_DEFAULT_RADIUS_OF_SIGHT <<"\n";
        std::cout << "Default " <<CNS_TAG_ATTR_TIMEBOUNDARYOBST << " = " << CN_DEFAULT_OBS_TIME_BOUNDARY <<"\n";
    }
    else
    {
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_SIZE, &defaultParam.radius) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_SIZE <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_SIZE<<"\n";
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_MAXSPEED, &defaultParam.maxSpeed) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_MAXSPEED <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_MAX_SPEED<<"\n";
        }
        if(tmpElement->QueryIntAttribute(CNS_TAG_ATTR_AGENTSMAXNUM, &defaultParam.agentsMaxNum) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_AGENTSMAXNUM <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_AGENTS_MAX_NUM<<"\n";
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARY, &defaultParam.timeBoundary) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_TIMEBOUNDARY <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_TIME_BOUNDARY<<"\n";
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_SIGHTRADIUS, &defaultParam.sightRadius) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_SIGHTRADIUS <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_RADIUS_OF_SIGHT<<"\n";
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARYOBST, &defaultParam.timeBoundaryObst) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_TIMEBOUNDARYOBST <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_OBS_TIME_BOUNDARY<<"\n";
        }
    }
    tmpElement = agents->FirstChildElement(CNS_TAG_AGENT);

    int id;
    float stx = 0, sty = 0, gx = 0, gy = 0;

    for(count = 0; tmpElement; tmpElement = tmpElement->NextSiblingElement(CNS_TAG_AGENT), count++)
    {
        AgentParam param = AgentParam(defaultParam);

        bool correct = true;

        if (tmpElement->QueryIntAttribute(CNS_TAG_ATTR_ID, &id) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_ID <<" element not found in XML file at agent №"<<count<<"\n";
            return false;
        }

        for(auto agent : *allAgents)
        {
            if(id == agent->GetID())
            {
                std::cout <<"There is an agent with same ID ("<<id<<")\n";
                correct = false;
            }
        }

        if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_STX, &stx) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_STX <<" element not found in XML file at agent №"<<count<<"\n";
            return false;
        }
        if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_STY, &sty) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_STY <<" element not found in XML file at agent "<<id<<"\n";
            return false;
        }
        if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_GX, &gx) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_GX <<" element not found in XML file at agent "<<id<<"\n";
            return false;
        }
        if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_GY, &gy) != XMLError::XML_SUCCESS)
        {
            std::cout <<CNS_TAG_ATTR_GY <<" element not found in XML file at agent "<<id<<"\n";
            return false;
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_SIZE, &param.radius) != XMLError::XML_SUCCESS || param.radius <= 0)
        {
            std::cout <<CNS_TAG_ATTR_SIZE <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
            param.radius = defaultParam.radius;
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_MAXSPEED, &param.maxSpeed) != XMLError::XML_SUCCESS || param.maxSpeed <= 0)
        {
            std::cout <<CNS_TAG_ATTR_MAXSPEED <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
            param.maxSpeed = defaultParam.maxSpeed;
        }
        if(tmpElement->QueryIntAttribute(CNS_TAG_ATTR_AGENTSMAXNUM, &param.agentsMaxNum) != XMLError::XML_SUCCESS || param.agentsMaxNum <= 0)
        {
            std::cout <<CNS_TAG_ATTR_AGENTSMAXNUM <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
            param.agentsMaxNum = defaultParam.agentsMaxNum;
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARY, &param.timeBoundary) != XMLError::XML_SUCCESS || param.timeBoundary <= 0)
        {
            std::cout <<CNS_TAG_ATTR_TIMEBOUNDARY <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
            param.timeBoundary = defaultParam.timeBoundary;
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_SIGHTRADIUS, &param.sightRadius) != XMLError::XML_SUCCESS || param.sightRadius <= 0)
        {
            std::cout <<CNS_TAG_ATTR_SIGHTRADIUS <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
            param.sightRadius = defaultParam.sightRadius;
        }
        if(tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARYOBST, &param.timeBoundaryObst) != XMLError::XML_SUCCESS || param.timeBoundaryObst <= 0)
        {
            std::cout <<CNS_TAG_ATTR_TIMEBOUNDARYOBST <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
            param.timeBoundaryObst = defaultParam.timeBoundaryObst;
        }




        if(stx <= param.radius || sty <= param.radius ||
            gx <= param.radius || gy <= param.radius  ||
           stx >= (map->GetWidth() * map->GetCellSize()) - param.radius  ||
            gx >= (map->GetWidth() * map->GetCellSize()) - param.radius  ||
           sty >= (map->GetHeight() * map->GetCellSize()) - param.radius ||
            gy >= (map->GetHeight() * map->GetCellSize()) - param.radius)
        {
            std::cout <<"Start or goal position of agent "<<id<<" is out of map or too close to boundaries\n";
            correct = false;
        }


        for(auto agent : *allAgents)
        {
            float sqRadiusSum = static_cast<float>(std::pow((agent->GetRadius() + param.radius), 2.0));
            float sqDist = (Point(stx, sty) - agent->GetPosition()).SquaredEuclideanNorm();
            if(sqDist <= sqRadiusSum)
            {
                std::cout <<"Position of agent "<<id<<" is too close to another agent\n";
                correct = false;
                break;
            }
        }

        Node tmpStNode = map->GetClosestNode(Point(stx, sty));
        Node tmpGlNode = map->GetClosestNode(Point(gx, gy));
        LineOfSight positionChecker(param.radius/ map->GetCellSize());

        if(!positionChecker.checkTraversability(tmpStNode.i, tmpStNode.j, *map) || !positionChecker.checkTraversability(tmpGlNode.i, tmpGlNode.j, *map))
        {
            std::cout <<"Position of agent "<<id<<" is too close to some obstacle\n";
            correct = false;
        }

        if(!correct)
        {
            std::cout<<"Agent "<<id<< " was skipped\n\n";
            continue;
        }

        std::cout<<"Agent "<<id<< " was added at position "<< Point(stx, sty).ToString()<<"\n";
        Agent *a = new Agent(id, Point(stx, sty), Point(gx, gy), *map, *options, param);

        switch(plannertype)
        {
            case CN_SP_ST_THETA:
            {
                a->SetPlanner(ThetaStar(*map, *options, Point(stx, sty), Point(gx, gy), param.radius));
                break;
            }
            case CN_SP_ST_DIR:
            {
                a->SetPlanner(DirectPlanner(*map, *options, Point(stx, sty), Point(gx, gy), param.radius));
                break;
            }
            default:
            {
                a->SetPlanner(ThetaStar(*map, *options, Point(stx, sty), Point(gx, gy), param.radius));
                break;
            }
        }

        allAgents->push_back(a);
    }

    return true;
}

XMLReader &XMLReader::operator = (const XMLReader &obj)
{
    if (this != &obj)
    {
        fileName = obj.fileName;
        if(doc != nullptr)
        {
            delete doc;
        }

        if(obj.doc != nullptr)
        {
            doc = new XMLDocument();
            obj.doc->DeepCopy(doc);
        }
        else
        {
            doc = nullptr;
        }

        if(allAgents != nullptr)
        {
            delete allAgents;
        }
        allAgents = (obj.allAgents == nullptr) ? nullptr : new std::vector<Agent *>(*(obj.allAgents));

        if(map != nullptr)
        {
            delete map;
        }
        map = (obj.map == nullptr) ? nullptr : new Map(*obj.map);

        if(options != nullptr)
        {
            delete options;
        }
        options = (obj.options == nullptr) ? nullptr : new EnvironmentOptions(*obj.options);

        if(grid != nullptr)
        {
            delete grid;
        }
        grid = (obj.grid == nullptr) ? nullptr : new std::vector<std::vector<int>>(*obj.grid);

        if(obstacles != nullptr)
        {
            delete obstacles;
        }
        obstacles = (obj.obstacles == nullptr) ? nullptr : new std::vector<std::vector<Point>>(*obj.obstacles);
    }
    return *this;
}

XMLReader *XMLReader::Clone() const
{
    return new XMLReader(*this);
}


