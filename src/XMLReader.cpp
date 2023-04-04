#include "XMLReader.h"


XMLReader::XMLReader(const std::string &file_name) {
	file = file_name;
	doc = new XMLDocument();
	all_agents = new std::vector<Agent *>();
	all_goals = std::vector<Point>();

	map = nullptr;
	options = nullptr;
	grid = nullptr;
	obstacles = nullptr;
	root = nullptr;
	planner_type = 0;
}


XMLReader::XMLReader() {
	file = "";
	doc = nullptr;
	all_agents = nullptr;
	map = nullptr;
	options = nullptr;
	grid = nullptr;
	obstacles = nullptr;
	root = nullptr;
	planner_type = 0;
}


XMLReader::XMLReader(const XMLReader &obj) {

	file = obj.file;
	if (doc != nullptr) {
		delete doc;
		doc = nullptr;
	}


	if (obj.doc != nullptr) {
		doc = new XMLDocument();
		obj.doc->DeepCopy(doc);
		root = doc->FirstChildElement(CNS_TAG_ROOT);

	}


	planner_type = obj.planner_type;
	all_agents = (obj.all_agents == nullptr) ? nullptr : new std::vector<Agent *>(*(obj.all_agents));
	map = (obj.map == nullptr) ? nullptr : new Map(*obj.map);
	options = (obj.options == nullptr) ? nullptr : new EnvironmentOptions(*obj.options);
	grid = (obj.grid == nullptr) ? nullptr : new std::vector<std::vector<int>>(*obj.grid);
	obstacles = (obj.obstacles == nullptr) ? nullptr : new std::vector<std::vector<Point>>(*obj.obstacles);
}


XMLReader::~XMLReader() {

	if (all_agents != nullptr) {
		delete all_agents;
		all_agents = nullptr;
	}

	if (options != nullptr) {
		delete options;
		options = nullptr;
	}

	if (map != nullptr) {
		delete map;
		map = nullptr;
	}
	if (grid != nullptr) {
		delete grid;
		grid = nullptr;
	}
	if (obstacles != nullptr) {
		delete obstacles;
		obstacles = nullptr;
	}

	if (doc != nullptr) {
		delete doc;
		doc = nullptr;
		root = nullptr;
	}

}


bool XMLReader::getMap(Map **map) {
	if (this->map != nullptr) {
		*map = this->map;
		this->map = nullptr;
		return true;
	}

	return false;
}


bool XMLReader::getEnvironmentOptions(EnvironmentOptions **env_opt) {
	if (this->options != nullptr) {
		*env_opt = this->options;
		this->options = nullptr;
		return true;
	}
	return false;
}


bool XMLReader::getAgents(std::vector<Agent *> &agents, const int &num_threshold) {
	std::vector<Point> goals;
	if (all_agents == nullptr || all_agents->size() < num_threshold) {
		return false;
	}

	if (agent_type_str == CNS_AT_ST_GOALEXCHANGE){
		goals = {all_goals.begin(), all_goals.begin()+num_threshold};
	}

	agents.clear();
	for (int i = 0; i < num_threshold; i++) {
		if (agent_type_str == CNS_AT_ST_GOALEXCHANGE) {
			dynamic_cast<AgentGoalExchange*>(all_agents->at(i))->setGoalList(goals);
		}
		agents.push_back((*all_agents)[i]);
	}

	for (int i = num_threshold; i < all_agents->size(); i++) {
		delete (*all_agents)[i];
	}
	all_agents->clear();
	delete all_agents;
	all_agents = nullptr;
	return true;
}


bool XMLReader::readData() {

	if (doc == nullptr || all_agents == nullptr) {
		return false;
	}

	/* Reading task from XML-file */

	if (doc->LoadFile(file.c_str()) != XMLError::XML_SUCCESS) {
		std::cout << "Error opening input XML file\n";
		return false;
	}

	root = doc->FirstChildElement(CNS_TAG_ROOT);
	if (!root) {
		std::cout << CNS_TAG_ROOT << " element not found in XML file\n";
		return false;
	}

	return readMap() && readAlgorithmOptions() && readAgents();
}

XMLReader &XMLReader::operator=(const XMLReader &obj) {
	if (this != &obj) {
		file = obj.file;
		if (doc != nullptr) {
			delete doc;
		}

		if (obj.doc != nullptr) {
			doc = new XMLDocument();
			obj.doc->DeepCopy(doc);
			root = root = doc->FirstChildElement(CNS_TAG_ROOT);
		}
		else {
			doc = nullptr;
			root = nullptr;
		}

		if (all_agents != nullptr) {
			delete all_agents;
		}
		all_agents = (obj.all_agents == nullptr) ? nullptr : new std::vector<Agent *>(*(obj.all_agents));

		if (map != nullptr) {
			delete map;
		}
		map = (obj.map == nullptr) ? nullptr : new Map(*obj.map);

		if (options != nullptr) {
			delete options;
		}
		options = (obj.options == nullptr) ? nullptr : new EnvironmentOptions(*obj.options);

		if (grid != nullptr) {
			delete grid;
		}
		grid = (obj.grid == nullptr) ? nullptr : new std::vector<std::vector<int>>(*obj.grid);

		if (obstacles != nullptr) {
			delete obstacles;
		}
		obstacles = (obj.obstacles == nullptr) ? nullptr : new std::vector<std::vector<Point>>(*obj.obstacles);

		planner_type = obj.planner_type;
	}
	return *this;
}

XMLReader *XMLReader::Clone() const {
	return new XMLReader(*this);
}

bool XMLReader::readMap() {
	XMLElement *tmpElement;
	// Чтение информации о карте и препятствиях / Reading information about map and obstacles //

	XMLElement *mapTag = 0, *element = 0, *mapnode, *obsts = 0, *obstElem = 0;

	std::string value;
	std::stringstream stream;

	bool hasGridMem = false, hasGrid = false, hasHeight = false, hasWidth = false, hasCellSize = false;
	int height = -1, width = -1, rowIter = 0, gridI = 0, gridJ = 0, obstNum = -1, count = 0;
	float cellSize;

	// Чтение карты / Map reading //

	mapTag = root->FirstChildElement(CNS_TAG_MAP);
	if (!mapTag) {
		std::cout << CNS_TAG_MAP << " element not found in XML file\n";
		return false;
	}

	for (mapnode = mapTag->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement()) {
		element = mapnode->ToElement();
		value = mapnode->Value();
		std::transform(value.begin(), value.end(), value.begin(), ::tolower);

		stream.str("");
		stream.clear();

		if (value != CNS_TAG_GRID) {
			stream << element->GetText();
		}

		if (!hasGridMem && hasHeight && hasWidth) {
			grid = new std::vector<std::vector<int>>(height);
			for (int i = 0; i < height; ++i)
				(*grid)[i] = std::vector<int>(width);
			hasGridMem = true;
		}

		if (value == CNS_TAG_HEIGHT) {
			if (hasHeight) {
#if FULL_OUTPUT
				std::cout << "Warning! Duplicate '" << CNS_TAG_HEIGHT << "' encountered." << std::endl;
				std::cout << "Only first value of '" << CNS_TAG_HEIGHT << "' =" << height << "will be used." << std::endl;
#endif
			}
			else {
				if (!((stream >> height) && (height > 0))) {
#if FULL_OUTPUT
					std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
							  << "' tag encountered (or could not convert to integer)." << std::endl;
					std::cout << "Value of '" << CNS_TAG_HEIGHT << "' tag should be an integer >=0" << std::endl;
					std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_HEIGHT
							  << "' tag will be encountered later..." << std::endl;
#endif
				}
				else {
					hasHeight = true;
				}
			}
		}
		else if (value == CNS_TAG_WIDTH) {
			if (hasWidth) {
#if FULL_OUTPUT
				std::cout << "Warning! Duplicate '" << CNS_TAG_WIDTH << "' encountered." << std::endl;
				std::cout << "Only first value of '" << CNS_TAG_WIDTH << "' =" << width << "will be used." << std::endl;
#endif
			}
			else {
				if (!((stream >> width) && (width > 0))) {
#if FULL_OUTPUT
					std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
							  << "' tag encountered (or could not convert to integer)." << std::endl;
					std::cout << "Value of '" << CNS_TAG_WIDTH << "' tag should be an integer AND >0" << std::endl;
					std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_WIDTH
							  << "' tag will be encountered later..." << std::endl;
#endif
				}
				else {
					hasWidth = true;
				}
			}
		}
		else if (value == CNS_TAG_CELLSIZE) {
			if (hasCellSize) {
#if FULL_OUTPUT
				std::cout << "Warning! Duplicate '" << CNS_TAG_CELLSIZE << "' encountered." << std::endl;
				std::cout << "Only first value of '" << CNS_TAG_CELLSIZE << "' =" << cellSize << "will be used."
						  << std::endl;
#endif
			}
			else {
				if (!((stream >> cellSize) && (cellSize > 0))) {
#if FULL_OUTPUT
					std::cout << "Warning! Invalid value of '" << CNS_TAG_CELLSIZE
							  << "' tag encountered (or could not convert to double)." << std::endl;
					std::cout << "Value of '" << CNS_TAG_CELLSIZE
							  << "' tag should be double AND >0. By default it is defined to '1'" << std::endl;
					std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_CELLSIZE
							  << "' tag will be encountered later..." << std::endl;
#endif
				}
				else
					hasCellSize = true;
			}
		}
		else if (value == CNS_TAG_GRID) {
			hasGrid = true;
			if (!(hasHeight && hasWidth)) {
				std::cout << "Error! No '" << CNS_TAG_WIDTH << "' tag or '" << CNS_TAG_HEIGHT << "' tag before '"
						  << CNS_TAG_GRID << "'tag encountered!" << std::endl;
				return false;
			}
			element = mapnode->FirstChildElement();
			while (gridI < height) {
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
				while (std::getline(ss, item, ' ')) {
					elems.push_back(item);
				}
				rowIter = gridJ = 0;
				int val;
				if (elems.size() > 0) {
					for (gridJ = 0; gridJ < width; ++gridJ) {
						if (gridJ == elems.size()) {
							break;
						}
						stream.str("");
						stream.clear();
						stream << elems[gridJ];
						stream >> val;
						(*grid)[gridI][gridJ] = val;
					}
				}
				if (gridJ != width) {
					std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << gridI + 1 << " " << CNS_TAG_ROW
							  << std::endl;
					return false;
				}
				++gridI;

				element = element->NextSiblingElement();
			}
		}
	}

	if (!hasGrid) {
		std::cout << CNS_TAG_GRID << " element not found in XML file\n";
		return false;
	}

	// Чтение препятствий / Obstacles reading //
	obsts = root->FirstChildElement(CNS_TAG_OBSTS);
	if (!obsts) {
		std::cout << CNS_TAG_OBSTS << " element not found in XML file\n";
		return false;
	}

	if (obsts->QueryIntAttribute(CNS_TAG_ATTR_NUM, &obstNum) != XMLError::XML_SUCCESS) {
		std::cout << CNS_TAG_ATTR_NUM << " element not found at " << CNS_TAG_OBSTS << " tag in XML file\n";
		return false;
	}

	obstacles = new std::vector<std::vector<Point>>(obstNum);
	float oX, oY;
	obstElem = obsts->FirstChildElement(CNS_TAG_OBST);
	for (count = 0; obstElem && count < obstNum; obstElem = obstElem->NextSiblingElement(CNS_TAG_OBST), count++) {
		for (tmpElement = obstElem->FirstChildElement(
				CNS_TAG_VERTEX); tmpElement; tmpElement = tmpElement->NextSiblingElement(CNS_TAG_VERTEX)) {
			if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_X, &oX) != XMLError::XML_SUCCESS) {
				std::cout << CNS_TAG_ATTR_X << " element not found at " << CNS_TAG_OBST << " #" << count + 1
						  << " tag in XML file\n";
				return false;
			}
			if (tmpElement->QueryFloatAttribute(CNS_TAG_ATTR_Y, &oY) != XMLError::XML_SUCCESS) {
				std::cout << CNS_TAG_ATTR_Y << " element not found at " << CNS_TAG_OBST << " #" << count + 1
						  << " tag in XML file\n";
				return false;
			}
			(*obstacles)[count].push_back(Point(oX, oY));
		}

	}

	if (count != obstNum) {
		std::cout << "The number of obstacles does not match the stated in " << CNS_TAG_ATTR_NUM << "attribute of "
				  << CNS_TAG_OBSTS << " tag\n";
		return false;
	}

	map = new Map(cellSize, *grid, *obstacles);
	return true;
}


// TODO Refactor tag reading as loop and refactor constants
bool XMLReader::readAlgorithmOptions() {
	XMLElement *tmp_element;
	bool tag_flag;

	/* Reading options of the environment and algorithm */
	options = new EnvironmentOptions(CN_DEFAULT_METRIC_TYPE, CN_DEFAULT_BREAKINGTIES, CN_DEFAULT_ALLOWSQUEEZE,
									 CN_DEFAULT_CUTCORNERS,
									 CN_DEFAULT_HWEIGHT, CN_DEFAULT_TIME_STEP, CN_DEFAULT_DELTA,
									 CN_DEFAULT_MAPF_TRIGGER, CN_DEFAULT_MAPFNUM);

	XMLElement *alg = root->FirstChildElement(CNS_TAG_ALG);
	if (!alg) {
		std::cout << CNS_TAG_ALG << " element not found in XML file\n";
		return false;
	}

	/* Planner type */
	planner_type = CN_DEFAULT_ST;
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_ST)) && tmp_element->GetText();
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_ST <<" element not found in XML file. It was defined to "<< CNS_DEFAULT_ST <<"\n";
#endif
	}
	else {
		auto tmpst = std::string(tmp_element->GetText());
		if (tmpst == CNS_SP_ST_THETA) {
			planner_type = CN_SP_ST_THETA;
		}
		else if (tmpst == CNS_SP_ST_DIR) {
			planner_type = CN_SP_ST_DIR;
		}
		else {
			std::cout << CNS_TAG_ST << " element are incorrect. It was defined to " << CNS_DEFAULT_ST << "\n";
		}
	}

	/* BREAKINGTIES */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_BT)) &&
			   (tmp_element->QueryBoolText(&(options->breakingties)) == XMLError::XML_SUCCESS);
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_BT <<" element not found in XML file. It was defined to "<< CN_DEFAULT_BREAKINGTIES<<"\n";
#endif
	}

	/* ALLOWSQUEEZE */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_AS)) &&
			   (tmp_element->QueryBoolText(&(options->allowsqueeze)) == XMLError::XML_SUCCESS);
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_AS <<" element not found in XML file. It was defined to "<< CN_DEFAULT_ALLOWSQUEEZE<<"\n";
#endif
	}

	/* CUTCORNERS */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_CC)) &&
			   (tmp_element->QueryBoolText(&(options->cutcorners)) == XMLError::XML_SUCCESS);
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_CC <<" element not found in XML file. It was defined to "<< CN_DEFAULT_CUTCORNERS<<"\n";
#endif
	}

	/* HWEIGHT */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_HW)) &&
			   (tmp_element->QueryFloatText(&(options->hweight)) == XMLError::XML_SUCCESS);
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_HW <<" element not found in XML file. It was defined to "<< CN_DEFAULT_HWEIGHT<<"\n";
#endif
	}

	/* TIME_STEP */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_TS)) &&
			   (tmp_element->QueryFloatText(&(options->timestep)) == XMLError::XML_SUCCESS);
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_TS <<" element not found in XML file. It was defined to "<< CN_DEFAULT_TIME_STEP<<"\n";
#endif
	}

	/* DELTA */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_DEL)) &&
			   (tmp_element->QueryFloatText(&(options->delta)) == XMLError::XML_SUCCESS);
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_DEL <<" element not found in XML file. It was defined to "<< CN_DEFAULT_DELTA<<"\n";
#endif
	}

	/* MAPF_TRIGGER */
	std::string MAPFTrigStr = CNS_DEFAULT_MAPF_TRIGGER;
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_TR)) && tmp_element->GetText();
	if (!tag_flag) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_TR <<" element not found in XML file. It was defined to "<< CNS_DEFAULT_MAPF_TRIGGER<<"\n";
#endif
	}
	else {
		MAPFTrigStr = std::string(tmp_element->GetText());
		if (MAPFTrigStr == CNS_AT_ST_COMMONPOINT) {
			options->trigger = COMMON_POINT;
		}
		else if (MAPFTrigStr == CNS_AT_ST_SPEEDBUFFER) {
			options->trigger = SPEED_BUFFER;
		}
		else {
			std::cout << CNS_TAG_TR << " element are incorrect. It was defined to " << CNS_DEFAULT_MAPF_TRIGGER << "\n";
		}
	}

	/* MAPFNUM */
	tag_flag = (tmp_element = alg->FirstChildElement(CNS_TAG_MN)) &&
			   (tmp_element->QueryIntText(&(options->MAPFNum)) == XMLError::XML_SUCCESS);
	if (!tag_flag && (options->MAPFNum >= 0)) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_MN <<" element not found in XML file. It was defined to "<< CN_DEFAULT_MAPFNUM<<"\n";
#endif
		options->MAPFNum = CN_DEFAULT_MAPFNUM;
	}

	return true;
}

bool XMLReader::readAgents() {
	XMLElement *tmp_element;
	int count;
	// Чтение информации о агентах// Reading agents information //

	AgentParam default_param = AgentParam();
	int agents_number;
	const char *ag_type = CNS_DEFAULT_AGENT_TYPE;
	const char *ag_trig_c_str = CNS_DEFAULT_MAPF_TRIGGER;


	XMLElement *agents = root->FirstChildElement(CNS_TAG_AGENTS);
	if (!agents) {
		std::cout << CNS_TAG_AGENTS << " element not found in XML file\n";
		return false;
	}

	/* Agent number */
	if (agents->QueryIntAttribute(CNS_TAG_ATTR_NUM, &agents_number) != XMLError::XML_SUCCESS) {
		std::cout << CNS_TAG_ATTR_NUM << " element not found at " << CNS_TAG_AGENTS << " tag in XML file\n";
		return false;
	}

	/* Agents type */
	if (agents->QueryStringAttribute(CNS_TAG_ATTR_TYPE, &ag_type) != XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_ATTR_TYPE <<" element not found at " << CNS_TAG_AGENTS << " tag in XML file. It was defined to "<< CNS_DEFAULT_AGENT_TYPE<<"\n";
#endif
	}
	agent_type_str = std::string(ag_type);

	/* Default parameters */
	tmp_element = agents->FirstChildElement(CNS_TAG_DEF_PARAMS);
	if (!tmp_element) {
#if FULL_OUTPUT
		std::cout <<CNS_TAG_DEF_PARAMS <<" element not found in XML file. The following default values was defined\n";
		std::cout << "Default " <<CNS_TAG_ATTR_SIZE << " = " << CN_DEFAULT_SIZE <<"\n";
		std::cout << "Default " <<CNS_TAG_ATTR_MAXSPEED << " = " << CN_DEFAULT_MAX_SPEED <<"\n";
		std::cout << "Default " <<CNS_TAG_ATTR_AGENTSMAXNUM << " = " << CN_DEFAULT_AGENTS_MAX_NUM <<"\n";
		std::cout << "Default " <<CNS_TAG_ATTR_TIMEBOUNDARY << " = " << CN_DEFAULT_TIME_BOUNDARY <<"\n";
		std::cout << "Default " <<CNS_TAG_ATTR_SIGHTRADIUS << " = " << CN_DEFAULT_RADIUS_OF_SIGHT <<"\n";
		std::cout << "Default " <<CNS_TAG_ATTR_TIMEBOUNDARYOBST << " = " << CN_DEFAULT_OBS_TIME_BOUNDARY <<"\n";
		std::cout << "Default " <<CNS_TAG_ATTR_REPS << " = " << CN_DEFAULT_REPS <<"\n";
#endif
	}
	else {
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_SIZE, &default_param.radius) != XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_SIZE <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_SIZE<<"\n";
#endif
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_MAXSPEED, &default_param.maxSpeed) != XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_MAXSPEED <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_MAX_SPEED<<"\n";
#endif
		}
		if (tmp_element->QueryIntAttribute(CNS_TAG_ATTR_AGENTSMAXNUM, &default_param.agentsMaxNum) !=
			XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_AGENTSMAXNUM <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_AGENTS_MAX_NUM<<"\n";
#endif
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARY, &default_param.timeBoundary) !=
			XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_TIMEBOUNDARY <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_TIME_BOUNDARY<<"\n";
#endif
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_SIGHTRADIUS, &default_param.sightRadius) !=
			XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_SIGHTRADIUS <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_RADIUS_OF_SIGHT<<"\n";
#endif
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARYOBST, &default_param.timeBoundaryObst) !=
			XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_TIMEBOUNDARYOBST <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_OBS_TIME_BOUNDARY<<"\n";
#endif
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_REPS, &default_param.rEps) != XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_REPS <<" element not found at "<< CNS_TAG_DEF_PARAMS << " tag. It was defined to "<< CN_DEFAULT_REPS<<"\n";
#endif
		}
	}
	tmp_element = agents->FirstChildElement(CNS_TAG_AGENT);

	int id;
	float stx = 0, sty = 0, gx = 0, gy = 0, stt = CN_DEFAULT_START_THETA;

	/* Goal list for AMAPF */
	all_goals.reserve(agents_number);

	/* Each individual agent */
	for (count = 0; tmp_element; tmp_element = tmp_element->NextSiblingElement(CNS_TAG_AGENT), count++) {
		AgentParam param = AgentParam(default_param);
		stt = CN_DEFAULT_START_THETA;
		bool correct = true;

		/* Agent's ID */
		if (tmp_element->QueryIntAttribute(CNS_TAG_ATTR_ID, &id) != XMLError::XML_SUCCESS) {
			std::cout << CNS_TAG_ATTR_ID << " element not found in XML file at agent " << count << "\n";
			return false;
		}
		for (auto agent: *all_agents) {
			if (id == agent->getID()) {
				std::cout << "There is an agent with same ID (" << id << ")\n";
				correct = false;
			}
		}

		/* Agent's params */
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_STX, &stx) != XMLError::XML_SUCCESS) {
			std::cout << CNS_TAG_ATTR_STX << " element not found in XML file at agent" << count << "\n";
			return false;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_STY, &sty) != XMLError::XML_SUCCESS) {
			std::cout << CNS_TAG_ATTR_STY << " element not found in XML file at agent " << id << "\n";
			return false;
		}
		if (agent_type_str == CNS_AT_ST_ORCADD and
			tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_STT, &stt) != XMLError::XML_SUCCESS) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_STT << " element not found in XML file at agent " << id << " It was defined to " << CN_DEFAULT_START_THETA << "\n";
#endif
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_GX, &gx) != XMLError::XML_SUCCESS) {
			std::cout << CNS_TAG_ATTR_GX << " element not found in XML file at agent " << id << "\n";
			return false;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_GY, &gy) != XMLError::XML_SUCCESS) {
			std::cout << CNS_TAG_ATTR_GY << " element not found in XML file at agent " << id << "\n";
			return false;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_SIZE, &param.radius) != XMLError::XML_SUCCESS ||
			param.radius <= 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_SIZE <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.radius = default_param.radius;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_MAXSPEED, &param.maxSpeed) != XMLError::XML_SUCCESS ||
			param.maxSpeed <= 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_MAXSPEED <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.maxSpeed = default_param.maxSpeed;
		}
		if (tmp_element->QueryIntAttribute(CNS_TAG_ATTR_AGENTSMAXNUM, &param.agentsMaxNum) != XMLError::XML_SUCCESS ||
			param.agentsMaxNum <= 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_AGENTSMAXNUM <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.agentsMaxNum = default_param.agentsMaxNum;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARY, &param.timeBoundary) != XMLError::XML_SUCCESS ||
			param.timeBoundary <= 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_TIMEBOUNDARY <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.timeBoundary = default_param.timeBoundary;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_SIGHTRADIUS, &param.sightRadius) != XMLError::XML_SUCCESS ||
			param.sightRadius <= 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_SIGHTRADIUS <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.sightRadius = default_param.sightRadius;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_TIMEBOUNDARYOBST, &param.timeBoundaryObst) !=
			XMLError::XML_SUCCESS || param.timeBoundaryObst <= 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_TIMEBOUNDARYOBST <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.timeBoundaryObst = default_param.timeBoundaryObst;
		}
		if (tmp_element->QueryFloatAttribute(CNS_TAG_ATTR_REPS, &param.rEps) != XMLError::XML_SUCCESS ||
			param.rEps < 0) {
#if FULL_OUTPUT
			std::cout <<CNS_TAG_ATTR_REPS <<" element not found in XML file (or it is incorrect) at agent "<<id<<"\n";
#endif
			param.rEps = default_param.rEps;
		}

		/* Checking params */
		if (stx <= param.radius || sty <= param.radius ||
			gx <= param.radius || gy <= param.radius ||
			stx >= (map->GetWidth() * map->GetCellSize()) - param.radius ||
			gx >= (map->GetWidth() * map->GetCellSize()) - param.radius ||
			sty >= (map->GetHeight() * map->GetCellSize()) - param.radius ||
			gy >= (map->GetHeight() * map->GetCellSize()) - param.radius) {
			std::cout << "Start or goal position of agent " << id << " is out of map or too close to boundaries\n";
			correct = false;
		}

		for (auto agent: *all_agents) {
			float sqRadiusSum = static_cast<float>(std::pow((agent->getRadius() + param.radius), 2.0));
			float sqDist = (Point(stx, sty) - agent->getPosition()).SquaredEuclideanNorm();
			if (sqDist <= sqRadiusSum) {
				std::cout << "Position of agent " << id << " is too close to another agent\n";
				correct = false;
				break;
			}
		}
		Node tmpStNode = map->GetClosestNode(Point(stx, sty));
		Node tmpGlNode = map->GetClosestNode(Point(gx, gy));
		LineOfSight positionChecker(param.radius / map->GetCellSize());
		if (!positionChecker.checkTraversability(tmpStNode.i, tmpStNode.j, *map) ||
			!positionChecker.checkTraversability(tmpGlNode.i, tmpGlNode.j, *map)) {
			std::cout << "Position of agent " << id << " is too close to some obstacle\n";
			correct = false;
		}
		if (!correct) {
#if FULL_OUTPUT
			std::cout<<"Agent "<<id<< " was skipped\n\n";
#endif
			continue;
		}
#if FULL_OUTPUT
		std::cout<<"Agent "<<id<< " was added at position "<< Point(stx, sty).ToString()<<"\n";
#endif

		/* Creating of an agent */
		Agent *a;
		if (agent_type_str == CNS_AT_ST_ORCA) {
			a = new ORCAAgent(id, Point(stx, sty), Point(gx, gy), *map, *options, param);
		}
		else if (agent_type_str == CNS_AT_ST_ORCADD) {
			a = new ORCADDAgent(id, Point(stx, sty), Point(gx, gy), *map, *options, param,
								2 * (param.radius + param.rEps), 2 * (param.radius), stt);
		}
		else if (agent_type_str == CNS_AT_ST_ORCAPAR) {
			a = new ORCAAgentWithPAR(id, Point(stx, sty), Point(gx, gy), *map, *options, param);
		}
		else if (agent_type_str == CNS_AT_ST_ORCAECBS) {
			a = new ORCAAgentWithECBS(id, Point(stx, sty), Point(gx, gy), *map, *options, param);
		}
		else if (agent_type_str == CNS_AT_ST_ORCAPARECBS) {
			a = new ORCAAgentWithPARAndECBS(id, Point(stx, sty), Point(gx, gy), *map, *options, param);
		}
		else if (agent_type_str == CNS_AT_ST_ORCARETURN) {
			a = new ORCAAgentWithReturning(id, Point(stx, sty), Point(gx, gy), *map, *options, param);
		}
		else if (agent_type_str == CNS_AT_ST_GOALEXCHANGE) {
			a = new AgentGoalExchange(id, Point(stx, sty), *map, *options, param);
			all_goals.emplace_back(gx, gy);
		}
		else {
			a = new ORCAAgent(id, Point(stx, sty), Point(gx, gy), *map, *options, param);
		}

		/* Set agent's planner */
		switch (planner_type) {
			case CN_SP_ST_THETA: {
				a->setGlobalPlanner(
						ThetaStar(*map, *options, Point(stx, sty), Point(gx, gy), param.radius + param.rEps));
				break;
			}
			case CN_SP_ST_DIR: {
				a->setGlobalPlanner(
						DirectPlanner(*map, *options, Point(stx, sty), Point(gx, gy), param.radius + param.rEps));
				break;
			}
			default: {
				a->setGlobalPlanner(
						ThetaStar(*map, *options, Point(stx, sty), Point(gx, gy), param.radius + param.rEps));
				break;
			}
		}
		all_agents->push_back(a);
	}
	return true;
}

