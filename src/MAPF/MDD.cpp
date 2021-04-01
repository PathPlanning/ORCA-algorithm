#include "../../include/MAPF/MDD.h"


MDD::MDD() {};

int MDD::getLayerSize(int cost) const {
    if (cost > layerSizes.size()) {
        return 1;
    }
    return layerSizes[cost];
}
