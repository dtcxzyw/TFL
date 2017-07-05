#include "Map.h"
#include <string>
#include <vector>
#include <fstream>
#include <Server.h>

Map::Map(const std::string & name) {
    std::string full = "/res/maps/" + name + "/";
    std::string map = full + "map.terrain";
    mTerrain = Terrain::create(map.c_str());

    mTerrain->setFlag(Terrain::Flags::FRUSTUM_CULLING, true);
    mTerrain->setFlag(Terrain::Flags::LEVEL_OF_DETAIL, false);

    uniqueRAII<Properties> info = Properties::create((full + "map.info").c_str());
    const char* id;
    Vector2 tmp;
    while ((id = info->getNextProperty()) && info->getVector2(id, &tmp)) {
        mKeyPoint.emplace_back(tmp / 512.0f* mapSizeF - Vector2{ mapSizeHF, mapSizeHF });
        mTerrain->setLayer(2, "res/common/key.png", { 16.0f,16.0f },
            "res/common/cover.png", 0, tmp.y / 32, tmp.x / 32);
    }

    if (mKeyPoint.size() < 4) GP_ERROR("The number of the map must be bigger than 3.");

}

void Map::set(Node* node) {
    node->setDrawable(mTerrain.get());
}

const std::vector<Vector2>& Map::getKey() const {
    return mKeyPoint;
}

float Map::getHeight(float x, float z) const {
    return mTerrain->getHeight(x, z);
}
