#pragma once
#include "common.h"

class Map final {
private:
    uniqueRAII<Terrain> mTerrain;
    std::vector<Vector2> mKeyPoint;
    float mSeaLevel;
public:
    Map(const std::string& name);
    void set(Node* node);
    const std::vector<Vector2>& getKey() const;
    float getHeight(float x, float z) const;
    Terrain* get() const;
};
