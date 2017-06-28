#pragma once
#include "common.h"

class Map final {
private:
    uint16_t mWidth, mHeight;
    uniqueRAII<Terrain> mTerrain;
    std::vector<Vector2> mKeyPoint;
    bool mIsServer;
public:
    Map(const std::string& name,bool isServer);
    void set(Node* node);
    const std::vector<Vector2>& getKey() const;
    float getHeight(float x, float z) const;
};
