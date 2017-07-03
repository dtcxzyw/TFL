#pragma once
#include "common.h"

class Bullet final {
private:
    uniqueRAII<Node> mModel,mDuang;
    float mHitRadius,mBoomTime;
public:
    void operator=(const std::string& name);
    Node* getModel() const;
    float getRadius() const;
    Node* boom();
    float getBoomTime() const;
};

extern std::map<std::string, Bullet> globalBullets;
void loadAllBullets();
Bullet& getBullet(uint16_t id);

class BulletInstance final {
private:
    float mHarm,mTime,mCnt,mHitRadius,mRadius;
    uniqueRAII<Node> mNode;
    Vector3 mBegin, mEnd;
    uint16_t mKind;
    uint8_t mGroup;
    static uint32_t cnt;
public:
    static uint32_t askID();
    BulletInstance() {
        throw;
    }
    BulletInstance(const std::string& kind, Vector3 begin, Vector3 end, 
        float time, float harm, float radius, uint8_t group);
    BulletInstance(uint16_t kind, Vector3 begin,Vector3 end,
        float time,float harm,float radius, uint8_t group);
#ifdef ANDROID
    BulletInstance(const BulletInstance& rhs) {
        auto& r = const_cast<BulletInstance&>(rhs);
        mNode.swap(r.mNode);
        mHarm = rhs.mHarm;
        mTime = rhs.mTime;
        mCnt = rhs.mCnt;
        mHitRadius=rhs.mHitRadius;
        mRadius = rhs.mRadius;
        mBegin = rhs.mBegin;
        mEnd = rhs.mEnd;
        mKind = rhs.mKind;
    }
#endif
    void update(float delta);
    BoundingSphere getHitBound();
    BoundingSphere getBound();
    float getHarm() const;
    uint16_t getKind() const;
    Node* getNode() const;
    uint8_t getGroup() const;
};

