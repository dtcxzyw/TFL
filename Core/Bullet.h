#pragma once
#include "common.h"

class Bullet final {
private:
    mutable uniqueRAII<Node> mModel;
    uniqueRAII<Node> mDuang;
    float mHitRadius,mBoomTime;
    std::string mModelPath;
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
    float mHarm,mSpeed,mCnt,mHitRadius,mRadius,mTime;
    uniqueRAII<Node> mNode;
    Vector3 mEnd;
    uint16_t mKind;
    uint8_t mGroup;
    uint32_t mObject;
    float mAngle;
    static uint32_t cnt;
public:
    static uint32_t askID();
    BulletInstance();
#ifdef ANDROID
    BulletInstance(const BulletInstance& src);
#endif
    BulletInstance(const std::string& kind, Vector3 begin, Vector3 end, Vector3 forward,
        float speed, float harm, float radius, uint8_t group, uint32_t obj=0,float angle=0.0f);
    BulletInstance(uint16_t kind, Vector3 begin,Vector3 end,
        float speed,float harm,float radius, uint8_t group, uint32_t obj=0,float angle=0.0f);
    void update(float delta);
    BoundingSphere getHitBound();
    BoundingSphere getBound();
    float getHarm() const;
    uint16_t getKind() const;
    Node* getNode() const;
    uint8_t getGroup() const;
    void updateClient(float delta);
};

