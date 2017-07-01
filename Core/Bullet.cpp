#include "Bullet.h"
#include <algorithm>

std::map<std::string, Bullet> globalBullets;
void loadAllBullets() {
    std::vector<std::string> paths;
    listDirs("/res/bullets", paths);
    for (auto p : paths)
        globalBullets[p] = p;
}

void Bullet::operator=(const std::string & name) {
    INFO("Load bullet ", name);
    std::string full = "/res/bullets/" + name + "/";
    uniqueRAII<Scene> scene= Scene::load((full + "model.scene").c_str());
    mModel = scene->findNode("root")->clone();
    uniqueRAII<Properties> info = Properties::create((full + "bullet.info").c_str());
    mHitRadius = info->getFloat("radius");
}

Node* Bullet::getModel() const {
    return mModel->clone();
}

float Bullet::getRadius() const {
    return mHitRadius;
}

uint32_t BulletInstance::cnt = 0;

uint32_t BulletInstance::askID() {
    return ++cnt;
}

BulletInstance::BulletInstance(const std::string & kind, Vector3 begin, Vector3 end,
    float time, float harm, float radius)
:BulletInstance(std::distance(globalBullets.begin(),globalBullets.find(kind)),
    begin,end,time,harm,radius){}

BulletInstance::BulletInstance(uint16_t kind, Vector3 begin,Vector3 end,
    float speed, float harm, float radius)
    :mHarm(harm),mBegin(begin),mEnd(end),mCnt(0.0f),
    mTime(begin.distance(end)/speed),mRadius(radius),mKind(kind){
    auto i = globalBullets.begin();
    std::advance(i, kind);
    mNode = i->second.getModel();
    mHitRadius = i->second.getRadius();
    mNode->setTranslation(begin);

    auto obj = end - begin;
    obj.normalize();

    auto dot = [&] {
        auto u = mNode->getForwardVector();
        u.normalize();
        return u.dot(obj);
    };

    constexpr auto unit = 0.001f;
    auto cd = dot();
#define TEST(a,b)\
    while (true) {\
        mNode->rotate##a((b));\
        auto nd = dot();\
        if (cd < nd)cd = nd;\
        else {\
            mNode->rotate##a(-(b));\
            break;\
        }\
    }\

    TEST(X, unit);
    TEST(X, -unit);
    TEST(Y, unit);
    TEST(Y, -unit);
    TEST(Z, unit);
    TEST(Z, -unit);
#undef TEST
}

void BulletInstance::update(float delta) {
    mCnt += delta;
    Vector3 pos=mBegin+(mEnd-mBegin)*std::min(mCnt,mTime)/mTime;
    mNode->setTranslation(pos);
}

BoundingSphere BulletInstance::getHitBound() {
    return { mNode->getTranslation(),(mCnt<mTime)?mHitRadius:100000.0f };
}

BoundingSphere BulletInstance::getBound() {
    return {mNode->getTranslation(),mRadius};
}

float BulletInstance::getHarm() const {
    return mHarm;
}

uint16_t BulletInstance::getKind() const {
    return mKind;
}

Node * BulletInstance::getNode() const {
    return mNode.get();
}
