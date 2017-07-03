#include "Bullet.h"
#include <algorithm>

std::map<std::string, Bullet> globalBullets;
void loadAllBullets() {
    std::vector<std::string> paths;
    listDirs("/res/bullets", paths);
    for (auto p : paths)
        globalBullets[p] = p;
}

Bullet & getBullet(uint16_t id) {
    auto i = globalBullets.begin();
    std::advance(i, id);
    return i->second;
}

void Bullet::operator=(const std::string & name) {
    INFO("Load bullet ", name);
    std::string full = "/res/bullets/" + name + "/";
    uniqueRAII<Scene> scene= Scene::load((full + "model.scene").c_str());
    mModel = scene->findNode("root")->clone();
    uniqueRAII<Properties> info = Properties::create((full + "bullet.info").c_str());
    mHitRadius = info->getFloat("radius");
    mBoomTime = info->getFloat("time");
    auto p = ParticleEmitter::create((full+"bullet.info#boom").c_str());
    mDuang = Node::create();
    mDuang->setDrawable(p);
}

Node* Bullet::getModel() const {
    return mModel->clone();
}

float Bullet::getRadius() const {
    return mHitRadius;
}

Node * Bullet::boom() {
    return mDuang->clone();
}

float Bullet::getBoomTime() const {
    return mBoomTime;
}

uint32_t BulletInstance::cnt = 0;

uint32_t BulletInstance::askID() {
    return ++cnt;
}

BulletInstance::BulletInstance(const std::string & kind, Vector3 begin, Vector3 end,
    float time, float harm, float radius,uint8_t group)
:BulletInstance(std::distance(globalBullets.begin(),globalBullets.find(kind)),
    begin,end,time,harm,radius,group){}

BulletInstance::BulletInstance(uint16_t kind, Vector3 begin,Vector3 end,
    float speed, float harm, float radius,uint8_t group)
    :mHarm(harm),mBegin(begin),mEnd(end),mCnt(0.0f),
    mTime(begin.distance(end)/speed),mRadius(radius),mKind(kind),mGroup(group){
    auto i = globalBullets.begin();
    std::advance(i, kind);
    mNode = i->second.getModel();
    mHitRadius = i->second.getRadius();
    mNode->setTranslation(begin);

    auto obj = end - begin;
    obj.normalize();

    correctVector(mNode.get(), &Node::getForwardVector, obj, M_PI, M_PI, M_PI);
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

uint8_t BulletInstance::getGroup() const {
    return mGroup;
}

Node * BulletInstance::getNode() const {
    return mNode.get();
}

