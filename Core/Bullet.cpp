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
    :mHarm(harm),mBegin(begin),mEnd(mEnd),mCnt(0.0f),
    mTime(begin.distance(end)/speed),mRadius(radius),mKind(kind){
    auto i = globalBullets.begin();
    std::advance(i, kind);
    mNode = i->second.getModel();
    mHitRadius = i->second.getRadius();
    mNode->setTranslation(begin);
    Matrix mat;
    Matrix::createLookAt(begin, end, Vector3::unitY(), &mat);
    mNode->setRotation(mat);
}

void BulletInstance::set(Scene * scene) {
    scene->addNode(mNode.get());
}

void BulletInstance::update(float delta) {
    mCnt += delta;
    Vector3 pos=mBegin+(mEnd-mBegin)*std::min(mCnt,mTime)/mTime;
    mNode->setTranslation(pos);
    mNode->rotateZ(delta*0.001f);
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
