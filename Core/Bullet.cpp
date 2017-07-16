#include "Bullet.h"
#include <algorithm>
#include "Server.h"
#include "Client.h"

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
    uniqueRAII<Properties> info = Properties::create((full + "bullet.info").c_str());
    mModelPath = full + "model.scene";
    mHitRadius = info->getFloat("radius");
    mBoomTime = info->getFloat("time");
    auto p = ParticleEmitter::create((full + "bullet.info#boom").c_str());
    mDuang = Node::create();
    mDuang->setDrawable(p);
}

Node* Bullet::getModel() const {
    if (!mModel) {
        uniqueRAII<Scene> scene = Scene::load(mModelPath.c_str());
        mModel = scene->findNode("root")->clone();
    }
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
    float speed, float harm, float radius, uint8_t group, uint32_t obj, float angle)
    :BulletInstance(std::distance(globalBullets.begin(), globalBullets.find(kind)),
        begin, end, speed, harm, radius, group, obj, angle) {}

BulletInstance::BulletInstance(uint16_t kind, Vector3 begin, Vector3 end,
    float speed, float harm, float radius, uint8_t group, uint32_t object, float angle)
    : mHarm(harm), mBegin(begin), mEnd(end), mCnt(0.0f),
    mTime(begin.distance(end) / speed), mRadius(radius), mKind(kind)
    , mGroup(group), mObject(object), mAngle(angle) {
    auto i = globalBullets.begin();
    std::advance(i, kind);
    mNode = i->second.getModel();
    mHitRadius = i->second.getRadius();
    mNode->setTranslation(begin);

    if (mObject) {
        correctVector(mNode.get(), &Node::getForwardVector, end, M_PI, M_PI, M_PI);
        mTime = speed;
    }
    else {
        auto obj = end - begin;
        obj.normalize();

        correctVector(mNode.get(), &Node::getForwardVector, obj, M_PI, M_PI, M_PI);
    }
}

void BulletInstance::update(float delta) {
    if (mObject) {
        auto p = localServer->getUnitPos(mObject);
        if (p.isZero())mCnt = 1e10f;
        else {
            auto mp= mNode->getTranslation();
            if (mp.distanceSquared(p) >= 3e5f)
                p.y = std::max(p.y, localClient->getHeight(p.x, p.z)+500.0f);
            auto f = p - mp;
            correctVector(mNode.get(), &Node::getForwardVector, f.normalize(),
                mAngle*delta, mAngle*delta, 0.0f);
            mNode->translateForward(mTime*delta);
        }
    }
    else {
        mCnt += delta;
        Vector3 pos = mBegin + (mEnd - mBegin)*std::min(mCnt, mTime) / mTime;
        mNode->setTranslation(pos);
    }
}

BoundingSphere BulletInstance::getHitBound() {
    return { mNode->getTranslation(),(mCnt < mTime) ? mHitRadius : 1e10f };
}

BoundingSphere BulletInstance::getBound() {
    return { mNode->getTranslation(),mRadius };
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

