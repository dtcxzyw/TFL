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
    uniqueRAII<ParticleEmitter> p = ParticleEmitter::create((full + "bullet.info#boom").c_str());
    mDuang = Node::create();
    mDuang->setDrawable(p.get());
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

BulletInstance::BulletInstance(const std::string & kind, Vector3 begin, Vector3 end,Vector3 forward,
    float speed, float harm, float radius, uint8_t group, uint32_t obj, float angle)
    :BulletInstance(std::distance(globalBullets.begin(), globalBullets.find(kind)),
        begin, end, speed, harm, radius, group, obj, angle) {
    correctVector(mNode.get(), &Node::getForwardVector, forward, M_PI, M_PI, 0.0f);
}

BulletInstance::BulletInstance(uint16_t kind, Vector3 begin, Vector3 end,
    float speed, float harm, float radius, uint8_t group, uint32_t object, float angle)
    : mHarm(harm), mEnd(end), mCnt(0.0f),
    mSpeed(speed), mRadius(radius), mKind(kind),mTime(1e5f)
    , mGroup(group), mObject(object), mAngle(angle) {
    auto i = globalBullets.begin();
    std::advance(i, kind);
    mNode = i->second.getModel();
    mHitRadius = i->second.getRadius();
    mNode->setTranslation(begin);
    mSpeed /= 1000.0f;

    if (!mObject)
        mTime = begin.distance(end) / mSpeed*1.5f;
}

void BulletInstance::update(float delta) {

    if (mObject) {
        auto p =mObject==pointID?mEnd :localServer->getUnitPos(mObject);
        if (p.isZero())mHitRadius = 1e10f;
        else {
            auto mp= mNode->getTranslation();
            auto hl = localClient->getHeight(mp.x, mp.z) + 300.0f;
            Vector3 f;
            auto dis = mp.distanceSquared(p);
            if (mp.y > hl || dis < 3e5f) {
                if (dis >= 3e5f)
                    p.y = std::max(p.y, localClient->getHeight(p.x, p.z) + 500.0f);
                f = p - mp;
            }
            else f = Vector3{ 0.0f,hl - mp.y,0.0f };
            correctVector(mNode.get(), &Node::getForwardVector, f.normalize(),
                mAngle*delta, mAngle*delta, 0.0f);
            mNode->translateForward(mSpeed*delta);
            mNode->rotateZ(mAngle*delta);
        }
    }
    else {
        mCnt += delta;
        auto now = mNode->getTranslation();
        if (mCnt > 0.3f*mTime) {
            constexpr auto angle = 0.05f;
            auto f =mEnd-now;
            correctVector(mNode.get(), &Node::getForwardVector, f.normalize(),
                angle*delta, angle*delta, 0.0f);
        }
        auto dis = mEnd.distance(now);
        if(dis>=mHitRadius)
            mNode->translateForward(std::min(mSpeed*delta, dis));
        else
            mHitRadius=1e10f;
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

void BulletInstance::updateClient(float delta) {
    std::function<void(Node*)> updateFire = [&](Node* node) {
        auto p = dynamic_cast<ParticleEmitter*>(node->getDrawable());
        if (p) {
            if (!p->isStarted())p->start();
            p->update(delta);
        }

        for (auto i = node->getFirstChild(); i; i = i->getNextSibling())
            updateFire(i);
    };

    updateFire(mNode.get());
}

Node * BulletInstance::getNode() const {
    return mNode.get();
}

BulletInstance::BulletInstance() {
    throw;
}

#ifdef ANDROID
BulletInstance::BulletInstance(const BulletInstance& src) {
    auto&& rhs = const_cast<BulletInstance&>(src);
    mHarm = rhs.mHarm;
    mSpeed = rhs.mSpeed;
    mCnt = rhs.mCnt;
    mHitRadius = rhs.mHitRadius;
    mRadius = rhs.mRadius;
    mTime = rhs.mTime;
    mNode.swap(rhs.mNode);
    mEnd = rhs.mEnd;
    mKind = rhs.mKind;
    mGroup = rhs.mGroup;
    mObject = rhs.mObject;
    mAngle = rhs.mAngle;
}
#endif
