#include "Server.h"
#include "Client.h"
#include "Unit.h"
#include <iterator>
#include <future>

std::map<std::string, Unit> globalUnits;

void Unit::operator=(const std::string& name) {
    mName = name;
    INFO("Loading unit ", name);
    std::string full = "/res/units/" + name + "/";
    mInfo = Properties::create((full + "unit.info").c_str());
    mHP = mInfo->getFloat("HP");
    mTime = mInfo->getFloat("cost")*1000.0f;
    mFOV = mInfo->getFloat("FOV");
    mFOV *= mFOV;
    mInfo->getVector2("plane", &mPlane);
    mOffset = mInfo->getFloat("offset");
    mPlane *= 0.5f;
    mRadius = mInfo->getFloat("radius");
    mCross = mInfo->getBool("cross", false);
    mLoading = mInfo->getInt("loading");
    if (mInfo->exists("releaseOffset"))
        mInfo->getVector3("releaseOffset", &mReleaseOffset);
    mSound = mInfo->getFloat("sound");
    mType = mInfo->getString("type", "army");
    auto p = mInfo->getNamespace("control", true);
    mMoveArg.x = p->getFloat("v")/1000.0f;
    mMoveArg.y =std::min(p->getFloat("RSC"),static_cast<float>(M_PI/1000.0));
    mMoveArg.z = p->getFloat("rfac");
    if (mMoveArg.z == 0.0f)
        mMoveArg.z = 1.0f;
}

std::string Unit::getName() const {
    return mName;
}

Node* Unit::getModel() const {
    if (!mModel)
        mModel = Scene::load(("/res/units/" + mName + "/model.scene").c_str());
    return mModel->findNode("root")->clone();
}

float Unit::getHP() const {
    return mHP;
}

float Unit::getFOV() const {
    return mFOV;
}

const Properties * Unit::getControlInfo() const {
    return mInfo->getNamespace("control", true);
}

float Unit::getTime() const {
    return mTime;
}

Vector2 Unit::getPlane() const {
    return mPlane;
}

float Unit::getOffset() const {
    return mOffset;
}

float Unit::getRadius() const {
    return mRadius;
}

bool Unit::canCross() const {
    return mCross;
}

uint32_t Unit::getLoading() const {
    return mLoading;
}

Vector3 Unit::getReleaseOffset() const {
    return mReleaseOffset;
}

float Unit::getSound() const {
    return mSound;
}

std::string Unit::getType() const {
    return mType;
}

Vector3 Unit::getMoveArg() const {
    return mMoveArg;
}

uint32_t UnitInstance::cnt = 0;

Node * UnitInstance::getNode() const {
    return mNode.get();
}

void UnitInstance::setMoveTarget(Vector2 pos) {
    if (mKind->canCross())
        mController->setMoveTarget(pos);
    else {
        mTarget = pos;
        mController->setMoveTarget(updateMoveTarget());
    }
}

bool UnitInstance::update(float delta) {
    mPos = mNode->getTranslation();
    if (mPos.x < -mapSizeHF)mNode->setTranslationX(-mapSizeHF);
    if (mPos.x > mapSizeHF)mNode->setTranslationX(mapSizeHF);
    if (mPos.z < -mapSizeHF)mNode->setTranslationZ(-mapSizeHF);
    if (mPos.z > mapSizeHF)mNode->setTranslationZ(mapSizeHF);
    if (mIsServer && !mKind->canCross() && !isDied()) {
        mController->setMoveTarget(updateMoveTarget());
        if (mPos.y < -10.0f)
            mHP -= 1000.0f;
    }
    return mController->update(*this, delta);
}

BoundingSphere UnitInstance::getBound() const {
    return { mNode->getTranslation(),mKind->getRadius() };
}

bool test(Vector2 b, Vector2 e) {
    if (e.x<-mapSizeHF || e.x>mapSizeHF || e.y<-mapSizeHF || e.y>mapSizeHF
        || localClient->getHeight(e.x, e.y) < 0.0f)
        return false;
    constexpr auto num = 16;
    for (auto i = 1; i < num; ++i) {
        auto p = (b*i + e*(num - i)) / num;
        if (localClient->getHeight(p.x, p.y) < 0.0f)
            return false;
    }
    return true;
}

float choose(Vector2 b, Vector2 m, Vector2 unit, float maxv) {
    for (float i = 1.0f; i < maxv; ++i) {
        if (test(b, m + unit*i)) return i;
        if (test(b, m - unit*i)) return -i;
    }
    return maxv;
}

Vector2 UnitInstance::updateMoveTarget() {
    Vector2 mp = { mPos.x,mPos.z };
    if (mTarget.isZero() || mTarget.distanceSquared(mp) <= 16.0f)
        return mTarget = {};
    if (test(mp, mTarget) || mPos.y <= 0.0f) 
        return mTarget;
    auto offset = mTarget - mp;
    if (offset.x == 0.0f)offset.x =1.0f;
    if (offset.y == 0.0f)offset.y =1.0f;
    Vector3 tmp = { offset.x,0.0f,offset.y };
    tmp.cross(Vector3::unitY());
    Vector2 unit = { tmp.x,tmp.z };
    unit.normalize();
    auto dis = offset.length()*10.0f;
    float w = 0.5f;
    while (w >= 0.01f) {
        auto mid = mTarget*w + mp*(1.0f - w);
        auto p = choose(mp, mid, unit, dis);
        if (p<dis) return mid+unit*p;
        w*=0.618f;
    }

    return mTarget = {};
}

void UnitInstance::setAttackTarget(uint32_t id) {
    mController->setAttackTarget(id);
    mAttackPos = {};
}

bool UnitInstance::attacked(float harm) {
    setHP(mHP - harm);
    return mHP < 0.0f;
}

uint32_t UnitInstance::getID() const {
    return mPID;
}

const Unit & UnitInstance::getKind() const {
    return *mKind;
}

UnitInstance::UnitInstance(const Unit & unit, uint8_t group, uint32_t id,
    Scene* add, bool isServer, Vector3 pos)
    :mGroup(group), mHP(unit.getHP()), mNode(nullptr), mPID(id), mKind(&unit),
    mIsServer(isServer), mLoadTarget(0), mPos(pos) {
    mNode = unit.getModel();
    add->addNode(mNode.get());
    mNode->setTranslation(pos);
    mController = UnitController::newInstance(unit.getControlInfo());
    if (isServer)mController->isServer();
}

void loadAllUnits() {
    std::vector<std::string> paths;
    listDirs("/res/units", paths);
    for (auto p : paths)
        globalUnits[p] = p;
}

uint16_t getUnitID(const std::string & name) {
    return std::distance(globalUnits.begin(), globalUnits.find(name));
}

Unit & getUnit(uint16_t id) {
    auto i = globalUnits.begin();
    std::advance(i, id);
    return i->second;
}

uint8_t UnitInstance::getGroup() const {
    return mGroup;
}

uint32_t UnitInstance::askID() {
    return ++cnt;
}

uint32_t UnitInstance::getAttackTarget() const {
    return mController->getAttackTarget();
}

bool UnitInstance::isDied() const {
    return mHP <= 0.0f;
}

Vector3 UnitInstance::getRoughPos() const {
    return mPos;
}

void UnitInstance::setHP(float HP) {
    mHP = HP;
    if (isDied())mController->onDied(*this);
}

float UnitInstance::getHP() const {
    return mHP;
}

bool UnitInstance::tryLoad(const UnitInstance & rhs) {
    if (mLoading.size() >= mKind->getLoading() ||
        rhs.getKind().getLoading() || rhs.isDied())return false;
    mLoading.emplace_back(getUnitID(rhs.getKind().getName()), rhs.getHP());
    return true;
}

std::vector<std::pair<uint16_t, float>> UnitInstance::release() {
    std::vector<std::pair<uint16_t, float>> res;
    res.swap(mLoading);
    return res;
}

void UnitInstance::setLoadTarget(uint32_t id) {
    mLoadTarget = id;
}

uint32_t UnitInstance::getLoadTarget() const {
    return mLoadTarget;
}

uint32_t UnitInstance::getLoadSize() const {
    return mLoading.size();
}

bool UnitInstance::isStoped() const {
    return mController->isStoped();
}

void UnitInstance::move(Vector2 force) const {
    auto arg = mKind->getMoveArg();
    float fac = 1.0f;
    if (force.y != 0.0f) fac = arg.z, mNode->rotateY(force.y*arg.y);
    if (force.x != 0.0f) {
        auto b = mPos + mNode->getDownVector().normalize()*mKind->getOffset();
        if (b.y < localClient->getHeight(b.x, b.z)) {
            auto f = mNode->getForwardVector().normalize();
            auto base = -Vector3::unitY();
            auto fd = f.dot(base.normalize());
            if (force.x > 0)fd = 1.0f + fd;
            else fd = 1.0f - fd;
            fac *= fd;
        }
        mNode->translateForward(arg.x*fac*force.x);
    }
}

Vector2 UnitInstance::getAttackPos() const {
    return { mAttackPos.x,mAttackPos.z };
}

void UnitInstance::setAttackPos(Vector2 pos) {
    mAttackPos = { pos.x,localClient->getHeight(pos.x,pos.y),pos.y };
    mController->setAttackTarget(pointID);
}

Vector3 UnitInstance::getPos(uint32_t& id) const {
    return id==pointID?mAttackPos:localClient->getPos(id);
}

UnitInstance::UnitInstance() {
    throw;
}

#ifdef ANDROID
UnitInstance::UnitInstance(const UnitInstance& src) {
    auto&& rhs = const_cast<UnitInstance&>(src);
    mNode.swap(rhs.mNode);
    mHP = rhs.mHP;
    mGroup = rhs.mGroup;
    mPID = rhs.mPID;
    mKind = rhs.mKind;
    mPos = rhs.mPos;
    mAttackPos = rhs.mAttackPos;
    mTarget = rhs.mTarget;
    mController.swap(rhs.mController);
    mIsServer = rhs.mIsServer;
    mLoading.swap(rhs.mLoading);
    mLoadTarget = rhs.mLoadTarget;
}
#endif
