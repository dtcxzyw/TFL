#include "Server.h"
#include "Client.h"
#include "Unit.h"
#include <iterator>

std::map<std::string, Unit> globalUnits;

void Unit::operator=(const std::string& name) {
    mName = name;
    INFO("Load unit ", name);
    std::string full = "/res/units/" + name + "/";
    mInfo = Properties::create((full + "unit.info").c_str());
    mHP = mInfo->getFloat("HP");
    mTime = mInfo->getFloat("cost")*1000.0f;
    mFOV = mInfo->getFloat("FOV");
    mFOV *= mFOV;
    mInfo->getVector2("plane", &mPlane);
    mInfo->getVector3("offset", &mOffset);
    mPlane *= 0.5f;
    mRadius = mInfo->getFloat("radius");
    mCross = mInfo->getBool("cross", false);
    mLoading = mInfo->getInt("loading");
    if(mInfo->exists("releaseOffset"))
        mInfo->getVector3("releaseOffset", &mReleaseOffset);
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

Vector3 Unit::getOffset() const {
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

uint32_t UnitInstance::cnt = 0;

Node * UnitInstance::getNode() const {
    return mNode.get();
}

void UnitInstance::setMoveTarget(Vector2 pos) {
    if (mKind->canCross())
        mController->setMoveTarget(pos);
    else {
        mTarget = pos;
        updateMoveTarget();
    }
}

bool UnitInstance::update(float delta) {
    mPos = mNode->getTranslation();
    if (mPos.x < -mapSizeHF)mNode->setTranslationX(-mapSizeHF);
    if (mPos.x > mapSizeHF)mNode->setTranslationX(mapSizeHF);
    if (mPos.z < -mapSizeHF)mNode->setTranslationZ(-mapSizeHF);
    if (mPos.z > mapSizeHF)mNode->setTranslationZ(mapSizeHF);
    if (mIsServer && !mKind->canCross() && !isDied()) {
        auto now = std::chrono::system_clock::now();
        if (now - mLast > 500ms)
            updateMoveTarget(), mLast = now;
        if (mPos.y < -10.0f)
            mHP -= 1000.0f;
    }
    delta += mDelta;
    mDelta = 0.0f;
    return mController->update(*this, delta);
}

BoundingSphere UnitInstance::getBound() const {
    return {mNode->getTranslation(),mKind->getRadius()};
}

bool test(Vector2 b, Vector2 e) {
    if (e.x<-mapSizeHF || e.x>mapSizeHF || e.y<-mapSizeHF || e.y>mapSizeHF
        || localClient->getHeight(e.x, e.y) < -2.0f)
        return false;
    auto dis = b.distance(e);
    constexpr auto step = 40.0f;
    if (dis <= step)return true;
    for (auto i = step; i < dis; i += step) {
        auto p = (b*i + e*(dis - i))/dis;
        if (localClient->getHeight(p.x, p.y) < -2.0f)
            return false;
    }
    return true;
}

Vector2 choose(Vector2 b, Vector2 m,Vector2 unit, float maxv) {
    float l = 1.0f, r = maxv;
    while (r - l >= 1.0f) {
        float mid = (l + r) / 2.0f;
        auto p = m + unit*mid;
        if (test(b, p))r = mid;
        else l = mid;
    }
    return m + unit*r;
}

void UnitInstance::updateMoveTarget() {
    Vector2 mp = { mPos.x,mPos.z };
    if (mTarget.isZero() || mTarget.distanceSquared(mp) <= 16.0f) {
        mTarget = {};
        mController->setMoveTarget({});
        return;
    }
    if (test(mp, mTarget) || mPos.y<=0.0f) {
        mController->setMoveTarget(mTarget);
        return;
    }
    auto offset = mp - mTarget;
    if (offset.x == 0.0f)offset.x = std::numeric_limits<float>::epsilon();
    if (offset.y == 0.0f)offset.y = std::numeric_limits<float>::epsilon();
    constexpr auto maxFac =128.0f;
    auto fac = 1.0f;
    while (fac <= maxFac) {
        float w = 0.5f;
        while (w >= 0.05f) {
            auto mid = mTarget*w + mp*(1.0f - w);
            Vector2 unit = { 1.0f,-offset.x / offset.y };
            unit.normalize();
            auto dis = offset.length()*fac;
            auto p1 = choose(mp, mid, unit, dis);
            auto p2 = choose(mp, mid, -unit, dis);
            if (std::min(p1.distanceSquared(mid), p2.distanceSquared(mid)) <= dis*dis - 1.0f) {
                mController->setMoveTarget(
                    p1.distanceSquared(mid) < p2.distanceSquared(mid) ? p1 : p2);
                return;
            }
            w *= 0.5f;
        }
        fac *= 2.0f;
    }

    if (fac> maxFac) {
        mTarget = {};
        mController->setMoveTarget({});
    }
}

void UnitInstance::setAttackTarget(uint32_t id) {
    mController->setAttackTarget(id);
}

bool UnitInstance::attacked(float harm) {
    mHP -= harm;
    if (mHP < 0.0f)mController->onDied(*this);
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
    mDelta(0.0f),mIsServer(isServer),mLoadTarget(0) {
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

uint32_t UnitInstance::askID(){
    return ++cnt;
}

uint32_t UnitInstance::getAttackTarget() const {
    return mController->getAttackTarget();
}

bool UnitInstance::isDied() const {
    return mHP<=0.0f;
}

Vector3 UnitInstance::getRoughPos() const{
    return mPos;
}

bool UnitInstance::updateSum(float delta) {
    mDelta += delta;
    if (mDelta > 20.0f) return update(0.0f);
    return false;
}

void UnitInstance::setHP(float HP) {
    mHP = HP;
}

float UnitInstance::getHP() const {
    return mHP;
}

bool UnitInstance::tryLoad(const UnitInstance & rhs) {
    if (mLoading.size() >= mKind->getLoading() ||
        rhs.getKind().getLoading() || rhs.isDied())return false;
    mLoading.emplace_back(getUnitID(rhs.getKind().getName()),rhs.getHP());
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
