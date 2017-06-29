#include "Server.h"
#include "Client.h"
#include "Unit.h"
#include <iterator>
#include <array>

std::map<std::string, Unit> globalUnits;

void Unit::operator=(const std::string& name) {
    mName = name;
    INFO("Load unit ", name);
    std::string full = "/res/units/" + name + "/";
    mModel =Scene::load((full + "model.scene").c_str());
    mInfo = Properties::create((full + "unit.info").c_str());
    mHP = mInfo->getFloat("HP");
    mTime = mInfo->getFloat("cost");
    mFOV = mInfo->getFloat("FOV");
    mFOV *= mFOV;
    mInfo->getVector2("plane", &mPlane);
    mInfo->getVector3("offset", &mOffset);
    mPlane *= 0.5f;
    mRadius = mInfo->getFloat("radius");
}

std::string Unit::getName() const {
    return mName;
}

Node* Unit::getModel() const {
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


uint32_t UnitInstance::cnt = 0;

Node * UnitInstance::getNode() {
    return mNode.get();
}

void UnitInstance::setMoveTarget(Vector2 pos) {
    mController->setMoveTarget(pos);
}

void UnitInstance::update(float delta) {
    auto p = mNode->getTranslation();
    if (localClient->getPos(mController->getAttackTarget()).
        distanceSquared({ p.x,p.z }) > mKind->getFOV())
        mController->setAttackTarget(0);
    if (mController->update(mNode.get(), delta)) {
        //correct
        auto p = mNode->getTranslation();
        auto b = p + mKind->getOffset();
        auto h = localClient->getHeight(b.x, b.z);
        mNode->setTranslationY(h - mKind->getOffset().y);
        auto d = mKind->getPlane();

        std::array<Vector2, 4> base =
        { Vector2{b.x + d.x,b.z + d.y},
            Vector2{ b.x + d.x,b.z - d.y } ,
            Vector2{ b.x - d.x,b.z + d.y } ,
            Vector2{ b.x - d.x,b.z - d.y } };
        std::array<Vector3, 4> sample;
        size_t idx = 0;
        for (auto&&x : base) {
            sample[idx] = Vector3{ x.x,localClient->getHeight(x.x,x.y),x.y };
            ++idx;
        }

        Vector3 mean;
        for (size_t i = 0; i < 4; ++i) {
            auto a = sample[(i + 3) % 4] - sample[i];
            a.normalize();
            auto b = sample[(i + 1) % 4] - sample[i];
            b.normalize();
            Vector3 up;
            Vector3::cross(a, b, &up);
            if (up.y < 0)up.negate();
            up.normalize();
            mean += up / 4.0f;
        }

        mean.normalize();
        auto dot = [&, this] {
            auto u = mNode->getUpVectorWorld();
            u.normalize();
            return u.dot(mean);
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
#undef TEST
    }
}

BoundingSphere UnitInstance::getBound() const {
    return {mNode->getTranslation(),mKind->getRadius()};
}

void UnitInstance::setAttackTarget(uint32_t id) {
    mController->setAttackTarget(id);
}

bool UnitInstance::attacked(float harm) {
    return (mHP -= harm) < 0.0f;
}

uint32_t UnitInstance::getID() const {
    return mPID;
}

const Unit & UnitInstance::getKind() const {
    return *mKind;
}

UnitInstance::UnitInstance(const Unit & unit, uint8_t group, uint32_t id,
    Scene* add, bool isServer, Vector3 pos)
    :mGroup(group), mHP(unit.getHP()), mNode(nullptr), mPID(id), mKind(&unit) {
    mNode = unit.getModel();
    add->addNode(mNode.get());
    mNode->setTranslation(pos);
    mController = UnitController::newInstance(unit.getControlInfo());
    if (isServer)mController->isServer();
}

void loadAll() {
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

uint16_t UnitInstance::getGroup() const {
    return mGroup;
}

uint32_t UnitInstance::askID(){
    return ++cnt;
}

uint32_t UnitInstance::getAttackTarget() const {
    return mController->getAttackTarget();
}
