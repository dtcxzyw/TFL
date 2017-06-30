#include "Server.h"
#include "Client.h"
#include "Unit.h"
#include <iterator>

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

bool UnitInstance::update(float delta) {
    auto p = mNode->getTranslation();
    if (localClient->getPos(mController->getAttackTarget()).
        distanceSquared({ p.x,p.z }) > mKind->getFOV())
        mController->setAttackTarget(0);
    if (p.x < -5000.0f)mNode->setTranslationX(-5000.0f);
    if (p.x > 5000.0f)mNode->setTranslationX(5000.0f);
    if (p.z < -5000.0f)mNode->setTranslationZ(-5000.0f);
    if (p.z > 5000.0f)mNode->setTranslationZ(5000.0f);
    return mController->update(*this, delta);
}

BoundingSphere UnitInstance::getBound() const {
    return {mNode->getTranslation(),mKind->getRadius()};
}

void UnitInstance::setAttackTarget(uint32_t id) {
    mController->setAttackTarget(id);
}

bool UnitInstance::attacked(float harm) {
    return (mHP -= harm) <= 0.0f;
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

bool UnitInstance::isDied() const {
    return mHP<=0.0f;
}
