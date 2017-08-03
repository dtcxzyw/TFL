#pragma once
#include "common.h"
#include "UnitController.h"

class Unit final {
private:
    float mHP,mTime,mFOV, mRadius,mSound,mOffset;
    mutable uniqueRAII<Scene> mModel;
    std::string mName,mType;
    uniqueRAII<Properties> mInfo;
    Vector2 mPlane;
    Vector3 mReleaseOffset;
    bool mCross;
    uint32_t mLoading;
    Vector3 mMoveArg;
public:
    void operator=(const std::string& name);
    std::string getName() const;
    Node* getModel() const;
    float getHP() const;
    float getTime() const;
    float getFOV() const;
    const Properties* getControlInfo() const;
    Vector2 getPlane() const;
    float getOffset() const;
    float getRadius() const;
    bool canCross() const;
    uint32_t getLoading() const;
    Vector3 getReleaseOffset() const;
    float getSound() const;
    std::string getType() const;
    Vector3 getMoveArg() const;
};

extern std::map<std::string,Unit> globalUnits;

void loadAllUnits();
uint16_t getUnitID(const std::string& name);
Unit& getUnit(uint16_t id);

class UnitInstance final {
private:
    static uint32_t cnt;
    uniqueRAII<Node> mNode;
    float mHP;
    uint8_t mGroup;
    uint32_t mPID;
    const Unit* mKind;
    Vector3 mPos;
    Vector2 mTarget;
    std::unique_ptr<UnitController> mController;
    bool mIsServer;
    std::vector<std::pair<uint16_t,float>> mLoading;
    uint32_t mLoadTarget;
    Vector2 updateMoveTarget();
public:
    UnitInstance() {
        throw;
    }
#ifdef ANDROID
    UnitInstance(const UnitInstance& rhs)
    {
        auto& r = const_cast<UnitInstance&>(rhs);
        mNode.swap(r.mNode);
        mController.swap(r.mController);
        mHP = rhs.mHP;
        mGroup = rhs.mGroup;
        mKind = rhs.mKind;
        mPID = rhs.mPID;
    }
#endif
    void setAttackTarget(uint32_t id);
    bool update(float delta);
    BoundingSphere getBound() const;
    //Server
    Node* getNode() const;
    bool attacked(float harm);
    uint32_t getID() const;
    const Unit& getKind() const;
    void setMoveTarget(Vector2 pos);
    static uint32_t askID();
    uint32_t getAttackTarget() const;
    bool isDied() const;
    Vector3 getRoughPos() const;
    void setHP(float HP);
    float getHP() const;
    bool tryLoad(const UnitInstance& rhs);
    std::vector<std::pair<uint16_t, float>> release();
    void setLoadTarget(uint32_t id);
    uint32_t getLoadTarget() const;
    uint32_t getLoadSize() const;
    bool isStoped() const;
    void move(Vector2 force) const;
    //Client
    UnitInstance(const Unit& unit, uint8_t group, uint32_t id, Scene* add, bool isServer,Vector3 pos);
    uint8_t getGroup() const;
};
