#pragma once
#include "common.h"
#include "UnitController.h"

class Unit final {
private:
    float mHP,mTime,mFOV, mRadius;
    mutable uniqueRAII<Scene> mModel;
    std::string mName;
    uniqueRAII<Properties> mInfo;
    Vector2 mPlane;
    Vector3 mOffset, mReleaseOffset;
    bool mCross;
    uint32_t mLoading;
public:
    void operator=(const std::string& name);
    std::string getName() const;
    Node* getModel() const;
    float getHP() const;
    float getTime() const;
    float getFOV() const;
    const Properties* getControlInfo() const;
    Vector2 getPlane() const;
    Vector3 getOffset() const;
    float getRadius() const;
    bool canCross() const;
    uint32_t getLoading() const;
    Vector3 getReleaseOffset() const;
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
    float mDelta;
    bool mIsServer;
    std::chrono::system_clock::time_point mLast;
    std::vector<std::pair<uint16_t,float>> mLoading;
    uint32_t mLoadTarget;
    void updateMoveTarget();
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
    bool updateSum(float delta);
    void setHP(float HP);
    float getHP() const;
    bool tryLoad(const UnitInstance& rhs);
    std::vector<std::pair<uint16_t, float>> release();
    void setLoadTarget(uint32_t id);
    uint32_t getLoadTarget() const;
    uint32_t getLoadSize() const;
    //Client
    UnitInstance(const Unit& unit, uint8_t group, uint32_t id, Scene* add, bool isServer,Vector3 pos);
    uint8_t getGroup() const;
};
