#pragma once
#include "common.h"
#include "Bullet.h"

class UnitInstance;

class UnitController {
public:
    static void initAllController();
    static std::unique_ptr<UnitController>  newInstance(const Properties* info);
    void setMoveTarget(Vector2 dest);
    void setAttackTarget(uint32_t id);
    uint32_t getAttackTarget() const;
    void isServer();
    virtual bool update(UnitInstance& instance,float delta) = 0;
protected:
    uint32_t mObject=0;
    Vector2 mDest=Vector2::zero();
    bool mIsServer=false;
};
