#pragma once
#include <MessageIdentifiers.h>

enum class ClientMessage : unsigned char {
    begin = ID_USER_PACKET_ENUM,
    changeGroup,
    exit,
    //code
    changeWeight,
    setAttackTarget,
    setMoveTarget
};

enum class ServerMessage : unsigned char {
    begin = ID_USER_PACKET_ENUM,
    info,
    go,
    updateUnit,
    updateWeight,
    updateBullet,
	changeSpeed,
    duang,
    win,
    out,
    stop
};

#define CheckBegin if(false)
#define CheckHeader(message) else if(packet->data[0] ==static_cast<unsigned char>(message))

struct UnitSyncInfo final {
    uint32_t id;
    uint16_t kind;
    Vector3 pos;
    Quaternion rotation;
    uint8_t group;
    uint32_t at;
    bool isDied;
};

struct BulletSyncInfo final {
    uint32_t id;
    uint16_t kind;
    Vector3 pos;
    Quaternion rotation;
};

struct DuangSyncInfo final {
    uint16_t kind;
    Vector3 pos;
};

extern uint64_t pakKey;

