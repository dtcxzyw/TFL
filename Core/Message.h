#pragma once
#include <MessageIdentifiers.h>

enum class ClientMessage : unsigned char {
    begin = ID_USER_PACKET_ENUM,
    changeGroup,
    exit,
    //code
    changeWeight,
    setAttackTarget,
    setMoveTarget,
    moveUnit,
    load,
    release
};

enum class ServerMessage : unsigned char {
    begin = ID_USER_PACKET_ENUM,
    info,
    go,
    updateUnit,
    updateState,
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
    Vector2 atp;
    uint32_t size;
    float HP;
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

struct ProducingSyncInfo final {
    uint8_t key;
    uint16_t kind;
    float time;
};

extern uint64_t pakKey;

constexpr auto pointID = std::numeric_limits<uint32_t>::max();
constexpr auto typeOffset = std::numeric_limits<uint32_t>::max() >> 1;
//[1,offset]->unit (offset,maxuint)->bullet maxunit->point

