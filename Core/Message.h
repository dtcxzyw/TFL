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
    //
    end
};

enum class ServerMessage : unsigned char {
    begin = static_cast<unsigned char>(ClientMessage::end),
    info,
    go,
    updateUnit,
    updateWeight,
    updateBullet,
    win,
    out,
    stop,
    end
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

extern uint64_t pakKey;

