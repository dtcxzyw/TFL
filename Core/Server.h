#pragma once
#include "Map.h"
#include "Unit.h"
#include <RakPeer.h>
#include <string>


struct ClientInfo final {
    uint8_t group=1;
};

struct GroupInfo final {
    std::vector<uint16_t> weight;
    std::map<uint32_t,UnitInstance> units;
    std::vector<uint8_t> key;
    GroupInfo();
};

struct KeyInfo final {
    const Vector2 pos;
    static constexpr auto nil = std::numeric_limits<uint8_t>::max();
    uint8_t owner;
    static constexpr auto none = std::numeric_limits<uint16_t>::max();
    uint16_t id;
    uint32_t time=0;
    KeyInfo(Vector2 p);
};

class Server final {
private:
    RakNet::RakPeerInterface* mPeer;
    std::map<RakNet::SystemAddress, ClientInfo> mClients;
    std::map<uint8_t, GroupInfo> mGroups;
    std::map<uint32_t, BulletInstance> mBullets;
    bool mState;
    std::vector<KeyInfo> mKey;
    Map mMap;
    std::string mMapName;
    uniqueRAII<Scene> mScene;
    struct DiedInfo {
        uint8_t group;
        uint32_t id;
        double time;
    };
    std::vector<DiedInfo> mDeferred;
    void send(uint8_t group,const RakNet::BitStream& data,PacketPriority priority);
    void chooseNew(KeyInfo& k);
public:
    Server(const std::string& path);
    ~Server();

    //void load(const std::string& save);
    //void save(const std::string& save);

    void waitClient();
    const std::map<RakNet::SystemAddress, ClientInfo>& getClientInfo();
    void run();
    void update(float delta);
    void stop();
    std::string getIP();
    void attack(uint32_t id, float harm);
    void newBullet(BulletInstance&& bullet);
    Vector3 getUnitPos(uint32_t id) const;
};

extern std::unique_ptr<Server> localServer;
