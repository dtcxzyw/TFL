#include "BuiltinAI.h"
#include "common.h"
#include <map>
#include <vector>
#include <functional>
#include <Message.h>
#include <RakPeer.h>
#include <thread>
using namespace std::literals;

class BuiltinAI final {
private:
    enum class Type {
        attack, defense, discover
    };
    std::map<std::string, Type> mUnits;
    using SendFunc = std::function<void(const RakNet::BitStream&, PacketPriority, PacketReliability)>;
    SendFunc mSend;
    std::vector<Vector2> mKeyPoint;
    std::map<uint32_t, UnitSyncInfo> mMine;
    std::map<uint32_t, UnitSyncInfo> mArmies;
    uint32_t mGroup;
    Type mStep = Type::discover;
    struct TeamInfo final {
        std::vector<uint32_t> current;
        Vector2 object;
    };
public:
    BuiltinAI() {
        std::vector<std::string> units;
        listDirs("res/units", units);
        for (auto&& x : units) {
            uniqueRAII<Properties> info = Properties::create(("res/units/" + x + "/unit.info").c_str());
            std::string tname = info->getString("AIType", "discover");
            if (tname != "load") {
                if (tname == "attack")mUnits[x] = Type::attack;
                else if (tname == "defense")mUnits[x] = Type::defense;
                else mUnits[x] = Type::discover;
            }
        }
    }

    void connect(const SendFunc& send, const std::string& map, uint8_t group) {
        mSend = send;
        uniqueRAII<Properties> info = Properties::create(("res/maps/" + map + "/map.info").c_str());
        const char* id;
        Vector2 tmp;
        while ((id = info->getNextProperty()) && info->getVector2(id, &tmp))
            mKeyPoint.emplace_back(tmp / 512.0f* mapSizeF - Vector2{ mapSizeHF, mapSizeHF });
        mGroup = group;
    }

    void updateUnit(const UnitSyncInfo& info) {
        if (info.group == mGroup)
            mMine[info.id] = info;
        else
            mArmies[info.id] = info;
    }

    void newUnit(const UnitSyncInfo& info) {
        updateUnit(info);
    }

    void deleteUnit(uint32_t id) {
        if (mMine.find(id) != mMine.cend())
            mMine.erase(id);
        else
            mArmies.erase(id);
    }

    void send(const TeamInfo& info) {
        RakNet::BitStream data;
        data.Write(ClientMessage::setMoveTarget);
        data.Write(info.object);
        data.Write(static_cast<uint32_t>(info.current.size()));
        for (auto&& x : info.current)
            data.Write(x);
        mSend(data, PacketPriority::IMMEDIATE_PRIORITY, PacketReliability::RELIABLE_ORDERED);
    }

    void send(uint16_t id, uint16_t weight) {
        RakNet::BitStream data;
        data.Write(ClientMessage::changeWeight);
        data.Write(id);
        data.Write(weight);
        mSend(data, PacketPriority::IMMEDIATE_PRIORITY, PacketReliability::RELIABLE_ORDERED);
    }

    void update() {

        if (mArmies.size() > mMine.size())mStep = Type::defense;
        else if ((mArmies.size() && mArmies.size() < mMine.size())) mStep = Type::attack;
        else mStep = Type::discover;

        {
            uint16_t id = 0;
            for (auto&& x : mUnits) {
                if (x.second == mStep)
                    send(id, 1000);
                else
                    send(id, 1);
                ++id;
            }
        }

        std::vector<TeamInfo> teams;

        for (auto&& k : mKeyPoint) {
            TeamInfo team;
            team.object = k;
            teams.emplace_back(team);
        }

        std::vector<uint32_t> free;

        for (auto&& x : mMine)
            free.emplace_back(x.first);

        while (free.size())
            for (auto&& t : teams) {
                if (free.empty())break;
                auto x = std::min_element(free.cbegin(), free.cend(), [&](auto&& a, auto&& b) {
                    return t.object.distanceSquared({mMine[a].pos.x,mMine[a].pos.z }) <
                        t.object.distanceSquared({ mMine[b].pos.x,mMine[b].pos.z });
                });
                t.current.emplace_back(*x);
                free.erase(x);
            }

        for (auto&& t : teams)
            send(t);
    }
};

#define FORNET for (auto packet = peer.Receive(); packet; peer.DeallocatePacket(packet), packet = peer.Receive())

void AIMain(bool* flag,uint8_t level) {
    constexpr uint8_t group = 5;
    BuiltinAI builtinAI;
    RakNet::RakPeer peer;
    RakNet::SocketDescriptor SD;
    peer.Startup(1, &SD, 1);
    std::string str;
    RakNet::SystemAddress server("127.0.0.1", 23333);
    auto res = peer.Connect("127.0.0.1", 23333, nullptr, 0);
    if (res == RakNet::CONNECTION_ATTEMPT_STARTED) {
        while (peer.NumberOfConnections() != 1)
            std::this_thread::sleep_for(1ms);
    }
    else INFO("BuiltinAI can't connect to local server");

    {
        RakNet::BitStream data;
        data.Write(ClientMessage::changeGroup);
        data.Write(group);
        peer.Send(&data, PacketPriority::IMMEDIATE_PRIORITY,
            PacketReliability::RELIABLE_ORDERED, 0, server, false);
    }

    while (true) {
        FORNET{
            CheckBegin;
        CheckHeader(ServerMessage::info) {
            RakNet::BitStream data(packet->data, packet->length,false);
            data.IgnoreBytes(1 + sizeof(uint64_t));
            RakNet::RakString str;
            data.Read(str);
            builtinAI.connect([&](const RakNet::BitStream& data, PacketPriority pp, PacketReliability pr) {
                peer.Send(&data, pp, pr, 0, server, false);
            },str.C_String(),5);
            *flag = true;
        }
        CheckHeader(ServerMessage::go) {
            peer.DeallocatePacket(packet);
            goto p2;
        }
        }
        std::this_thread::sleep_for(1ms);
    }
p2:

    bool isStop = false;
    std::set<uint32_t> old;

    while (true) {
        std::vector<unsigned char> latestData;
        FORNET{
            if (isStop)continue;
        RakNet::BitStream data(packet->data, packet->length, false);
        data.IgnoreBytes(1);
        CheckBegin;
        CheckHeader(ServerMessage::out) {
            isStop = true;
            continue;
        }
        CheckHeader(ServerMessage::stop) {
            isStop = true;
            continue;
        }
        CheckHeader(ServerMessage::win) {
            isStop = true;
            continue;
        }
        CheckHeader(ServerMessage::updateUnit) {
            latestData = std::vector<unsigned char>(packet->data, packet->data + packet->length);
        }
        }

            if (isStop ||
                peer.GetConnectionState(server) != RakNet::ConnectionState::IS_CONNECTED)break;
        if (latestData.empty())continue;

        uint32_t mine = 0, armies = 0;
        std::set<uint32_t> copy = old;

        RakNet::BitStream latest(latestData.data(), latestData.size(), false);
        latest.IgnoreBytes(1);
        uint32_t size;
        latest.Read(size);
        for (uint32_t i = 0; i < size; ++i) {
            UnitSyncInfo u;
            latest.Read(u);
            if (u.isDied)continue;
            auto iter = copy.find(u.id);
            if (iter == copy.cend()) {
                old.insert(u.id);
                builtinAI.newUnit(u);
            }
            else {
                copy.erase(iter);
                builtinAI.updateUnit(u);
            }
            if (u.group == group)
                ++mine;
            else
                ++armies;
        }

        for (auto&& x : copy)
            builtinAI.deleteUnit(x);

        builtinAI.update();

        std::this_thread::sleep_for(50ms*(10-level));
    }
    peer.Shutdown(500, 0, IMMEDIATE_PRIORITY);
}

#undef FORNET

std::unique_ptr<std::future<void>> aiFuture;
