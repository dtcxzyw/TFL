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
    float mValue;
    Type mStep = Type::defense;
    struct TeamInfo final {
        std::set<uint32_t> current;
        uint32_t size;
        Vector2 object;
    };
    std::vector<TeamInfo> mTeams;
    std::vector<uint32_t> mFree;
    Vector2 mRoot;
public:
    BuiltinAI() :mValue(0.0f) {
        std::vector<std::string> units;
        listDirs("res/units", units);
        for (auto&& x : units) {
            uniqueRAII<Properties> info = Properties::create(("res/units/" + x + "/unit.info").c_str());
            auto tname = info->getString("AIType", "discover");
            if (tname == "attack")mUnits[x] = Type::attack;
            else if (tname == "defense")mUnits[x] = Type::defense;
            else mUnits[x] = Type::discover;
        }
    }

    void connect(const SendFunc& send, const std::string& map, uint8_t group) {
        mSend = send;
        uniqueRAII<Properties> info = Properties::create(("res/maps/" + map + "/map.info").c_str());
        const char* id;
        Vector2 tmp;
        while ((id = info->getNextProperty()) && info->getVector2(id, &tmp))
            mKeyPoint.emplace_back(tmp / 512.0f* 3000.0f - Vector2{ 1500.0f, 1500.0f });
        mGroup = group;
    }

    void setRoot(const Vector2 root) {
        mRoot = root;
    }

    void updateUnit(const UnitSyncInfo& info) {
        if (info.group == mGroup)
            mMine[info.id] = info;
        else
            mArmies[info.id] = info;
    }

    void newUnit(const UnitSyncInfo& info) {
        updateUnit(info);
        mValue += (info.group == mGroup) ? 0.5f : -1.0f;
        if (info.group == mGroup)
            mFree.emplace_back(info.id);
    }

    void deleteUnit(uint32_t id) {
        if (mMine.find(id) != mMine.cend())
            mMine.erase(id), --mValue;
        else
            mArmies.erase(id), mValue += 0.5f;
    }

    void send(const TeamInfo& info) {
        RakNet::BitStream data;
        data.Write(ClientMessage::setMoveTarget);
        data.Write(info.object);
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

    void clearTeams() {
        for (auto&& x : mTeams)
            for (auto&& u : x.current)
                if (mMine.find(u) != mMine.cend())
                    mFree.emplace_back(u);
        mTeams.clear();
    }

    void update() {

        auto old = mStep;
        //discover
        if (mValue > -20.0f || mArmies.empty())mStep = Type::discover;
        //attack
        if (mValue > 100.0f || (mArmies.size() &&
            mArmies.size() * 5 < mMine.size())) mStep = Type::attack;
        //defense
        if (mValue<-10.0f || mArmies.size()>0.2*mMine.size())mStep = Type::defense;

        if (old != mStep) {
            uint16_t id = 0;
            for (auto&& x : mUnits) {
                if (x.second == mStep)
                    send(id, 1000);
                ++id;
            }
            clearTeams();

            switch (mStep) {
            case Type::attack:
            {
            }
            break;
            case Type::defense:
            {
                TeamInfo team;
                team.object = mRoot;
                team.size = std::numeric_limits<uint32_t>::max();
                send(team);
                mTeams.emplace_back(team);
                uint16_t id = 0;
                for (auto&& x : mUnits) {
                    if (x.second != Type::defense)
                        send(id, 1);
                    ++id;
                }
            }
            break;
            case Type::discover:
            {
                for (auto&& x : mKeyPoint) {
                    TeamInfo team;
                    team.size = mFree.size() / mKeyPoint.size() / 3 + 3;
                    team.object = x;
                    mTeams.emplace_back(team);
                }
            }
            break;
            default:
                break;
            }
        }

        if (mStep == Type::attack) {
            clearTeams();

            std::map<Vector2, uint32_t> find;
            for (auto&& x : mArmies) {
                auto&& info = x.second;
                if (info.group != mGroup) {
                    Vector2 p{ info.pos.x,info.pos.z };
                    bool flag = true;
                    for (auto&& x : find) {
                        if (x.first.distanceSquared(p) < 90000.0f) {
                            ++x.second, flag = false;
                            break;
                        }
                    }
                    if (flag)find[p] = 1;
                }
            }

            int32_t size = mFree.size();
            for (auto&& x : find) {
                TeamInfo team;
                team.size = x.second*1.5;
                size -= team.size;
                team.object = x.first;
                mTeams.emplace_back(team);
            }

            size = std::max(size, 0);

            for (auto&& x : mKeyPoint) {
                TeamInfo team;
                team.size = 50 + size / mKeyPoint.size();
                team.object = x;
                mTeams.emplace_back(team);
            }
        }

        std::remove_if(mFree.begin(), mFree.end(),
            [this](uint32_t id) {return mMine.find(id) == mMine.cend(); });

        if (mStep == Type::discover && mFree.size() > 5) {
            TeamInfo team;
            team.size = 3;
            team.object = { mt() % 3000 - 1500.0f,mt() % 3000 -1500.0f };
            mTeams.emplace_back(team);
        }

        for (auto&& x : mTeams) {
            if (mFree.empty())continue;
            std::set<uint32_t> deferred;
            for (auto&& id : x.current)
                if (mMine.find(id) == mMine.cend())
                    deferred.insert(id);
            for (auto&& y : deferred)
                x.current.erase(y);
            uint32_t size = std::min(mFree.size(), x.size - x.current.size());
            std::partial_sort(mFree.begin(), mFree.begin() + size, mFree.end(),
                [this, &x](uint32_t a, uint32_t b) {
                return x.object.distanceSquared({ mMine[a].pos.x,mMine[a].pos.z }) >
                    x.object.distanceSquared({ mMine[b].pos.x,mMine[b].pos.z });
            });
            for (auto i = 0; i < size; ++i)
                x.current.insert(*(mFree.rbegin() + i));
            mFree.resize(mFree.size() - size);
            send(x);
        }

        std::map<uint32_t, uint32_t> attackMap;
        for (auto&& x : mMine) {
            float md = std::numeric_limits<float>::max();
            uint32_t maxwell = 0;
            for (auto&& y : mArmies) {
                auto dis = y.second.pos.distanceSquared(x.second.pos);
                if (dis < md)md = dis, maxwell = y.first;
            }
            if (maxwell) attackMap[x.first] = maxwell;
        }
        if (attackMap.size()) {
            RakNet::BitStream data;
            data.Write(ClientMessage::setAttackTarget);
            for (auto&& x : attackMap) {
                data.Write(x.first);
                data.Write(x.second);
            }
            mSend(data, PacketPriority::HIGH_PRIORITY, PacketReliability::RELIABLE);
        }
    }
};

#define FORNET for (auto packet = peer.Receive(); packet; peer.DeallocatePacket(packet), packet = peer.Receive())

void AIMain(bool* flag) {
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
            RakNet::BitStream data(packet->data, packet->length,false);
            data.IgnoreBytes(1);
            Vector2 p;
            data.Read(p);
            builtinAI.setRoot(p);
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

        std::this_thread::sleep_for(1s);
    }
    peer.Shutdown(500, 0, IMMEDIATE_PRIORITY);
}

#undef FORNET

std::unique_ptr<std::future<void>> aiFuture;
