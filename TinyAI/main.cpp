//Pause maintenance
#include <iostream>
#include <fstream>
#include <string>
#include <RakPeer.h>
#include <set>
#include <thread>
#include <map>
#include <vector>
#include <random>
#include <algorithm>
#undef REGISTERED
#undef max
#undef min
#include "../Core/common.cpp"
using namespace gameplay;
#include "../Core/Message.h"
using namespace std::literals;

class AI final {
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
    AI() :mValue(0.0f) {
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
            mKeyPoint.emplace_back(tmp / 512.0f* 10000.0f - Vector2{ 5000.0f, 5000.0f });
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
        if(info.group==mGroup)
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

    void clearTeams(){
        for (auto&& x : mTeams)
            for (auto&& u : x.current) 
                if (mMine.find(u) != mMine.cend())
                    mFree.emplace_back(u);
        mTeams.clear();
    }

    void update() {

        auto old = mStep;
        //discover
        if (mValue > -5.0f || mArmies.empty())mStep = Type::discover;
        //attack
        if (mValue > 20.0f || (mArmies.size() && 
            mArmies.size() * 5 < mMine.size())) mStep = Type::attack;
        //defense
        if (mValue<-10.0f || mArmies.size()>0.2*mMine.size())mStep = Type::defense;

        if (old != mStep) {
            std::cout << "Switch state : ";
            switch (mStep) {
            case Type::attack:std::cout << "attack";
                break;
            case Type::defense:std::cout << "defense";
                break;
            case Type::discover:std::cout << "discover";
                break;
            default:
                break;
            }
            std::cout << std::endl;
            uint16_t id=0;
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
                    team.size = mFree.size()/mKeyPoint.size()/3+2;
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
                team.size = 20+size/mKeyPoint.size();
                team.object = x;
                mTeams.emplace_back(team);
            }
        }

        mFree.erase(std::remove_if(mFree.begin(), mFree.end(),
            [this](uint32_t id) {return mMine.find(id) == mMine.cend(); }),mFree.end());

        if (mStep == Type::discover && mFree.size() > 5) {
            TeamInfo team;
            team.size = 3;
            team.object = { mt() % 10000 - 5000.0f,mt() % 10000 - 5000.0f };
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
                [this,&x](uint32_t a,uint32_t b) {
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
            uint32_t maxwell=0;
            for (auto&& y : mArmies) {
                auto dis = y.second.pos.distanceSquared(x.second.pos);
                if (dis < md)md = dis, maxwell = y.first;
            }
            if (maxwell) attackMap[x.first] = maxwell;
        }
        if (attackMap.size()) {
            RakNet::BitStream data;
            data.Write(ClientMessage::setAttackTarget);
            for (auto&& x : attackMap){
                data.Write(x.first);
                data.Write(x.second);
            }
            mSend(data, PacketPriority::HIGH_PRIORITY, PacketReliability::RELIABLE);
        }

    }
} tinyAI;

#define FORNET for (auto packet = peer.Receive(); packet; peer.DeallocatePacket(packet), packet = peer.Receive())

int main() {
    RakNet::RakPeer peer;
    RakNet::SocketDescriptor SD;
    peer.Startup(1, &SD, 1);
    std::cout << "Input group:" << std::endl;
    uint16_t group;
    std::cin >> group;
    std::cout << "Scanning servers..." << std::endl;
    std::string str;
    std::set<RakNet::SystemAddress> ban;
    RakNet::SystemAddress server;
    while (true) {
        peer.Ping("255.255.255.255", 23333, false);
        FORNET{
            if (packet->data[0] == ID_UNCONNECTED_PONG &&
            ban.find(packet->systemAddress) == ban.cend()) {
                std::cout << "Find server " << packet->systemAddress.ToString() << std::endl;
                std::cout << "Shall we connect it?(y/n)" << std::endl;
                char c;
                std::cin >> c;
                if (c == 'y') {
                    server = packet->systemAddress;
                    auto res = peer.Connect(server.ToString(false), 23333, nullptr,0);
                    if (res == RakNet::CONNECTION_ATTEMPT_STARTED) {
                        while (peer.NumberOfConnections() != 1)
                            std::this_thread::sleep_for(1ms);
                        std::cout << "OK!" << std::endl;
                        peer.DeallocatePacket(packet);
                        goto p1;
                    }
                }
                else ban.insert(packet->systemAddress);
            }
        }
        std::this_thread::sleep_for(1ms);
    }
p1:
    {
        RakNet::BitStream data;
        data.Write(ClientMessage::changeGroup);
        data.Write(static_cast<uint8_t>(group));
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
                tinyAI.connect([&](const RakNet::BitStream& data, PacketPriority pp, PacketReliability pr) {
                    peer.Send(&data, pp, pr, 0, server, false);
                },str.C_String(),group);
            }
            CheckHeader(ServerMessage::go) {
                RakNet::BitStream data(packet->data, packet->length,false);
                data.IgnoreBytes(1);
                Vector2 p;
                data.Read(p);
                tinyAI.setRoot(p);
                std::cout << "Go!" << std::endl;
                goto p2;
            }
        }
        std::this_thread::sleep_for(1ms);
    }
p2:
    struct UnitInfo final {
        uint32_t id;
        Vector3 pos;
        bool operator<(const UnitInfo& rhs) const {
            return id < rhs.id;
        }
    };

    bool isStop = false;
    std::set<uint32_t> old;

    auto begin = std::chrono::system_clock::now();
    while (true) {
        std::vector<unsigned char> latestData;
        FORNET{
            if (isStop)continue;
            RakNet::BitStream data(packet->data, packet->length, false);
            data.IgnoreBytes(1);
            CheckBegin;
            CheckHeader(ServerMessage::out) {
                std::cout << "What a pity!" << std::endl;
                isStop = true;
                continue;
            }
            CheckHeader(ServerMessage::stop) {
                std::cout << "The game stopped." << std::endl;
                isStop = true;
                continue;
            }
            CheckHeader(ServerMessage::win) {
                std::cout << "We are winner!" << std::endl;
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
                tinyAI.newUnit(u);
            }
            else {
                copy.erase(iter);
                tinyAI.updateUnit(u);
            }
            if (u.group == group)
                ++mine;
            else
                ++armies;
        }

        for (auto&& x : copy)
            tinyAI.deleteUnit(x);

        tinyAI.update();

        std::cout << "mine:" << mine << " armies:" << armies << std::endl;
        std::this_thread::sleep_for(1s);
    }

    auto second = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - begin).count();
    std::cout << "time:" << second / 60 << ":" << second % 60 << std::endl;

    peer.Shutdown(500, 0, IMMEDIATE_PRIORITY);
    std::cin.get();
    std::cin.get();
    return 0;
}

#undef FORNET
