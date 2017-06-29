#include <iostream>
#include <fstream>
#include <string>
#include <Vector2.h>
#include <Vector3.h>
#include <Quaternion.h>
using namespace gameplay;
#include "../Core/Message.h"
#include <RakPeer.h>
#include <set>
#include <thread>
#include <map>
#include <vector>
#include <random>
using namespace std::literals;
#undef max

#define FORNET for (auto packet = peer.Receive(); packet; peer.DeallocatePacket(packet), packet = peer.Receive())

std::mt19937_64 mt(std::chrono::high_resolution_clock::now().time_since_epoch().count());

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
            CheckHeader(ServerMessage::go) {
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
                latestData=std::vector<unsigned char>(packet->data, packet->data + packet->length);
            }
        }
        if (isStop || 
            peer.GetConnectionState(server)!=RakNet::ConnectionState::IS_CONNECTED)break;
        if (latestData.empty())continue;

        std::set<UnitInfo> mine;
        std::set<UnitInfo> army;

        RakNet::BitStream latest(latestData.data(),latestData.size(),false);
        latest.IgnoreBytes(1);
        uint32_t size;
        latest.Read(size);
        for (uint32_t i = 0; i < size; ++i) {
            UnitSyncInfo u;
            latest.Read(u);
            if (u.group == group) {
                mine.insert({ u.id,u.pos });
                RakNet::BitStream data;
                data.Write(ClientMessage::setMoveTarget);
                if (old.find(u.id) == old.cend()) {
                    auto random = [] {return mt() % 10000; };
                    Vector2 p = { -5000.0f + random(),-5000.0f + random() };
                    data.Write(p);
                    data.Write(static_cast<uint32_t>(1));
                    data.Write(u.id);
                    peer.Send(&data, PacketPriority::HIGH_PRIORITY,
                        PacketReliability::RELIABLE, 0, server, false);
                    old.insert(u.id);
                }
            }
            else
                army.insert({ u.id,u.pos });
        }

        RakNet::BitStream data;
        data.Write(ClientMessage::setAttackTarget);

        {
            for (auto&& m : mine) {
                float md = std::numeric_limits<float>::max();
                uint32_t maxwell = 0;
                for (auto&& a : army) {
                    auto dis = m.pos.distanceSquared(a.pos);
                    if (dis < md)
                        md = dis, maxwell = a.id;
                    if (md < 1e6f) {
                        data.Write(m);
                        data.Write(maxwell);
                    }
                }
            }
            peer.Send(&data, PacketPriority::HIGH_PRIORITY, PacketReliability::RELIABLE, 0, server, false);
        }

        std::cout << "mine:" << mine.size() << " armies:" << army.size()<<std::endl;
        std::this_thread::sleep_for(1s);
    }

    peer.Shutdown(500,0,IMMEDIATE_PRIORITY);
    std::cin.get();
    std::cin.get();
    return 0;
}
