#include "Server.h"
#include "Message.h"
#include <functional>

std::unique_ptr<Server> localServer;

void Server::send(uint8_t group, const RakNet::BitStream & data, PacketPriority priority) {
    for (auto&& c : mClients)
        if (c.second.group == group)
            mPeer->Send(&data, priority,
                PacketReliability::RELIABLE, 0, c.first, false);
}

void Server::chooseNew(KeyInfo& k) {
    auto&& w = mGroups[k.owner].weight;
    uint64_t sum = 0;
    for (auto x : w)
        sum += x;
    auto pos = mt() % sum;
    uint16_t idx = 0;
    for (auto x : w) {
        if (x > pos)break;
        ++idx, pos -= x;
    }

    k.id = (idx == w.size()) ? 0 : idx;
    if (k.time)
        k.time += getUnit(k.id).getTime();
    else
        k.time = Game::getAbsoluteTime();
}

Server::Server(const std::string & path) :
    mPeer(RakNet::RakPeerInterface::GetInstance()), mState(false),
    mMap(path, true), mMapName(path) {
    RakNet::SocketDescriptor SD(23333, nullptr);
    mPeer->Startup(16, &SD, 1);
    mPeer->SetMaximumIncomingConnections(16);
}

Server::~Server() {
    mPeer->Shutdown(500, 0, PacketPriority::IMMEDIATE_PRIORITY);
    RakNet::RakPeerInterface::DestroyInstance(mPeer);
}

void Server::waitClient() {
    for (auto packet = mPeer->Receive(); packet; mPeer->DeallocatePacket(packet), packet = mPeer->Receive()) {
        RakNet::BitStream data(packet->data, packet->bitSize >> 3, false);
        data.IgnoreBytes(1);
        CheckBegin;
        CheckHeader(ID_NEW_INCOMING_CONNECTION) {
            mClients[packet->systemAddress];
            INFO("A client has connected this server.", "(IP=",
                packet->systemAddress.ToString(), ")");
            RakNet::BitStream map;
            map.Write(ServerMessage::info);
            map.Write(pakKey);
            map.Write(mMapName.c_str());
            mPeer->Send(&map, IMMEDIATE_PRIORITY, RELIABLE_ORDERED, 0,
                packet->systemAddress, false);
        }
        CheckHeader(ClientMessage::changeGroup) {
            data.Read(mClients[packet->systemAddress].group);
        }
        CheckHeader(ID_DISCONNECTION_NOTIFICATION) {
            INFO("Client ", packet->systemAddress.ToString(), " disconnected.");
            mClients.erase(packet->systemAddress);
        }
    }
}

const std::map<RakNet::SystemAddress, ClientInfo>& Server::getClientInfo() {
    return mClients;
}

void Server::run() {
    mScene = Scene::create();
    mMap.set(mScene->addNode("terrain"));

    for (auto&& x : mMap.getKey())
        mKey.emplace_back(x);

    bool flag[5]{};
    for (auto&& x : mClients)
        flag[x.second.group] = true;

    auto time = Game::getAbsoluteTime() + getUnit(0).getTime();

    for (uint8_t i = 1; i <= 4; ++i)
        if (flag[i]) {
            while (true) {
                auto id = mt() % mKey.size();
                uint8_t&x = mKey[id].owner;
                if (x == KeyInfo::nil) {
                    x = i;
                    mGroups[i].key.push_back(id);
                    mKey[id].id = 0;
                    mKey[id].time = time;
                    break;
                }
            }
        }

    waitClient();

    for (auto&& x : mClients) {
        RakNet::BitStream data;
        data.Write(ServerMessage::go);
        data.Write(mKey[mGroups[x.second.group].key.front()].pos);
        mPeer->Send(&data, IMMEDIATE_PRIORITY, RELIABLE_ORDERED, 0, x.first, false);
    }

    mState = true;
}

void Server::update(float delta) {
    if (!mState)return;

    bool updateWeight = false;

    for (auto packet = mPeer->Receive(); packet; mPeer->DeallocatePacket(packet), packet = mPeer->Receive()) {
        RakNet::BitStream data(packet->data, packet->bitSize >> 3, false);
        data.IgnoreBytes(1);
        CheckBegin;
        CheckHeader(ID_DISCONNECTION_NOTIFICATION) {
            INFO("Client ", packet->systemAddress.ToString(), " disconnected.");
            mClients.erase(packet->systemAddress);
        }
        CheckHeader(ClientMessage::exit) {
            INFO("Client ", packet->systemAddress.ToString(), " exited.");
            mClients.erase(packet->systemAddress);
        }
        CheckHeader(ClientMessage::changeWeight) {
            auto group = mClients[packet->systemAddress].group;
            uint16_t id, weight;
            data.Read(id);
            data.Read(weight);
            mGroups[group].weight[id] = weight;
            updateWeight = true;
        }
        CheckHeader(ClientMessage::setAttackTarget) {
            auto& units = mGroups[mClients[packet->systemAddress].group].units;
            uint32_t x, y;
            while (data.Read(x) && data.Read(y)) {
                auto xi = units.find(x);
                if (xi != units.cend())
                    xi->second.setAttackTarget(y);
            }
        }
        CheckHeader(ClientMessage::setMoveTarget) {
            auto& units = mGroups[mClients[packet->systemAddress].group].units;
            Vector2 pos;
            uint32_t size;
            data.Read(pos);
            data.Read(size);
            for (uint32_t i = 0; i < size; ++i) {
                uint32_t id;
                data.Read(id);
                auto u = units.find(id);
                if (u != units.cend())
                    u->second.setMoveTarget(pos);
            }
        }
    }

    //check owner
    {
        uint8_t idx = 0;
        for (auto&& k : mKey) {
            std::map<uint8_t, float> dis;
            for (auto&& g : mGroups) {
                auto minDis = std::numeric_limits<float>::max();
                for (auto&& u : g.second.units) {
                    auto p = u.second.getNode()->getTranslationWorld();
                    minDis = std::min(minDis, k.pos.distanceSquared({ p.x,p.z }));
                }
                dis[g.first] = minDis;
            }
            auto min = std::min_element(dis.cbegin(), dis.cend(),
                [](auto p1, auto p2) {return p1.second < p2.second; });

            if (min->second < 100000.0f) {
                if (k.owner != KeyInfo::nil) {
                    if (k.owner != min->first) {
                        auto& old = mGroups[k.owner].key;
                        old.erase(std::find(old.cbegin(), old.cend(), idx));
                        k.owner = min->first;
                        mGroups[k.owner].key.emplace_back(idx);
                    }
                }
                else {
                    k.owner = min->first;
                    mGroups[k.owner].key.emplace_back(idx);
                }

                if (k.id == KeyInfo::none)
                    chooseNew(k);
            }
            ++idx;
        }
    }

    //check state
    for (auto && g : mGroups) {
        if (g.second.key.empty() && g.second.units.empty()) {
            RakNet::BitStream data;
            data.Write(ServerMessage::out);
            send(g.first, data, PacketPriority::IMMEDIATE_PRIORITY);
        }
        else if (g.second.key.size() == mKey.size()) {
            RakNet::BitStream data;
            data.Write(ServerMessage::win);
            send(g.first, data, PacketPriority::IMMEDIATE_PRIORITY);
            stop();
            return;
        }
    }

    //update scene
    struct CheckInfo {
        uint32_t id;
        uint8_t group;
        UnitInstance* instance;
    };
    std::vector<CheckInfo> check;

    //produce unit
    auto now = Game::getAbsoluteTime();
    bool flag;
    do {
        flag = false;
        for (auto&& k : mKey) {
            if (k.owner != KeyInfo::nil && Game::getAbsoluteTime() > k.time) {
                auto pos = Vector2{ k.pos.x,k.pos.y }+
                    Vector2{ 100.0f / 16.0f*(mt() % 100),  100.0f / 16.0f*(mt() % 100) }
                -Vector2{ 5000.0f,5000.0f } / 16.0f;
                Vector3 p(pos.x, mMap.getHeight(pos.x, pos.y) + 10.0f, pos.y);
                auto id = UnitInstance::askID();
                mGroups[k.owner].units.
                    insert({ id,std::move(UnitInstance{ getUnit(k.id), k.owner, id, mScene.get(), true, p }) });
                mGroups[k.owner].units[id].update(0);
                check.push_back({ id,k.owner,&mGroups[k.owner].units[id] });
                chooseNew(k);
                flag = true;
            }
        }
    }
    while (flag && Game::getAbsoluteTime() - now < 10.0);

    {
        //shuffle groups to make the game blance.
        std::vector<CheckInfo> units;

        for (auto&& x : mGroups)
            for (auto&& u : x.second.units)
                units.push_back({ u.first,x.first,&u.second });

        std::shuffle(units.begin(), units.end(), mt);

        for (auto&& u : units) {
            auto old = u.instance->getNode()->getTranslation();
            u.instance->update(delta);
            if (old != u.instance->getNode()->getTranslation())
                check.emplace_back(u);
        }
    }

    for (auto&& x : mDeferred) {
        auto& units = mGroups[x.first].units;
        auto i = units.find(x.second);
        if (i != units.cend()) {
            mScene->removeNode(i->second.getNode());
            mGroups[x.first].units.erase(i);
        }
    }
    mDeferred.clear();

    for (auto&& c : check)
        if (mGroups[c.group].units.find(c.id) != mGroups[c.group].units.cend()) {
            for (auto&& x : mGroups)
                for (auto&& u : x.second.units)
                    if (c.id != u.first) {
                        auto bs1 = c.instance->getBound();
                        auto bs2 = u.second.getBound();
                        if (bs1.intersects(bs2)) {
                            auto v = bs1.center - bs2.center;
                            v.normalize();
                            v *= bs1.radius + bs2.radius - bs1.center.distance(bs2.center);
                            auto cube = [](float x) {return x*x*x; };
                            auto ss = cube(bs1.radius) + cube(bs2.radius);
                            c.instance->getNode()->translate(v*cube(bs2.radius) / ss);
                            u.second.getNode()->translate(-v*cube(bs1.radius) / ss);
                        }
                    }
        }

    std::vector<uint8_t> groups;
    for (auto c : mClients)
        groups.emplace_back(c.second.group);
    uint8_t choose = groups[mt() % groups.size()];
    GroupInfo& update = mGroups[choose];

    //update unit
    std::vector<UnitSyncInfo> saw;
    for (auto&& g : mGroups)
        for (auto&& u : g.second.units) {
            auto p = u.second.getNode()->getTranslationWorld();
            for (auto&& mu : update.units)
                if (p.distanceSquared(mu.second.getNode()->getTranslationWorld())
                    < mu.second.getKind().getFOV()) {
                    uint16_t uk = getUnitID(u.second.getKind().getName());
                    saw.push_back({ u.first,uk,p,u.second.getNode()->getRotation(),
                        g.first,u.second.getAttackTarget() });
                    break;
                }
        }
    RakNet::BitStream data;
    data.Write(ServerMessage::updateUnit);
    data.Write(static_cast<uint32_t>(saw.size()));
    for (auto&& u : saw)
        data.Write(u);

    send(choose, data, PacketPriority::HIGH_PRIORITY);

    //update weight
    if (updateWeight) {
        RakNet::BitStream weight;
        weight.Write(ServerMessage::updateWeight);
        for (auto&& x : update.weight)
            weight.Write(x);
        send(choose, weight, PacketPriority::MEDIUM_PRIORITY);
    }
}

void Server::stop() {
    mKey.clear();
    mScene.reset();
    mGroups.clear();
    RakNet::BitStream data;
    data.Write(ServerMessage::stop);
    for (auto && c : mClients)
        mPeer->Send(&data, PacketPriority::IMMEDIATE_PRIORITY, PacketReliability::RELIABLE_ORDERED,
            0, c.first, false);
    mState = false;
    for (auto packet = mPeer->Receive(); packet; mPeer->DeallocatePacket(packet), packet = mPeer->Receive());
}

std::string Server::getIP() {
    std::string s;
    std::set<std::string> IP;
    IP.insert(mPeer->GetInternalID().ToString(false));
    for (uint32_t i = 0; i < mPeer->GetNumberOfAddresses(); ++i)
        IP.insert(mPeer->GetLocalIP(i));
    for (auto&& x : IP)
        s += x + ' ';
    return s;
}

void Server::attack(uint32_t id, float harm) {
    for (auto&& g : mGroups) {
        auto&& units = g.second.units;
        auto i = units.find(id);
        if (i != units.end()) {
            if (i->second.attacked(harm))
                mDeferred.push_back({ g.first,id });
            break;
        }
    }
}

GroupInfo::GroupInfo() :weight(globalUnits.size(), 1) {}

KeyInfo::KeyInfo(Vector2 p) : owner(nil), id(none), pos(p) {}

