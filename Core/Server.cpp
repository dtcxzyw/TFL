#include "Server.h"
#include "Message.h"
#include <functional>

std::unique_ptr<Server> localServer;

void Server::send(uint8_t group, const RakNet::BitStream & data, PacketPriority priority) {
    for (auto&& c : mClients)
        if (c.second.group == group)
            mPeer->Send(&data, priority,
                PacketReliability::RELIABLE_ORDERED, 0, c.first, false);
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
        k.time += getUnit(k.id).getTime() / mSpeed;
    else
        k.time = Game::getAbsoluteTime();
}

Server::Server(const std::string & path) :
    mPeer(RakNet::RakPeerInterface::GetInstance()), mState(false),
    mMap(path), mMapName(path), mSpeed(1.0f) {
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
        RakNet::BitStream data(packet->data, packet->length, false);
        data.IgnoreBytes(1);
        CheckBegin;
        CheckHeader(ID_NEW_INCOMING_CONNECTION) {
            mClients[packet->systemAddress];
            INFO("A client has connected this server.", "(IP=",
                packet->systemAddress.ToString(), ")");
            RakNet::BitStream info;
            info.Write(ServerMessage::info);
            info.Write(pakKey);
            info.Write(mMapName.c_str());
            mPeer->Send(&info, IMMEDIATE_PRIORITY, RELIABLE_ORDERED, 0,
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
    std::vector<RakNet::SystemAddress> deferred;
    for (auto&& x : mClients)
        if (mPeer->GetConnectionState(x.first) != RakNet::ConnectionState::IS_CONNECTED)
            deferred.emplace_back(x.first);
    for (auto&& x : deferred) {
        INFO("A client has disconnected this server.", "(IP=", x.ToString(), ")");
        mClients.erase(x);
    }
    std::vector<uint8_t> list;
    for (auto&& x : mGroups) {
        bool flag = true;
        for (auto&& y : mClients)
            if (y.second.group == x.first) {
                flag = false;
                break;
            }
        if (flag)
            list.emplace_back(x.first);
    }
    for (auto&& x : list)
        mGroups.erase(x);
    return mClients;
}

void Server::run() {
    mScene = Scene::create();
    mMap.set(mScene->addNode("terrain"));

    for (auto&& x : mMap.getKey())
        mKey.emplace_back(x);

    bool flag[6]{};
    for (auto&& x : mClients)
        flag[x.second.group] = true;

    auto time = Game::getAbsoluteTime() + getUnit(0).getTime() / mSpeed;

    for (uint8_t i = 1; i <= 5; ++i)
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
    getClientInfo();

    for (auto&& x : mClients) {
        RakNet::BitStream data;
        data.Write(ServerMessage::go);
        data.Write(mKey[mGroups[x.second.group].key.front()].pos);
        mPeer->Send(&data, IMMEDIATE_PRIORITY, RELIABLE_ORDERED, 0, x.first, false);
    }

    mState = true;
}

void Server::update(float delta) {

    delta *= mSpeed;

    if (!mState)return;

    getClientInfo();

    if (mGroups.empty()) {
        stop();
        return;
    }

    for (auto packet = mPeer->Receive(); packet; mPeer->DeallocatePacket(packet), packet = mPeer->Receive()) {
        if (mClients.find(packet->systemAddress) == mClients.cend())continue;
        RakNet::BitStream data(packet->data, packet->length, false);
        data.IgnoreBytes(1);
        auto group = mClients[packet->systemAddress].group;
        auto& units = mGroups[group].units;
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
            if (mGroups[group].weight.size() <= id)continue;
            data.Read(weight);
            if (weight < 1)weight = 1;
            if (weight > 1000)weight = 1000;
            mGroups[group].weight[id] = weight;
        }
        CheckHeader(ClientMessage::setAttackTarget) {
            uint32_t x, y;
            while (data.Read(x) && data.Read(y)) {
                auto xi = units.find(x);
                if (xi != units.cend()) {
                    auto p = getUnitPos(y);
                    if (!p.isZero()) {
                        xi->second.setMoveTarget({ p.x, p.z });
                        xi->second.setAttackTarget(y);
                    }
                }
            }
        }
        CheckHeader(ClientMessage::setMoveTarget) {
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
        CheckHeader(ClientMessage::load) {
            uint32_t id;
            data.Read(id);
            auto object = units.find(id);
            if (object != units.cend()) {
                auto p = object->second.getRoughPos();
                Vector2 pos = { p.x,p.z };
                uint32_t load;
                while (data.Read(load)) {
                    auto iter = units.find(load);
                    if (iter != units.cend() && !iter->second.getKind().getLoading()) {
                        iter->second.setMoveTarget(pos);
                        iter->second.setLoadTarget(id);
                    }
                }
            }
        }
        CheckHeader(ClientMessage::release) {
            uint32_t object;
            data.Read(object);
            auto it = units.find(object);
            if (it != units.cend())
                releaseUnit(it->second);
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
                    if (std::find_if(mDeferred.cbegin(), mDeferred.cend(), [&u](auto&& x) {
                        return x.id == u.first;
                    }) != mDeferred.cend())continue;
                    auto p = u.second.getNode()->getTranslation();
                    minDis = std::min(minDis, k.pos.distanceSquared({ p.x,p.z }));
                }
                dis[g.first] = minDis;
            }
            auto cnt = std::count_if(dis.cbegin(), dis.cend(), [](auto p) {return p.second < 40000.0f; });

            if (cnt == 1) {
                auto min = std::min_element(dis.cbegin(), dis.cend(),
                    [](auto p1, auto p2) {return p1.second < p2.second; });
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

    auto win = [this](uint8_t group) {
        RakNet::BitStream data;
        data.Write(ServerMessage::win);
        send(group, data, PacketPriority::IMMEDIATE_PRIORITY);
        stop();
    };

    uint8_t out = 0;
    for (auto && g : mGroups) {
        if (g.second.key.empty() && g.second.units.empty()) {
            RakNet::BitStream data;
            data.Write(ServerMessage::out);
            send(g.first, data, PacketPriority::IMMEDIATE_PRIORITY);
            ++out;
        }
        else if (g.second.key.size() == mKey.size()) {
            win(g.first);
            return;
        }
    }

    if (out == mGroups.size() - 1)
        for (auto && g : mGroups)
            if (!(g.second.key.empty() && g.second.units.empty())) {
                win(g.first);
                return;
            }

    //produce unit
    auto now = Game::getAbsoluteTime();
    bool flag;
    std::uniform_real_distribution<float> dis(-100.0f, 100.0f);
    do {
        flag = false;
        for (auto&& k : mKey) {
            if (k.owner != KeyInfo::nil && Game::getAbsoluteTime() > k.time) {
                auto pos = Vector2{ k.pos.x + dis(mt),k.pos.y + dis(mt) };
                Vector3 p(pos.x, mMap.getHeight(pos.x, pos.y) + 10.0f, pos.y);
                auto id = UnitInstance::askID();
                mGroups[k.owner].units.
                    emplace(id,std::move(UnitInstance{ getUnit(k.id), k.owner, id, mScene.get(), true, p }));
                mGroups[k.owner].units[id].update(0);
                mCheck.insert({ id,k.owner,&mGroups[k.owner].units[id] });
                chooseNew(k);
                flag = true;
            }
        }
    } while (flag);

    {
        //shuffle groups to make the game blance.
        std::vector<CheckInfo> units;

        for (auto&& x : mGroups)
            for (auto&& u : x.second.units)
                units.push_back({ u.first,x.first,&u.second });

        std::shuffle(units.begin(), units.end(), mt);

        for (auto&& u : units) {
            if (u.instance->isStoped() && u.instance->getLoadTarget()) {
                auto&& units = mGroups[u.group].units;
                auto x = units.find(u.instance->getLoadTarget());
                if (x != units.cend() && x->second.getLoadSize()<x->second.getKind().getLoading()) {
                    auto p = x->second.getRoughPos();
                    u.instance->setMoveTarget({ p.x,p.z });
                }
                else u.instance->setLoadTarget(0);
            }

            if (u.instance->updateSum(delta))
                mCheck.insert(u);
        }
    }

    {
        auto now = Game::getAbsoluteTime();
        mDeferred.erase(std::remove_if(mDeferred.begin(), mDeferred.end(),
            [this, now](const DiedInfo& x) {
            if (now - x.time > 5000.0) {
                auto& units = mGroups[x.group].units;
                auto i = units.find(x.id);
                if (i != units.cend()) {
                    mScene->removeNode(i->second.getNode());
                    mGroups[x.group].units.erase(i);
                }
                return true;
            }
            return false;
        }),mDeferred.end());
    }

    std::set<CheckInfo> newCheck;
    auto test = [&](uint32_t id) {
        return std::find_if(mDeferred.cbegin(), mDeferred.cend(),
            [id](auto&& x) {return x.id == id; }) == mDeferred.cend();
    };

    now = Game::getAbsoluteTime();

    do {
        if (newCheck.size()) {
            mCheck.swap(newCheck);
            newCheck.clear();
        }

        for (auto&& c : mCheck)
            if (mGroups[c.group].units.find(c.id) != mGroups[c.group].units.cend()) {
                auto bs1 = c.instance->getBound();
                for (auto&& x : mGroups)
                    for (auto&& u : x.second.units)
                        if (c.id != u.first) {
                            auto bs2 = u.second.getBound();
                            if (bs1.intersects(bs2)) {
                                if (c.group == x.first && test(u.first) && test(c.id) &&
                                    ((c.instance->getLoadTarget() == u.first && u.second.tryLoad(*c.instance)) ||
                                    (u.second.getLoadTarget() == c.id && c.instance->tryLoad(u.second)))) {
                                    if (c.instance->getLoadTarget() == u.first)
                                        mDeferred.push_back({ c.group, c.id,0.0f });
                                    else 
                                        mDeferred.push_back({ u.second.getGroup(), u.first,0.0f });
                                }
                                else {
                                    auto v = bs1.center - bs2.center;
                                    v.y = 0.0f;
                                    v.normalize();
                                    v *= bs1.radius + bs2.radius - bs1.center.distance(bs2.center);
                                    v *= 1.3f;
                                    auto cube = [](float x) {return x*x*x; };
                                    auto ss = cube(bs1.radius) + cube(bs2.radius);
                                    c.instance->getNode()->translate(v*cube(bs2.radius) / ss);
                                    u.second.getNode()->translate(-v*cube(bs1.radius) / ss);
                                    newCheck.insert(c);
                                    newCheck.insert({ u.first,x.first,&u.second });
                                }
                            }
                        }
                for (auto&& x : mMap.getKey()) {
                    Vector3 p{ x.x,mMap.getHeight(x.x,x.y),x.y };
                    BoundingSphere bs2{ p,5.0f };
                    if (bs1.intersects(bs2)) {
                        auto v = bs1.center - bs2.center;
                        v.y = 0.0f;
                        v.normalize();
                        v *= bs1.radius + bs2.radius - bs1.center.distance(bs2.center);
                        v *= 1.3f;
                        c.instance->getNode()->translate(v);
                        newCheck.insert(c);
                    }
                }
            }
    } while (newCheck.size() && Game::getAbsoluteTime()-now<=10.0);

    if (newCheck.size()) {
        mCheck.swap(newCheck);
        newCheck.clear();
    }

    {
        std::map<uint8_t, std::set<uint32_t>> duang;
        std::map<uint32_t, DuangSyncInfo> info;
        std::set<uint32_t> deferred;
        std::vector < std::pair<uint8_t, BoundingSphere>> vbs;

        for (auto&& x : mBullets) {
            x.second.update(delta);
            vbs.emplace_back(x.second.getGroup(), x.second.getHitBound());
        }

        size_t idx = 0;
        for (auto&& x : mBullets) {
            auto bb = vbs[idx].second;
            if (bb.center.x<-mapSizeHF || bb.center.x>mapSizeHF
                || bb.center.z<-mapSizeHF || bb.center.z>mapSizeHF) {
                deferred.insert(x.first);
                ++idx;
                continue;
            }

            bool boom = false;
            for (auto&& g : mGroups)
                for (auto&& u : g.second.units)
                    if (u.second.getGroup() != x.second.getGroup() && bb.intersects(u.second.getBound())) {
                        boom = true;
                        goto point;
                    }
            for (auto&& x : mDeferred) {
                auto iter = mGroups[x.group].units.find(x.id);
                if (iter != mGroups[x.group].units.end() && iter->second.getBound().intersects(bb)) {
                    boom = true;
                    goto point;
                }
            }

            for (size_t i = 0; i < vbs.size(); ++i)
                if (vbs[i].first != vbs[idx].first && vbs[i].second.intersects(bb)) {
                    boom = true;
                    goto point;
                }

            if ((bb.center.y - bb.radius < mMap.getHeight(bb.center.x, bb.center.z)))
                boom = true;

        point:
            if (boom) {
                auto b = x.second.getBound();
                for (auto&& g : mGroups)
                    for (auto&& u : g.second.units) {
                        auto bu = u.second.getBound();
                        if (x.second.getGroup() != u.second.getGroup() && b.intersects(bu)) {
                            auto dis = b.center.distance(bu.center);
                            auto fac = (dis - bu.radius) / b.radius;
                            fac = std::max(fac, 0.0f);
                            attack(u.first, x.second.getHarm()*(1.0f - fac*fac));
                        }
                        if (b.center.distanceSquared(bu.center) < u.second.getKind().getFOV())
                            duang[g.first].insert(x.first);
                    }
                deferred.insert(x.first);
                info[x.first] = { x.second.getKind(), b.center };
            }
            ++idx;
        }

        for (auto&& x : duang) {
            RakNet::BitStream data;
            data.Write(ServerMessage::duang);
            data.Write(static_cast<uint16_t>(x.second.size()));
            for (auto&& y : x.second)
                data.Write(info[y]);
            send(x.first, data, PacketPriority::MEDIUM_PRIORITY);
        }

        for (auto&& x : deferred) {
            mScene->removeNode(mBullets[x].getNode());
            mBullets.erase(x);
        }
    }

    std::vector<uint8_t> groups;
    for (auto c : mClients)
        groups.emplace_back(c.second.group);
    uint8_t choose = groups[mt() % groups.size()];
    GroupInfo& update = mGroups[choose];

    //update unit
    {
        std::vector<UnitSyncInfo> saw;
        for (auto&& g : mGroups)
            for (auto&& u : g.second.units) {
                auto p = u.second.getNode()->getTranslation();
                for (auto&& mu : update.units)
                    if ((g.first == mu.second.getGroup() && !u.second.isDied()) ||
                        (!mu.second.isDied() && p.distanceSquared(mu.second.getNode()->getTranslation())
                        <=(p.y>=0.0f?mu.second.getKind().getFOV():mu.second.getKind().getSound()))) {
                        uint16_t uk = getUnitID(u.second.getKind().getName());
                        saw.push_back({ u.first,uk,p,u.second.getNode()->getRotation(),
                            g.first,u.second.getAttackTarget(),
                            g.first == mu.second.getGroup() ? u.second.getLoadSize() : 0
                            ,u.second.getHP() });
                        break;
                    }
            }
        RakNet::BitStream data;
        data.Write(ServerMessage::updateUnit);
        data.Write(static_cast<uint32_t>(saw.size()));
        for (auto&& u : saw)
            data.Write(u);

        send(choose, data, PacketPriority::HIGH_PRIORITY);

        for (auto&& x : update.units) {
            auto old = x.second.getAttackTarget();
            if (old && !getUnitPos(old).isZero())continue;
            auto p = x.second.getRoughPos();
            float md = std::numeric_limits<float>::max();
            uint32_t maxwell = 0;
            for (auto&& y : saw)
                if (y.group != x.second.getGroup()) {
                    auto dis = p.distanceSquared(y.pos);
                    if (dis < md)
                        md = dis, maxwell = y.id;
                }
            x.second.setAttackTarget(maxwell);
        }
    }


    //update bullet
    {
        std::vector<BulletSyncInfo> bullets;
        for (auto&& b : mBullets) {
            auto p = b.second.getBound().center;
            for (auto&& mu : update.units)
                if ((b.second.getGroup() == mu.second.getGroup()) || 
                    (!mu.second.isDied() && p.distanceSquared(mu.second.getNode()->getTranslation())
                    <= (p.y >= 0.0f ? mu.second.getKind().getFOV() : mu.second.getKind().getSound()))) {
                    bullets.push_back({ b.first,b.second.getKind(),p,b.second.getNode()->getRotation() });
                    break;
                }
        }

        RakNet::BitStream data;
        data.Write(ServerMessage::updateBullet);
        data.Write(static_cast<uint32_t>(bullets.size()));
        for (auto&& b : bullets)
            data.Write(b);

        send(choose, data, PacketPriority::HIGH_PRIORITY);
    }

    //update state
    {
        RakNet::BitStream data;
        data.Write(ServerMessage::updateState);
        for (auto&& x : update.weight)
            data.Write(x);
        float now = Game::getAbsoluteTime();
        for (auto&& k : update.key) {
            ProducingSyncInfo info{ k,mKey[k].id,std::max((mKey[k].time-now)/1000.0f,0.0f) };
            data.Write(info);
        }
        send(choose, data, PacketPriority::MEDIUM_PRIORITY);
    }
}

void Server::stop() {
    mCheck.clear();
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
                mDeferred.push_back({ g.first,id,Game::getAbsoluteTime() });
            break;
        }
    }
}

void Server::newBullet(BulletInstance && bullet) {
    auto id = BulletInstance::askID();
    mBullets.insert({ id,std::move(bullet) });
    mScene->addNode(mBullets[id].getNode());
}

Vector3 Server::getUnitPos(uint32_t id) const {
    for (auto&& g : mGroups) {
        auto&& units = g.second.units;
        auto i = units.find(id);
        if (i != units.end())
            return i->second.isDied() ? Vector3{}:i->second.getNode()->getTranslation();
    }
    return {};
}

void Server::changeSpeed(float speed) {
    mSpeed = speed;
    RakNet::BitStream data;
    data.Write(ServerMessage::changeSpeed);
    data.Write(mSpeed);
    for (auto&& c : getClientInfo())
        mPeer->Send(&data, PacketPriority::IMMEDIATE_PRIORITY,
            PacketReliability::RELIABLE_ORDERED, 0, c.first, false);
}

void Server::releaseUnit(UnitInstance & instance) {
    std::uniform_real_distribution<float> URD(-1.0f, 1.0f);
    auto pos = instance.getRoughPos() + instance.getKind().getReleaseOffset();
    auto res = instance.release();
    auto group = instance.getGroup();
    mCheck.insert({ instance.getID(),group,&instance });
    auto&& units = mGroups[group].units;
    for (auto&& x : res) {
        auto id = UnitInstance::askID();
        auto p = pos;
        p.x += URD(mt), p.y += URD(mt), p.z += URD(mt);
        units.insert({ id,
            std::move(UnitInstance{ getUnit(x.first), group, id, mScene.get(), true,p }) });
        units[id].setHP(x.second);
        units[id].update(0);
        mCheck.insert({ id,group,&units[id] });
    }
}

GroupInfo::GroupInfo() :weight(globalUnits.size(), 1) {}

KeyInfo::KeyInfo(Vector2 p) : owner(nil), id(none), pos(p) {}

