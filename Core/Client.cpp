#include "Client.h"
#include "Message.h"
#include "Unit.h"
#include <thread>
#include <chrono>
using namespace std::literals;

std::unique_ptr<Client> localClient;

void Client::drawNode(Node * node) {
    if (node->getDrawable() && node->getBoundingSphere().intersects(mCamera->getFrustum()))
        node->getDrawable()->draw();
    for (auto i = node->getFirstChild(); i; i = i->getNextSibling())
        drawNode(i);
}

Vector2 Client::getPoint(int x, int y) const {
    auto game = Game::getInstance();
    Ray ray;
    auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
    mCamera->setAspectRatio(rect.width / rect.height);
    mCamera->pickRay(rect, x, y, &ray);
#ifdef ANDROID
    Vector3 maxwell;
    float minError = std::numeric_limits<float>::max();
    auto reslover = [ray](float x) {return ray.getOrigin() -
        ray.getDirection()*(ray.getOrigin().y*x / ray.getDirection().y); };
    constexpr auto num = 16.0f;
    for (uint8_t i = 0; i <num ; ++i) {
        auto x = i/num;
        auto p=reslover(x);
        auto error = std::abs(p.y - mMap->getHeight(p.x, p.z));
        if (error < minError)
            minError = error, maxwell = p;
    }
    return { maxwell.x,maxwell.z };
#else
    PhysicsController::HitResult res;
    game->getPhysicsController()->rayTest(ray, 1e10f, &res);
    return { res.point.x,res.point.z };
#endif
}

bool Client::checkCamera() {
    auto game = Game::getInstance();
    auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
    auto test = [this, rect](float x, float y) {
        Ray r;
        mCamera->pickRay(rect, x, y, &r);
        BoundingBox b(-5000.0f, 0.0f, -5000.0f, 5000.0f, 1.0f, 5000.0f);
        return b.intersects(r) == Ray::INTERSECTS_NONE;
    };
    return test(0, 0) || test(0, rect.height) || test(rect.width, 0) || test(rect.width, rect.height);
}

void Client::move(int x, int y) {
    if (mChoosed.size()) {
        RakNet::BitStream data;
        data.Write(ClientMessage::setMoveTarget);
        data.Write(getPoint(x, y));
        data.Write(static_cast<uint32_t>(mChoosed.size()));
        for (auto x : mChoosed)
            data.Write(x);
        mPeer->Send(&data, PacketPriority::HIGH_PRIORITY,
            PacketReliability::RELIABLE, 0, mServer, false);
    }
}

Client::Client(const std::string & server,bool& res) :
    mPeer(RakNet::RakPeerInterface::GetInstance()), mServer(server.c_str(), 23333),
    mState(false), mWeight(globalUnits.size(), 1),mCnt(0.0f) {

    RakNet::SocketDescriptor SD;
    mPeer->Startup(1, &SD, 1);
    mPeer->SetMaximumIncomingConnections(1);
    auto result = mPeer->Connect(server.c_str(), 23333, nullptr, 0);
    auto wait = [&] {
        auto t = std::chrono::system_clock::now();
        while ((mPeer->NumberOfConnections() == 0) &&
            (std::chrono::system_clock::now() - t <= 500ms))
            std::this_thread::yield();
        return mPeer->NumberOfConnections();
    };

    if (result != RakNet::CONNECTION_ATTEMPT_STARTED
        || !wait())
        INFO("Failed to connect to the server.");

    mRECT = SpriteBatch::create("res/common/rect.png");
    res=mPeer->NumberOfConnections();
}

Client::~Client() {
    mPeer->Shutdown(500, 0, PacketPriority::IMMEDIATE_PRIORITY);
    RakNet::RakPeerInterface::DestroyInstance(mPeer);
}

void Client::changeGroup(uint8_t group) {
    mGroup = group;
    RakNet::BitStream stream;
    stream.Write(ClientMessage::changeGroup);
    stream.Write(group);
    mPeer->Send(&stream, PacketPriority::IMMEDIATE_PRIORITY, PacketReliability::RELIABLE_ORDERED
        , 0, mServer, false);
}

Client::WaitResult Client::wait() {
    auto packet = mPeer->Receive();
    if (packet) {
        CheckBegin;
        CheckHeader(ID_DISCONNECTION_NOTIFICATION) {
            mPeer->DeallocatePacket(packet);
            return WaitResult::Disconnected;
        }
        CheckHeader(ServerMessage::info) {
            RakNet::BitStream data(packet->data, packet->length, false);
            data.IgnoreBytes(1);
            uint64_t key;
            data.Read(key);
            if (key != pakKey) {
                INFO("The pakKey of server is not equal yours.");
                mPeer->DeallocatePacket(packet);
                return WaitResult::Disconnected;
            }
            RakNet::RakString str;
            data.Read(str);
            mMap = std::make_unique<Map>(str.C_String(), false);
            INFO("Load map ", str.C_String());
        }
        CheckHeader(ServerMessage::go) {
            RakNet::BitStream data(packet->data, packet->bitSize >> 3, false);
            data.IgnoreBytes(1);
            mScene = Scene::create();
            mCamera =
                Camera::createPerspective(45.0f, Game::getInstance()->getAspectRatio(), 1.0f, 20000.0f);
            mScene->addNode()->setCamera(mCamera.get());
            mScene->setActiveCamera(mCamera.get());
            mMap->set(mScene->addNode("terrain"));
            auto c = mCamera->getNode();
            c->rotateX(-M_PI_2);
            Vector2 p;
            data.Read(p);
            c->setTranslation(p.x, mMap->getHeight(p.x, p.y) + 500.0f, p.y);
            mX = mY = mBX = mBY = 0;
            mState = true;
            mPeer->DeallocatePacket(packet);
            return WaitResult::Go;
        }
        mPeer->DeallocatePacket(packet);
        return wait();
    }
    return WaitResult::None;

}

void Client::stop() {
    mState = false;
    mScene.reset();
    mCamera.reset();
    mUnits.clear();
    mChoosed.clear();
    RakNet::BitStream stream;
    stream.Write(ClientMessage::exit);
    mPeer->Send(&stream, PacketPriority::IMMEDIATE_PRIORITY, PacketReliability::RELIABLE_ORDERED
        , 0, mServer, false);
}

bool Client::update(float delta) {

#ifdef WIN32
    if (!(mBX || mBY)) {
        auto game = Game::getInstance();
        auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
        float vx = (mX - rect.width / 2) / rect.width;
        float vy = (mY - rect.height / 2) / rect.height;
        constexpr float unit = 0.001f;
        float m = unit*delta;
        float mx = (std::abs(vx) <= 0.5 && std::abs(vx) >= 0.4) ? (vx > 0.0f ? m : -m) : 0.0f;
        float my = (std::abs(vy) <= 0.5 && std::abs(vy) >= 0.4) ? (vy > 0.0f ? m : -m) : 0.0f;
        moveEvent(mx, my);
    }
#endif // WIN32

    std::set<uint32_t> others;
    std::set<uint32_t> mine;

    auto isStop = false;
    for (auto packet = mPeer->Receive(); packet; mPeer->DeallocatePacket(packet), packet = mPeer->Receive()) {
        if (isStop)continue;
        RakNet::BitStream data(packet->data, packet->length, false);
        data.IgnoreBytes(1);
        CheckBegin;
        CheckHeader(ServerMessage::stop) {
            INFO("The game stopped.");
            isStop = true;
            continue;
        }
        CheckHeader(ServerMessage::win) {
            INFO("We are winner!");
        }
        CheckHeader(ServerMessage::out) {
            INFO("What a pity!");
            isStop = true;
            continue;
        }
        CheckHeader(ServerMessage::updateUnit) {
            uint32_t size;
            data.Read(size);
            std::set<uint32_t> old;
            for (auto&& u : mUnits)
                old.insert(u.first);
            for (uint32_t i = 0; i < size; ++i) {
                UnitSyncInfo u;
                data.Read(u);
                auto oi = old.find(u.id);
                if (oi != old.cend()) {
                    mUnits[u.id].getNode()->setTranslation(u.pos);
                    mUnits[u.id].getNode()->setRotation(u.rotation);
                    mUnits[u.id].setAttackTarget(u.at);
                    old.erase(oi);
                }
                else {
                    auto i = mUnits.insert({ u.id,
                        std::move(UnitInstance{ getUnit(u.kind), u.group, u.id, mScene.get(), false, u.pos })});
                    i.first->second.getNode()->setRotation(u.rotation);
                    i.first->second.update(0);
                    i.first->second.setAttackTarget(u.at);
                }
                if(u.isDied)
                    mUnits[u.id].attacked(1e10f);
                else {
                    if (u.group == mGroup)
                        mine.insert(u.id);
                    else
                        others.insert(u.id);
                }
            }
            for (auto&& o : old) {
                mScene->removeNode(mUnits[o].getNode());
                mUnits.erase(o);
            }
        }
        CheckHeader(ServerMessage::updateBullet) {
            uint32_t size;
            data.Read(size);
            std::set<uint32_t> old;
            for (auto&& x : mBullets)
                old.insert(x.first);
            for (uint32_t i = 0; i < size; ++i) {
                BulletSyncInfo info;
                data.Read(info);
                auto iter = old.find(info.id);
                if (iter == old.cend()) {
                    mBullets.insert({ info.id,std::move(BulletInstance(info.kind, {}, {}, 0.0f, 0.0f, 0.0f,0)) });
                    mScene->addNode(mBullets[info.id].getNode());
                }
                else old.erase(iter);
                mBullets[info.id].getNode()->setTranslation(info.pos);
                mBullets[info.id].getNode()->setRotation(info.rotation);
            }

            for (auto&& x : old) {
                mScene->removeNode(mBullets[x].getNode());
                mBullets.erase(x);
            }
        }
        CheckHeader(ServerMessage::updateWeight) {
            for (auto& x : mWeight)
                data.Read(x);
        }
    }

    if (isStop) {
        stop();
        return false;
    }

    for (auto&& x : mUnits)
        x.second.update(delta);

    //control
    mCnt += delta;
    if (mCnt>500.0f) {

        RakNet::BitStream data;
        data.Write(ClientMessage::setAttackTarget);
        
        for (auto&& x : mine) {
            float md = std::numeric_limits<float>::max();
            uint32_t maxwell=0;
            for (auto&& y : others) {
              auto dis= mUnits[x].getNode()->getTranslation().
                    distanceSquared(mUnits[y].getNode()->getTranslation());
              if (dis < md)
                  md = dis, maxwell = y;
            }
            if (md < mUnits[x].getKind().getFOV()) {
                data.Write(x);
                data.Write(maxwell);
            }
        }

        mPeer->Send(&data, PacketPriority::HIGH_PRIORITY,
            PacketReliability::RELIABLE_ORDERED, 0, mServer, false);

        mCnt = 0.0f;
    }

    return true;
}

void Client::render() {
    if (mState) {
        auto game = Game::getInstance();
        auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
        game->setViewport(rect);
        mCamera->setAspectRatio(rect.width / rect.height);

        mScene->setAmbientColor(-0.8f, -0.8f, -0.8f);
        for (auto&& x : mUnits) 
            if (x.second.isDied())
                drawNode(x.second.getNode());
        mScene->setAmbientColor(0.0f, 0.3f, 0.0f);
        for (auto&& x : mUnits)
            if (!x.second.isDied() && mChoosed.find(x.first)!=mChoosed.cend())
                drawNode(x.second.getNode());
        mScene->setAmbientColor(0.0f, 0.0f, 0.0f);
        for (auto&& x : mUnits)
            if (!x.second.isDied() && mChoosed.find(x.first) == mChoosed.cend())
                drawNode(x.second.getNode());
        for (auto&& x : mBullets)
            drawNode(x.second.getNode());
        mScene->findNode("terrain")->getDrawable()->draw();

        auto rect2 = gameplay::Rectangle(game->getWidth(), game->getHeight());
        game->setViewport(rect2);
        mCamera->setAspectRatio(rect2.width / rect2.height);

#ifdef WIN32
        if (mBX&&mBY) {
            float x1 = mX, y1 = mY, x2 = mBX, y2 = mBY;
            if (x1 > x2)std::swap(x1, x2);
            if (y1 > y2)std::swap(y1, y2);
            gameplay::Rectangle range{ x1,y1,x2 - x1,y2 - y1 };
            mRECT->start();
            mRECT->draw(range, { 32,32 });
            mRECT->finish();
        }
#endif // WIN32
    }
}

Vector3 Client::getPos(uint32_t id) {
    auto i = mUnits.find(id);
    if (i == mUnits.cend() || i->second.isDied()) 
        return Vector3::zero();
    return i->second.getNode()->getTranslation();
}

float Client::getHeight(int x, int z) const {
    return mMap->getHeight(x, z);
}

void Client::setViewport(uint32_t right) { mRight = right; }

void Client::changeWeight(const std::string& name, uint16_t weight) {
    uint16_t id = getUnitID(name);
    RakNet::BitStream data;
    data.Write(ClientMessage::changeWeight);
    data.Write(id);
    data.Write(weight);
    mPeer->Send(&data, PacketPriority::MEDIUM_PRIORITY, PacketReliability::RELIABLE, 0, mServer, false);
    mWeight[id] = weight;
}

uint16_t Client::getWeight(const std::string & name) const {
    return mWeight[getUnitID(name)];
}

uint16_t Client::getUnitNum() const {
    return mUnits.size();
}

void Client::moveEvent(float x, float y) {
    if (mState) {
        auto node = mCamera->getNode();
        auto pos = node->getTranslation();
        x *= pos.y,y *= pos.y;
        node->translate(x, 0.0f, y);
        auto height = mMap->getHeight(pos.x+x, pos.z+y);
        if (pos.y < height + 100.0f)
            node->setTranslationY(height + 100.0f);

        auto game = Game::getInstance();
        if (game->getWidth() && game->getHeight() && checkCamera())
            node->setTranslation(pos);
    }
}

void Client::scaleEvent(float x) {
    if (mState) {
        auto node = mCamera->getNode();
        auto pos = node->getTranslation();
        auto height = mMap->getHeight(pos.x, pos.z);
        if (pos.y > 200.0f && pos.y - 100.0f > height) {
            node->translateY(x);
            if (checkCamera())
                node->translateY(-x);
        }
    }
}

void Client::mousePos(int x, int y) {
    mX = x, mY = y;
}

void Client::beginPoint(int x, int y) {

    if (!mState)return;

    mBX = x, mBY = y;
}

void Client::endPoint(int x, int y) {
    if (mState) {
        if ((mBX-x)*(mBX-x)+(mBY-y)*(mBY-y)>256) {
            Vector2 b = getPoint(mBX, mBY), e = getPoint(x, y);
            auto x1 = b.x, y1 = b.y, x2 = e.x, y2 = e.y;
            if (x1 > x2)std::swap(x1, x2);
            if (y1 > y2)std::swap(y1, y2);
            mChoosed.clear();
            for (auto&& x : mUnits)
                if (x.second.getGroup() == mGroup) {
                    auto p = x.second.getNode()->getTranslation();
                    if (x1<p.x && x2>p.x && y1<p.z && y2>p.z)
                        mChoosed.insert(x.first);
                }
        }
        else move(x, y);
    }
    mBX = 0, mBY = 0;
}

void Client::cancel() {
    mChoosed.clear();
}

bool Client::hasChoosed() const {
    return mChoosed.size();
}
