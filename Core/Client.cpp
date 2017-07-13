#include "Client.h"
#include "Message.h"
#include "Unit.h"
#include <thread>
#include <chrono>
using namespace std::literals;

std::unique_ptr<Client> localClient;

bool Client::resolveAutoBinding(const char * autoBinding, Node * node, MaterialParameter * parameter) {
#define CMP(x) (strcmp(autoBinding,x)==0)
    if (CMP("SHADOW_MAP"))
        parameter->setSampler(mShadowMap.get());
    else if (CMP("MAP_SIZE"))
        parameter->setInt(shadowSize);
    else if (CMP("LIGHT_MATRIX"))
        parameter->bindValue(this, &Client::getMat);
    else if (CMP("BIAS"))
        parameter->setValue(bias);
    else if (CMP("ARGS")) {
        auto dir = -Vector3::one();
        dir.normalize();
        float mat[16];
        *reinterpret_cast<Vector3*>(mat) = dir;
        mat[3] = shadowSize;
        *reinterpret_cast<Vector3*>(mat+4) = Vector3::one()*1.5f;
        mat[7] = bias;
        parameter->setValue(Matrix(mat));
    }
    else return false;
#undef CMP
    return true;
}

Matrix Client::getMat() const {
    return mLightSpace;
}

void Client::drawNode(Node * node, bool shadow) {
    if (node->getDrawable()
        && node->getBoundingSphere().intersects(mScene->getActiveCamera()->getFrustum())
        ) {

        auto m = dynamic_cast<Model*>(node->getDrawable());
        auto t = dynamic_cast<Terrain*>(node->getDrawable());

        if (shadow) {
            if (m) m->getMaterial()->setTechnique("depth");
            else if (t) {
                for (unsigned int i = 0; i < t->getPatchCount(); ++i)
                    t->getPatch(i)->getMaterial(0)->setTechnique("depth");
            }
        }
        else {
            if (m) m->getMaterial()->setTechnique("shadow");
            else if (t) {
                for (unsigned int i = 0; i < t->getPatchCount(); ++i)
                    t->getPatch(i)->getMaterial(0)->setTechnique("shadow");
            }
        }

        node->getDrawable()->draw();
    }

    for (auto i = node->getFirstChild(); i; i = i->getNextSibling())
        drawNode(i, shadow);
}

Vector3 Client::getPoint(int x, int y) const {
    auto game = Game::getInstance();
    Ray ray;
    auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
    mCamera->setAspectRatio(rect.width / rect.height);
    mCamera->pickRay(rect, x, y, &ray);
    Vector3 maxwell;
    float minError = std::numeric_limits<float>::max();
    auto reslover = [ray](float x) {return ray.getOrigin() -
        ray.getDirection()*(ray.getOrigin().y*x / ray.getDirection().y); };
    constexpr auto num = 16.0f;
    for (uint8_t i = 0; i < num; ++i) {
        auto x = i / num;
        auto p = reslover(x);
        auto error = std::abs(p.y - mMap->getHeight(p.x, p.z));
        if (error < minError)
            minError = error, maxwell = p;
    }
    return maxwell;
}

bool Client::checkCamera() {
    auto game = Game::getInstance();
    auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
    auto test = [this, rect](float x, float y) {
        Ray r;
        mCamera->pickRay(rect, x, y, &r);
        BoundingBox b(-mapSizeHF, 0.0f, -mapSizeHF, mapSizeHF, 1.0f, mapSizeHF);
        return b.intersects(r) == Ray::INTERSECTS_NONE;
    };
    return test(0, 0) || test(0, rect.height) || test(rect.width, 0) || test(rect.width, rect.height);
}

void Client::move(int x, int y) {
    if (mChoosed.size()) {
        RakNet::BitStream data;
        data.Write(ClientMessage::setMoveTarget);
        auto p = getPoint(x, y);
        data.Write(Vector2{ p.x,p.z });
        data.Write(static_cast<uint32_t>(mChoosed.size()));
        for (auto x : mChoosed)
            data.Write(x);
        mPeer->Send(&data, PacketPriority::HIGH_PRIORITY,
            PacketReliability::RELIABLE, 0, mServer, false);
    }
}

Client::Client(const std::string & server, bool& res) :
    mPeer(RakNet::RakPeerInterface::GetInstance()), mServer(server.c_str(), 23333),
    mState(false), mWeight(globalUnits.size(), 1), mCnt(0.0f) {

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
    res = mPeer->NumberOfConnections();
    mDepth = FrameBuffer::create("depth", shadowSize, shadowSize,
#ifdef ANDROID
        Texture::Format::RGB888
#else
        Texture::Format::ALPHA
#endif // ANDROID
    );

    mDepth->setDepthStencilTarget(DepthStencilTarget::create("shadow",
        DepthStencilTarget::DEPTH, shadowSize, shadowSize));
    mShadowMap = Texture::Sampler::create(mDepth->getRenderTarget()->getTexture());
    mShadowMap->setFilterMode(Texture::LINEAR, Texture::LINEAR);
    mShadowMap->setWrapMode(Texture::CLAMP, Texture::CLAMP);
    mLight = Node::create();
    mLight->setCamera(Camera::createOrthographic(1.0f, 1.0f, 1.0f, 1.0f, 2.0f));
    auto f = -Vector3::one();
    correctVector(mLight.get(), &Node::getForwardVector, f.normalize(), M_PI, M_PI, 0.0f);
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
            mMap = std::make_unique<Map>(str.C_String());
            INFO("Load map ", str.C_String());
        }
        CheckHeader(ServerMessage::go) {
            RakNet::BitStream data(packet->data, packet->bitSize >> 3, false);
            data.IgnoreBytes(1);
            mScene = Scene::create();
            mCamera =
                Camera::createPerspective(45.0f, Game::getInstance()->getAspectRatio(), 1.0f, 5000.0f);
            mScene->addNode()->setCamera(mCamera.get());
            mScene->setActiveCamera(mCamera.get());
            mMap->set(mScene->addNode("terrain"));
            auto c = mCamera->getNode();
            c->rotateX(-M_PI_2);
            Vector2 p;
            data.Read(p);
            c->setTranslation(p.x, mMap->getHeight(p.x, p.y) + 200.0f, p.y);
            mScene->addNode(mLight.get());
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
    mDuang.clear();
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
                        std::move(UnitInstance{ getUnit(u.kind), u.group, u.id, mScene.get(), false, u.pos }) });
                    i.first->second.getNode()->setRotation(u.rotation);
                    i.first->second.update(0);
                    i.first->second.setAttackTarget(u.at);
                }
                if (u.isDied)
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
        CheckHeader(ServerMessage::duang) {
            if (!enableParticle)continue;
            uint16_t size;
            data.Read(size);
            float now = Game::getAbsoluteTime();
            for (uint16_t i = 0; i < size; ++i) {
                DuangSyncInfo info;
                data.Read(info);
                auto& bullet = getBullet(info.kind);
                auto iter = mDuang.insert({ bullet.boom(),now + bullet.getBoomTime() }).first;
                mScene->addNode(iter->emitter.get());
                iter->emitter->setTranslation(info.pos);
                auto p = dynamic_cast<ParticleEmitter*>(iter->emitter->getDrawable());
                p->start();
            }
        }
    }

    if (isStop || mPeer->NumberOfConnections()!=1) {
        INFO("The game stopped.");
        stop();
        return false;
    }

    for (auto&& x : mUnits)
        x.second.update(delta);

    if (enableParticle) {
        std::vector<decltype(mDuang)::const_iterator> deferred;
        auto end = Game::getAbsoluteTime();
        for (auto i = mDuang.cbegin(); i != mDuang.cend(); ++i)
            if (i->end < end)
                deferred.emplace_back(i);
            else
                dynamic_cast<ParticleEmitter*>(i->emitter->getDrawable())->update(delta);
        for (auto&& x : deferred) {
            mScene->removeNode(x->emitter.get());
            mDuang.erase(x);
        }
    }

    //control
    mCnt += delta;
    if (mCnt > 500.0f) {

        RakNet::BitStream data;
        data.Write(ClientMessage::setAttackTarget);

        for (auto&& x : mine) {
            float md = std::numeric_limits<float>::max();
            uint32_t maxwell = 0;
            for (auto&& y : others) {
                auto dis = mUnits[x].getNode()->getTranslation().
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

        if (shadowSize > 1) {
            mDepth->bind();
            game->setViewport(gameplay::Rectangle(shadowSize, shadowSize));
            game->clear(Game::CLEAR_COLOR_DEPTH, Vector4::one(), 1.0f, 1.0f);
            auto y = mCamera->getNode()->getTranslationY();
            Matrix projection;
            auto p = getPoint(rect.width / 2, rect.height / 2);
            auto fn = (Vector3::one()*100.0f).length();
            Matrix::createOrthographic(y*2.0f, y*2.0f, 0.0f, fn*(y / 500.0f + 6.0f), &projection);
            mLight->setTranslation(p + Vector3::one()*100.0f);
            mLight->getCamera()->setProjectionMatrix(projection);
            mLightSpace = mLight->getCamera()->getViewProjectionMatrix();
            mScene->setActiveCamera(mLight->getCamera());
            for (auto&& x : mUnits)
                drawNode(x.second.getNode(), true);
            for (auto&& x : mBullets)
                drawNode(x.second.getNode(), true);
            drawNode(mScene->findNode("terrain"), true);
            mScene->setActiveCamera(mCamera.get());
            FrameBuffer::bindDefault();
        }

        game->setViewport(rect);
        mCamera->setAspectRatio(rect.width / rect.height);
        mScene->setAmbientColor(-0.8f, -0.8f, -0.8f);
        for (auto&& x : mUnits)
            if (x.second.isDied())
                drawNode(x.second.getNode());
        mScene->setAmbientColor(0.0f, 0.3f, 0.0f);
        for (auto&& x : mUnits)
            if (!x.second.isDied() && mChoosed.find(x.first) != mChoosed.cend())
                drawNode(x.second.getNode());
        mScene->setAmbientColor(0.3f, 0.0f, 0.0f);
        for (auto&& x : mUnits)
            if (!x.second.isDied() && mChoosed.find(x.first) == mChoosed.cend()
                && x.second.getGroup() == mGroup)
                drawNode(x.second.getNode());
        mScene->setAmbientColor(0.0f, 0.0f, 0.3f);
        for (auto&& x : mUnits)
            if (!x.second.isDied() && x.second.getGroup() != mGroup)
                drawNode(x.second.getNode());
        mScene->setAmbientColor(0.0f, 0.0f, 0.0f);
        for (auto&& x : mBullets)
            drawNode(x.second.getNode());
        drawNode(mScene->findNode("terrain"));

        if (enableParticle)
            for (auto&& x : mDuang)
                drawNode(x.emitter.get());

        auto rect2 = gameplay::Rectangle(game->getWidth(), game->getHeight());
        game->setViewport(rect2);

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
        x *= pos.y, y *= pos.y;
        node->translate(x, 0.0f, y);
        auto height = mMap->getHeight(pos.x + x, pos.z + y);
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
        if (pos.y + x > 10.0f && pos.y + x - 100.0f > height) {
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
        if ((mBX - x)*(mBX - x) + (mBY - y)*(mBY - y) > 256) {
            Vector3 b = getPoint(mBX, mBY), e = getPoint(x, y);
            auto x1 = b.x, y1 = b.z, x2 = e.x, y2 = e.z;
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
