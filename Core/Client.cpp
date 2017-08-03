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
    else if (CMP("SCREEN"))
        parameter->bindValue(this, &Client::getScreen);
    else if (CMP("PIXEL"))
        parameter->bindValue(this, &Client::getPixel);
    else if (CMP("LIGHT_DIRECTION"))
        parameter->bindValue(mLight.get(), &Node::getForwardVectorView);
    else return false;
#undef CMP
    return true;
}

Matrix Client::getMat() const {
    return mLightSpace;
}

Vector2 Client::getPixel() const {
    return mBlurPixel;
}

const Texture::Sampler * Client::getScreen() const {
    return mScreenMap.get();
}

void Client::drawNode(Node * node, const char* effect) {
    if (node->isEnabled() && node->getDrawable()) {
        auto bs = node->getBoundingSphere();
        if (*effect == 'w')bs.center.y = -bs.center.y;
        if (bs.intersects(mScene->getActiveCamera()->getFrustum())) {
            auto m = dynamic_cast<Model*>(node->getDrawable());
            auto t = dynamic_cast<Terrain*>(node->getDrawable());

            if (m) m->getMaterial()->setTechnique(effect);
            else if (t) {
                for (unsigned int i = 0; i < t->getPatchCount(); ++i)
                    t->getPatch(i)->getMaterial(0)->setTechnique(effect);
            }

            if (effect[0] == 's' || m || t)
                node->getDrawable()->draw();
        }
    }

    for (auto i = node->getFirstChild(); i; i = i->getNextSibling())
        drawNode(i, effect);
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
    if (mFollower)return false;
    auto game = Game::getInstance();
    auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
    mCamera->setAspectRatio(rect.width / rect.height);
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
        uint32_t ix = static_cast<uint32_t>(p.x + mapSizeHF) * 16 / mapSize;
        uint32_t iy = static_cast<uint32_t>(p.y + mapSizeHF) * 16 / mapSize;
        mAudio.voice(CodeType::to, { *mChoosed.begin(), ix * 16 + iy });
    }
}

Client::Client(const std::string & server, bool& res) :
    mPeer(RakNet::RakPeerInterface::GetInstance()), mServer(server.c_str(), 23333),
    mState(false), mWeight(globalUnits.size(), 1), mSpeed(1.0f), mRight(0), mFollower(0) {

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

    res = mPeer->NumberOfConnections();
    if (!res)return;

    mRECT = SpriteBatch::create("res/common/rect.png");
    mMiniMapUnit = SpriteBatch::create("res/common/white.png");
    mHot = SpriteBatch::create("res/common/hot.png");

    mDepth = FrameBuffer::create("depth", shadowSize, shadowSize, Texture::Format::RGBA8888);

    uniqueRAII<DepthStencilTarget> shadow = DepthStencilTarget::create("shadow",
        DepthStencilTarget::DEPTH, shadowSize, shadowSize);
    mDepth->setDepthStencilTarget(shadow.get());

    uniqueRAII<Texture> texture = mDepth->getRenderTarget()->getTexture();
    mShadowMap = Texture::Sampler::create(texture.get());
    mShadowMap->setFilterMode(Texture::LINEAR, Texture::LINEAR);
    mShadowMap->setWrapMode(Texture::CLAMP, Texture::CLAMP);
    mLight = Node::create();

    uniqueRAII<Camera> light = Camera::createOrthographic(1.0f, 1.0f, 1.0f, 1.0f, 2.0f);
    mLight->setCamera(light.get());

    Vector3 init{ 0.7f,-1.0f,0.7f };
    correctVector(mLight.get(), &Node::getForwardVector, init.normalize(), M_PI, M_PI, 0.0f);
    correctVector(mLight.get(), &Node::getUpVector, Vector3::unitY(), 0.0f, 0.0f, M_PI);

    if (reflection > 0.0f) {
        auto game = Game::getInstance();
        recreate(game->getWidth(), game->getHeight());
    }

    {
        mStateInfo = Form::create("label", Theme::getDefault()->getStyle("Label"));
        mStateInfo->setAutoSize(Control::AutoSize::AUTO_SIZE_BOTH);
        mStateInfo->setConsumeInputEvents(false);
        mStateInfo->setAlignment(Control::Alignment::ALIGN_BOTTOM_HCENTER);
        uniqueRAII<Label> info = Label::create("info");
        mStateInfo->addControl(info.get());
        info->setAutoSize(Control::AutoSize::AUTO_SIZE_BOTH);
        info->setAlignment(Control::Alignment::ALIGN_BOTTOM_HCENTER);
        info->setConsumeInputEvents(false);
    }
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
        RakNet::BitStream data(packet->data, packet->length, false);
        data.IgnoreBytes(1);
        CheckBegin;
        CheckHeader(ID_DISCONNECTION_NOTIFICATION) {
            mPeer->DeallocatePacket(packet);
            return WaitResult::Disconnected;
        }
        CheckHeader(ServerMessage::info) {
            uint64_t key;
            data.Read(key);
            if (key != pakKey) {
                INFO("The pakKey of server is not equal yours.");
                mPeer->DeallocatePacket(packet);
                return WaitResult::Disconnected;
            }
            RakNet::RakString str;
            data.Read(str);
            INFO("Loading map ", str.C_String());
            mMap = std::make_unique<Map>(str.C_String());
            mMiniMap = SpriteBatch::create(("res/maps/"s + str.C_String() + "/view.png").c_str());
            uniqueRAII<Scene> sky = Scene::load(("res/maps/"s + str.C_String() + "/sky.scene").c_str());
            mSky = sky->findNode("sky")->clone();
            mScene->addNode(mSky.get());
        }
        CheckHeader(ServerMessage::changeSpeed) {
            data.Read(mSpeed);
        }
        CheckHeader(ServerMessage::go) {

            //lazy load
            if (!mFlagModel) {
                uniqueRAII<Scene> model = Scene::load("res/common/common.scene");
                mFlagModel = model->findNode("key")->clone();
                mWaterPlane = model->findNode("plane")->clone();
                mWaterPlane->scale(1.4f*mapSizeHF / mWaterPlane->getBoundingSphere().radius);
            }

            mScene = Scene::create();
            mCamera =
                Camera::createPerspective(45.0f, Game::getInstance()->getAspectRatio(), 1.0f, 10000.0f);
            mScene->addNode()->setCamera(mCamera.get());
            mScene->setActiveCamera(mCamera.get());
            mScene->addNode(mFlagModel.get());
            mMap->set(mScene->addNode("terrain"));
            auto c = mCamera->getNode();
            c->rotateX(-M_PI_2);
            Vector2 p;
            data.Read(p);
            mCameraPos = { p.x, mMap->getHeight(p.x, p.y) + 200.0f, p.y };
            c->setTranslation(mCameraPos);
            mScene->addNode(mLight.get());
            mX = mY = mBX = mBY = 0;
            std::fill(mWeight.begin(), mWeight.end(), 1);
            mScene->addNode(mSky.get());
            mScene->addNode(mWaterPlane.get());
            mAudio.setScene(mScene.get());

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
    mAudio.clear();
    mScene.reset();
    mCamera.reset();
    mUnits.clear();
    mDuang.clear();
    mChoosed.clear();
    mHotPoint.clear();
    mProducingState.clear();
    RakNet::BitStream stream;
    stream.Write(ClientMessage::exit);
    mPeer->Send(&stream, PacketPriority::IMMEDIATE_PRIORITY, PacketReliability::RELIABLE_ORDERED
        , 0, mServer, false);
}

bool Client::update(float delta) {

    if (mFollower) {
        auto u = mUnits.find(mFollower);
        if (u != mUnits.cend() && !u->second.isDied()) {
            auto node = u->second.getNode();
            auto bs = node->getBoundingSphere();
            auto p = node->getTranslation();
            auto back = node->getBackVector().normalize();
            auto up = node->getUpVector().normalize();
            auto len = bs.radius*3.0f;
            auto pos = p + (back+up)*len;
            pos.y = std::max(getHeight(pos.x, pos.z) + 10.0f, pos.y);
            mCamera->getNode()->translateSmooth(pos, delta, 500.0f);
            auto off = p - pos;
            auto isNear = mCamera->getNode()->getTranslation().distanceSquared(pos) < 10000.0f;
            constexpr auto RV = 0.00001f;
            auto r = isNear ? RV*delta : M_PI;
            correctVector(mCamera->getNode(), &Node::getForwardVector,
                off.normalize(), r, r, 0.0f);
            correctVector(mCamera->getNode(), &Node::getUpVector,
                node->getUpVector().normalize(), 0.0f, 0.0f, r);
        }
        else mFollower = 0;
    }
    else {
        if (checkCamera())
            mCamera->getNode()->setTranslation(mCameraPos);
        else
            mCameraPos = mCamera->getNode()->getTranslation();
        correctVector(mCamera->getNode(), &Node::getForwardVector,
            -Vector3::unitY(), M_PI, M_PI, 0.0f);
        correctVector(mCamera->getNode(), &Node::getUpVector, -Vector3::unitZ(), 0.0f, 0.0f, M_PI);
    }

    {
        mLight->rotateX(delta*0.000001f);
        auto dir = mLight->getForwardVector().normalize();
        auto atNight = dir.y >= 0.0f || dir.dot(-Vector3::unitY()) < 0.55f;
        if (atNight)
            mLight->rotateZ(M_PI);
    }

#ifdef WIN32
    if (!(mBX || mBY)) {
        auto game = Game::getInstance();
        auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
        if (mX<rect.width - miniMapSize || mY>miniMapSize) {
            float vx = (mX - rect.width / 2) / rect.width;
            float vy = (mY - rect.height / 2) / rect.height;
            constexpr float unit = 0.001f;
            float m = unit*delta;
            float mx = (std::abs(vx) <= 0.5f && std::abs(vx) >= 0.4f) ? (vx > 0.0f ? m : -m) : 0.0f;
            float my = (std::abs(vy) <= 0.5f && std::abs(vy) >= 0.4f) ? (vy > 0.0f ? m : -m) : 0.0f;
            moveEvent(mx, 0.0f);
            moveEvent(0.0f, my);
        }
    }
#endif // WIN32

    delta *= mSpeed;

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
            mLoadSize.clear();
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
                    mUnits[u.id].getNode()->translateSmooth(u.pos, delta, 100.0f);
                    mUnits[u.id].getNode()->setRotation(u.rotation);
                    mUnits[u.id].setAttackTarget(u.at);
                    old.erase(oi);
                }
                else {
                    auto i = mUnits.emplace(u.id,
                        UnitInstance{ getUnit(u.kind), u.group, u.id, mScene.get(), false, u.pos });
                    i.first->second.getNode()->setRotation(u.rotation);
                    i.first->second.update(0);
                    i.first->second.setAttackTarget(u.at);
                    if (u.group != mGroup && u.HP > 0.0f)
                        mAudio.voice(CodeType::found, { u.id });
                }
                mUnits[u.id].setHP(u.HP);
                if (u.group != mGroup && u.HP <= 0.0f)
                    mAudio.voice(CodeType::success, { u.id });

                if (u.size)
                    mLoadSize[u.id] = u.size;
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
        CheckHeader(ServerMessage::updateState) {
            for (auto& x : mWeight)
                data.Read(x);
            ProducingSyncInfo info;
            mProducingState.clear();
            while (data.Read(info))
                mProducingState.emplace_back(info);
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
                mHotPoint.push_back({ info.pos.x,info.pos.z });
                if (mHotPoint.size() > 4)
                    mHotPoint.pop_front();
                mAudio.play(AudioType::boom, info.pos);
            }
        }
    }

    if (isStop || mPeer->GetConnectionState(mServer) != RakNet::IS_CONNECTED) {
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

        for (auto&& x : mBullets)
            x.second.updateClient(delta);
    }
    {
        std::set<uint32_t> choosed;
        for (auto&& x : mChoosed)
            if (mUnits.find(x) != mUnits.cend() && !mUnits[x].isDied())
                choosed.insert(x);
        choosed.swap(mChoosed);
    }

    {
        auto label = dynamic_cast<Label*>(mStateInfo->getControl(0U));
        std::string str;
        {
            auto pos = mCamera->getNode()->getTranslation();
            for (auto&& x : mProducingState) {
                auto&& u = getUnit(x.kind);
                float t = u.getTime() / 1000.0f / mSpeed;
                uint32_t p = x.time*100.0f / t;
                str += "\nThe key " + to_string(static_cast<uint16_t>(x.key) + 1) + " is producing " +
                    u.getName() + ' ' + to_string(100U - p)
                    + "% (" + to_string(static_cast<uint32_t>(x.time))
                    + '/' + to_string(static_cast<uint32_t>(t)) + ")";
            }
        }

        if (mChoosed.size()) {
            str += "\nChoosed " + to_string(mChoosed.size());
            float cnt = 0.0f, tot = 0.0f;
            for (auto&& x : mChoosed) {
                auto& u = mUnits[x];
                cnt += std::max(u.getHP(), 0.0f);
                tot += u.getKind().getHP();
            }
            str += "\nHP " + to_string(static_cast<int32_t>(cnt))
                + "/" + to_string(static_cast<int32_t>(tot));
            if (mChoosed.size() == 1) {
                auto& u = mUnits[*mChoosed.begin()];
                if (u.getKind().getLoading())
                    str += "\nLoad " + to_string(mLoadSize[u.getID()])
                    + "/" + to_string(u.getKind().getLoading());
            }
        }
        label->setText(str.c_str());
        mStateInfo->update(delta);
    }

    mAudio.update();

    return true;
}

void Client::render() {
    if (mState) {
        auto game = Game::getInstance();
        auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());

        if (shadowSize > 1) {
            constexpr auto depth = "depth";
            mDepth->bind();
            game->setViewport(gameplay::Rectangle(shadowSize, shadowSize));
            game->clear(Game::CLEAR_COLOR_DEPTH, Vector4::one(), 1.0f, 0);
            auto y = mCamera->getNode()->getTranslationY();
            Matrix projection;
            auto p = getPoint(rect.width / 2, rect.height / 2);
            Matrix::createOrthographic(y*2.0f, y*2.0f, 0.0f, 170.0f*(y / 500.0f + 6.0f), &projection);
            mLight->setTranslation(p + mLight->getBackVector()*100.0f);
            mLight->getCamera()->setProjectionMatrix(projection);
            mLightSpace = mLight->getCamera()->getViewProjectionMatrix();
            mScene->setActiveCamera(mLight->getCamera());
            for (auto&& x : mUnits)
                drawNode(x.second.getNode(), depth);
            for (auto&& x : mBullets)
                drawNode(x.second.getNode(), depth);

            for (auto&& p : mMap->getKey()) {
                mFlagModel->setTranslation(p.x, mMap->getHeight(p.x, p.y), p.y);
                drawNode(mFlagModel.get(), depth);
            }

            drawNode(mScene->findNode("terrain"), depth);

            mScene->setActiveCamera(mCamera.get());
        }

        if (reflection > 0.0f) mScreenBuffer->bind();
        else FrameBuffer::bindDefault();

        game->clear(Game::CLEAR_COLOR_DEPTH_STENCIL, Vector4::zero(), 1.0f, 0);

        game->setViewport(rect);
        mCamera->setAspectRatio(rect.width / rect.height);

        mScene->setAmbientColor(0.3f, 0.0f, 0.0f);
        std::vector<Node*> list;
        for (auto&& x : mUnits)
            if (!x.second.isDied() && mChoosed.find(x.first) != mChoosed.cend()) {
                list.emplace_back(x.second.getNode());
                drawNode(list.back(), "choosedShadow");
            }

        mScene->setAmbientColor(-0.8f, -0.8f, -0.8f);
        for (auto&& x : mUnits)
            if (x.second.isDied())
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

        for (auto&& p : mMap->getKey()) {
            mFlagModel->setTranslation(p.x, mMap->getHeight(p.x, p.y), p.y);
            drawNode(mFlagModel.get());
        }

        drawNode(mScene->findNode("terrain"));

        drawNode(mSky.get());

        for (auto&& x : list) {
            auto s = x->getScale();
            x->scale(1.2f);
            drawNode(x, "choosed");
            x->setScale(s);
        }

        glBlendColor(1.0f, 1.0f, 1.0f, waterAlpha);
        drawNode(mWaterPlane.get());

        if (reflection > 0.0f) {
            mScreenBuffer->bind(GL_READ_FRAMEBUFFER);
            FrameBuffer::bindDefault(GL_DRAW_FRAMEBUFFER);

            glBlitFramebuffer(0, 0, rect.width, rect.height
                , 0, 0, rect.width, rect.height
                , GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, GL_NEAREST);
            mScreenBuffer->bind();

            glBlendColor(1.0f, 1.0f, 1.0f, reflection);

            game->clear(Game::CLEAR_DEPTH, {}, 1.0f, 0);

            constexpr auto water = "water";

            mScene->setAmbientColor(-0.8f, -0.8f, -0.8f);
            for (auto&& x : mUnits)
                if (x.second.isDied())
                    drawNode(x.second.getNode(), water);

            mScene->setAmbientColor(0.3f, 0.0f, 0.0f);
            for (auto&& x : mUnits)
                if (!x.second.isDied() && x.second.getGroup() == mGroup)
                    drawNode(x.second.getNode(), water);

            mScene->setAmbientColor(0.0f, 0.0f, 0.3f);
            for (auto&& x : mUnits)
                if (!x.second.isDied() && x.second.getGroup() != mGroup)
                    drawNode(x.second.getNode());

            mScene->setAmbientColor(0.0f, 0.0f, 0.0f);

            for (auto&& x : mBullets)
                drawNode(x.second.getNode(), water);

            drawNode(mScene->findNode("terrain"), water);

            drawNode(mSky.get(), water);

            //Postprocessing
            FrameBuffer::bindDefault();
            game->clear(Game::CLEAR_COLOR, Vector4::zero(), 1.0f, 0);
            mBlurPixel = { 1.0f / rect.width,0.0f };
            drawNode(mWaterPlane.get(), "blur");
            drawNode(mWaterPlane.get(), "none");
            mScreenBuffer->bind(GL_DRAW_FRAMEBUFFER);
            FrameBuffer::bindDefault(GL_READ_FRAMEBUFFER);
            glBlitFramebuffer(0, 0, rect.width, rect.height
                , 0, 0, rect.width, rect.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
            FrameBuffer::bindDefault();
            mBlurPixel = { 0.0f,1.0f / rect.height };
            drawNode(mWaterPlane.get(), "blur");
            drawNode(mWaterPlane.get(), "none");
        }

        for (auto&& x : mBullets)
            drawNode(x.second.getNode());

        if (enableParticle)
            for (auto&& x : mDuang)
                drawNode(x.emitter.get());

        auto rect2 = gameplay::Rectangle(game->getWidth(), game->getHeight());
        game->setViewport(rect2);

        if (mBX&&mBY) {
            float x1 = mX, y1 = mY, x2 = mBX, y2 = mBY;
            if (x1 > x2)std::swap(x1, x2);
            if (y1 > y2)std::swap(y1, y2);
            gameplay::Rectangle range{ x1,y1,x2 - x1,y2 - y1 };
            mRECT->start();
            mRECT->draw(range, { 32,32 });
            mRECT->finish();
        }

        if (miniMapSize) {

            {
                gameplay::Rectangle range{ rect.width - miniMapSize,0.0f,
                    miniMapSize*1.0f,miniMapSize*1.0f };
                mMiniMap->start();
                auto texture = mMiniMap->getSampler()->getTexture();
                mMiniMap->draw(range, { texture->getWidth()*1.0f,texture->getHeight()*1.0f });
                mMiniMap->finish();
            }


            static const Vector4 red = { 1.0f,0.0f,0.0f,1.0f };
            static const Vector4 blue = { 0.0f,0.0f,1.0f,1.0f };

            auto fac = miniMapSize / mapSizeF;
            Vector2 base{ rect.width - miniMapSize / 2.0f,miniMapSize / 2.0f };

            if (mHotPoint.size()) {
                mHot->start();
                for (auto&& x : mHotPoint) {
                    auto dp = base + x*fac;
                    gameplay::Rectangle range{ dp.x - miniMapSize / 32.0f,dp.y - miniMapSize / 32.0f
                        ,miniMapSize / 16.0f,miniMapSize / 16.0f };
                    mHot->draw(range, { 32,32 });
                }
                mHot->finish();
            }

            mMiniMapUnit->start();
            for (auto&& p : mMap->getKey()) {
                auto dp = base + p*fac;
                gameplay::Rectangle range{ dp.x - miniMapSize / 64.0f,dp.y - miniMapSize / 64.0f
                    ,miniMapSize / 32.0f,miniMapSize / 32.0f };
                mMiniMapUnit->draw(range, { 1,1 });
            }

            for (auto&& x : mUnits)
                if (!x.second.isDied()) {
                    auto p = x.second.getRoughPos();
                    auto dp = base + Vector2(p.x, p.z)*fac;
                    gameplay::Rectangle range{ dp.x - miniMapSize / 128.0f,dp.y - miniMapSize / 128.0f
                        ,miniMapSize / 64.0f,miniMapSize / 64.0f };
                    mMiniMapUnit->draw(range, { 1,1 }, x.second.getGroup() == mGroup ? red : blue);
                }
            mMiniMapUnit->finish();

            if (!mFollower) {
                auto p1 = getPoint(0, 0)*fac,
                    p2 = getPoint(rect.width, rect.height)*fac;
                float x1 = p1.x, y1 = p1.z, x2 = p2.x, y2 = p2.z;
                if (x1 > x2)std::swap(x1, x2);
                if (y1 > y2)std::swap(y1, y2);
                gameplay::Rectangle range{ base.x + x1,base.y + y1
                    ,std::max(x2 - x1,16.0f),std::max(y2 - y1,16.0f) };
                mRECT->start();
                mRECT->draw(range, { 32,32 });
                mRECT->finish();
            }
        }

        mStateInfo->draw();
    }
}

Vector3 Client::getPos(uint32_t& id) {
    auto i = mUnits.find(id);
    if (i == mUnits.cend() || i->second.isDied()) {
        id = 0;
        return Vector3::zero();
    }
    return i->second.getNode()->getTranslation();
}

float Client::getHeight(int x, int z) const {
    return mMap->getHeight(x, z);
}

void Client::setViewport(uint32_t right) {
    if (mRight != right) {
        mRight = right;
        auto game = Game::getInstance();
        recreate(game->getWidth(), game->getHeight());
    }
}

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
        auto old = node->getTranslation();
        node->translateForward(-x);
        auto p = node->getTranslation();
        if (node->getTranslationY() <= std::max(getHeight(p.x, p.z), 0.0f) + 100.0f || checkCamera())
            node->setTranslation(old);
    }
}

void Client::mousePos(int x, int y) {
    mX = x, mY = y;
}

void Client::beginPoint(int x, int y) {
    if (mState) {
        auto right = Game::getInstance()->getWidth() - mRight;
        if (y <= miniMapSize && x <= right && x >= right - miniMapSize) {
            auto fac = mapSizeF / miniMapSize;
            Vector2 pos{ fac*(x - right + miniMapSize) - mapSizeHF,fac*y - mapSizeHF };
            if (mChoosed.empty()) {
                constexpr auto offset = 200.0f;
                pos.x = std::min(mapSizeHF - offset, pos.x);
                pos.x = std::max(offset - mapSizeHF, pos.x);
                pos.y = std::min(mapSizeHF - offset, pos.y);
                pos.y = std::max(offset - mapSizeHF, pos.y);

                auto node = mCamera->getNode();
                auto old = node->getTranslation();
                node->setTranslation(pos.x, mMap->getHeight(pos.x, pos.y) +
                    std::max(100.0f, old.y - mMap->getHeight(old.x, old.z)), pos.y);
                while (checkCamera())
                    node->translateY(-50.0f);
            }
            else {
                RakNet::BitStream data;
                data.Write(ClientMessage::setMoveTarget);
                data.Write(pos);
                data.Write(static_cast<uint32_t>(mChoosed.size()));
                for (auto x : mChoosed)
                    data.Write(x);
                mPeer->Send(&data, PacketPriority::HIGH_PRIORITY,
                    PacketReliability::RELIABLE, 0, mServer, false);
                uint32_t ix = static_cast<uint32_t>(pos.x + mapSizeHF) * 16 / mapSize;
                uint32_t iy = static_cast<uint32_t>(pos.y + mapSizeHF) * 16 / mapSize;
                mAudio.voice(CodeType::to, { *mChoosed.begin(), ix * 16 + iy });
            }
        }
        else mBX = x, mBY = y;
    }
}

void Client::endPoint(int x, int y) {

    bool useFeature = Game::getAbsoluteTime() - mLast < 200.0f;
    mLast = Game::getAbsoluteTime();

    auto toNDC = [](auto&& u, auto rect, auto offset) {
        auto MVP = u.second.getNode()->getWorldViewProjectionMatrix();
        auto NDC = MVP*offset;
        NDC.x /= NDC.w; NDC.y /= NDC.w; NDC.z /= NDC.w;
        NDC.x /= 2.0f, NDC.y /= 2.0f; NDC.z /= 2.0f;
        NDC.x += 0.5f; NDC.y += 0.5f; NDC.z += 0.5f;
        NDC.y = 1.0f - NDC.y;
        NDC.x *= rect.width;
        NDC.y *= rect.height;
        return NDC;
    };

    if (mState && mBX) {
        auto game = Game::getInstance();
        auto rect = gameplay::Rectangle(game->getWidth() - mRight, game->getHeight());
        mCamera->setAspectRatio(rect.width / rect.height);

        if ((mBX - x)*(mBX - x) + (mBY - y)*(mBY - y) > 256) {
            std::set<uint32_t> choosed, mine;
            if (mBX > x)std::swap(mBX, x);
            if (mBY > y)std::swap(mBY, y);
            for (auto&& u : mUnits)
                if (!u.second.isDied()) {
                    auto NDC = toNDC(u, rect, Vector4::unitW());
                    if (NDC.x >= mBX && NDC.x <= x && NDC.y >= mBY && NDC.y <= y) {
                        if (u.second.getGroup() != mGroup)choosed.insert(u.first);
                        else mine.insert(u.first);
                    }
                }

            if (choosed.size() && (mine.size() || mChoosed.size())) {
                for (auto&& x : mine)
                    mChoosed.insert(x);

                RakNet::BitStream data;
                data.Write(ClientMessage::setAttackTarget);
                for (auto&& x : mChoosed) {
                    if (mUnits.find(x) == mUnits.cend())continue;
                    auto pos = mUnits[x].getRoughPos();
                    auto md = std::numeric_limits<float>::max();
                    uint32_t maxwell = 0;
                    for (auto&& y : choosed) {
                        auto p = mUnits[y].getRoughPos();
                        auto dis = p.distanceSquared(pos);
                        if (dis < md)md = dis, maxwell = y;
                    }
                    data.Write(x);
                    data.Write(maxwell);
                }
                mPeer->Send(&data, PacketPriority::HIGH_PRIORITY,
                    PacketReliability::RELIABLE_ORDERED, 0, mServer, false);
                mAudio.voice(CodeType::attack, { *mChoosed.begin(),*choosed.begin() });
            }
            else mChoosed.swap(mine);
        }
        else {
            uint32_t choosed = 0;
            float currectDepth = 1.0f;
            auto dis = [](auto x, auto y) {
                return x*x + y*y;
            };
            for (auto&& u : mUnits)
                if (!u.second.isDied()) {
                    auto NDC = toNDC(u, rect, Vector4::unitW());
                    auto right = toNDC(u, rect, Vector4{
                        u.second.getKind().getRadius() / u.second.getNode()->getScaleX(),0.0f,0.0f,1.0f });
                    auto radius = dis(right.x - NDC.x, right.y - NDC.y);
                    if (NDC.z < currectDepth && dis(NDC.x - x, NDC.y - y) <= radius) {
                        currectDepth = NDC.z;
                        choosed = u.first;
                    }
                }

            if (choosed) {
                if (mUnits[choosed].getGroup() == mGroup) {
                    if (useFeature && mUnits[choosed].getKind().getLoading()) {
                        RakNet::BitStream data;
                        if (mLastChoosed.empty() ||
                            (mLastChoosed.size() == 1 && *mLastChoosed.begin() == choosed)) {
                            data.Write(ClientMessage::release);
                            data.Write(choosed);
                        }
                        else {
                            data.Write(ClientMessage::load);
                            data.Write(choosed);
                            for (auto&& x : mLastChoosed)
                                data.Write(x);
                        }
                        mPeer->Send(&data, PacketPriority::HIGH_PRIORITY
                            , PacketReliability::RELIABLE_ORDERED, 0, mServer, false);
                    }
                    else mLastChoosed.swap(mChoosed);
                    mChoosed.clear();
                    mChoosed.insert(choosed);
                }
                else {
                    RakNet::BitStream data;
                    data.Write(ClientMessage::setAttackTarget);
                    for (auto&& x : mChoosed) {
                        if (mUnits.find(x) == mUnits.cend())continue;
                        data.Write(x);
                        data.Write(choosed);
                    }
                    mPeer->Send(&data, PacketPriority::HIGH_PRIORITY,
                        PacketReliability::RELIABLE_ORDERED, 0, mServer, false);
                    mAudio.voice(CodeType::attack, { *mChoosed.begin(),choosed });
                }
            }
            else move(x, y);
        }

        if (mFollower && mUnits.find(mFollower) != mUnits.cend() && !mUnits[mFollower].isDied())
            mChoosed.insert(mFollower);
    }
    mBX = 0, mBY = 0;
}

void Client::cancel() {
    mChoosed.clear();
}

void Client::follow() {
    if (mState && mChoosed.size() == 1 && !mFollower)
        mFollower = *mChoosed.begin();
    else
        mFollower = 0;
}

bool Client::isPlaying() const {
    return mState;
}

void Client::recreate(uint32_t width, uint32_t height) {
    if (reflection == 0.0f)return;
    width -= mRight;
    mScreenBuffer = FrameBuffer::create("screen", width, height, Texture::RGBA8888);
    uniqueRAII<DepthStencilTarget> DS = DepthStencilTarget::create("water",
        DepthStencilTarget::DEPTH_STENCIL, width, height);
    mScreenBuffer->setDepthStencilTarget(DS.get());

    mScreenMap = Texture::Sampler::create(mScreenBuffer->getRenderTarget()->getTexture());
    mScreenMap->setWrapMode(Texture::Wrap::CLAMP, Texture::Wrap::CLAMP);
    mScreenMap->setFilterMode(Texture::Filter::LINEAR, Texture::Filter::LINEAR);
}

AudioManager & Client::getAudio() {
    return mAudio;
}

bool Client::isMine(uint32_t id) const {
    auto i = mUnits.find(id);
    return i != mUnits.cend() && i->second.getGroup() == mGroup;
}
