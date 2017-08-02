#pragma once
#include "common.h"
#include <RakPeer.h>
#include "Map.h"
#include "Unit.h"
#include <list>
#include "Audio.h"
#include "Message.h"

struct DuangInfo final {
    uniqueRAII<Node> emitter;
    float end;
    bool operator<(const DuangInfo& rhs) const {
        return emitter.get() < rhs.emitter.get();
    }
};

class Client final:RenderState::AutoBindingResolver {
private:
    RakNet::RakPeerInterface* mPeer;
    RakNet::SystemAddress mServer;
    bool mState;
    uniqueRAII<Scene> mScene;
    uint32_t mRight;
    uniqueRAII<Camera> mCamera;
    uniqueRAII<SpriteBatch> mRECT,mMiniMap,mMiniMapUnit,mHot;
    std::list<Vector2> mHotPoint;
    std::unique_ptr<Map> mMap;
    std::vector<uint16_t> mWeight;
    std::map<uint32_t, UnitInstance> mUnits;
    std::map<uint32_t, BulletInstance> mBullets;
    uniqueRAII<Node> mFlagModel;
    uint32_t mGroup;
    float mSpeed;
    Vector3 mCameraPos;
    uniqueRAII<Form> mStateInfo;
    std::map<uint32_t, uint32_t> mLoadSize;
    std::vector<ProducingSyncInfo> mProducingState;
    AudioManager mAudio;

    //Effects
    std::set<DuangInfo> mDuang;
    uniqueRAII<FrameBuffer> mDepth;
    Matrix mLightSpace;
    uniqueRAII<Texture::Sampler> mShadowMap;
    bool resolveAutoBinding(const char* autoBinding, Node* node,
        MaterialParameter* parameter) override;
    Matrix getMat() const;
    uniqueRAII<Node> mLight;
    uniqueRAII<Node> mSky;
    uniqueRAII<Node> mWaterPlane;
    uniqueRAII<FrameBuffer> mScreenBuffer;
    uniqueRAII<Texture::Sampler> mScreenMap;
    Vector2 mBlurPixel;
    Vector2 getPixel() const;
    const Texture::Sampler* getScreen() const;

    void drawNode(Node* node, const char* effect="shadow");

    Vector3 getPoint(int x, int y) const;
    //control
    int mX, mY,mBX,mBY;
    double mLast;
    bool checkCamera();
    std::set<uint32_t> mChoosed,mLastChoosed;
    void move(int x, int y);
    uint32_t mFollower;

public:
    Client(const std::string& server,bool& res);
    ~Client();
    void changeGroup(uint8_t group);
    enum class WaitResult {
        None,Disconnected,Go
    };
    WaitResult wait();
    void stop();
    bool update(float delta);
    void render();
    Vector3 getPos(uint32_t& id);
    float getHeight(int x, int z) const;
    AudioManager& getAudio();
    bool isMine(uint32_t id) const;
    //UI
    void setViewport(uint32_t right);
    void changeWeight(const std::string& name,uint16_t weight);
    uint16_t getWeight(const std::string& name) const;
    uint16_t getUnitNum() const;
    //events
    void moveEvent(float x, float y);
    void scaleEvent(float x);
    void mousePos(int x, int y);
    void beginPoint(int x, int y);
    void endPoint(int x, int y);
    void cancel();
    void follow();
    bool isPlaying() const;
    void recreate(uint32_t width, uint32_t height);
};

extern std::unique_ptr<Client> localClient;
