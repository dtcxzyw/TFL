#pragma once
#include "common.h"
#include <RakPeer.h>
#include "Map.h"
#include "Unit.h"
#include <list>

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

    void drawNode(Node* node, bool shadow = false);
    Vector3 getPoint(int x, int y) const;
    //control
    int mX, mY,mBX,mBY;
    bool checkCamera();
    std::set<uint32_t> mChoosed;
    void move(int x, int y);

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
    Vector3 getPos(uint32_t id);
    float getHeight(int x, int z) const;
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
    bool isPlaying() const;
};

extern std::unique_ptr<Client> localClient;
