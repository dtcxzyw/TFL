#pragma once
#include "common.h"
#include <RakPeer.h>
#include "Map.h"
#include "Unit.h"

class Client final {
private:
    RakNet::RakPeerInterface* mPeer;
    RakNet::SystemAddress mServer;
    bool mState;
    uniqueRAII<Scene> mScene;
    uint32_t mRight;
    uniqueRAII<Camera> mCamera;
    uniqueRAII<SpriteBatch> mRECT;
    std::unique_ptr<Map> mMap;
    std::vector<uint16_t> mWeight;
    std::map<uint32_t, UnitInstance> mUnits;
    std::map<uint32_t, BulletInstance> mBullets;
    gameplay::Rectangle mRect;
    uint32_t mGroup;
    void drawNode(Node* node);
    Vector2 getPoint(int x, int y) const;
    float mCnt;
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
    bool hasChoosed() const;
};

extern std::unique_ptr<Client> localClient;
