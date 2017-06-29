#include "UnitController.h"
#include "Client.h"
#include "Server.h"
#include <functional>

void UnitController::setMoveTarget(Vector2 dest) { mDest = dest; }

void UnitController::setAttackTarget(uint32_t id) { mObject = id; }

uint32_t UnitController::getAttackTarget() const {
    return mObject;
}

void UnitController::isServer() { mIsServer = true; }

#define Init(name) name(info->getFloat(#name))
struct Tank final :public UnitController {
    float RST, RSC, v, time, harm, dis, count, rfac, sample;
    Vector2 last;
    Tank(const Properties* info) : Init(RST), Init(RSC), Init(v), Init(time), Init(harm), Init(dis), Init(rfac)
        , count(0.0f), sample(0.0f) {
        v /= 1000.0f;
    }
    auto dot(Node* node, Vector2 obj) {
        auto f = node->getForwardVectorWorld();
        Vector2 x{ f.x,f.z };
        x.normalize();
        return obj.dot(x);
    }

    bool update(Node* node, float delta) override {
        count += delta;
        count = std::min(count, time + 0.1f);

        auto c = node;
        auto t = node->findNode("turret");
        auto point = localClient->getPos(mObject);
        auto now = node->getTranslation();
        Vector2 np{ now.x,now.z };

        if (sample > 100.0f && !mDest.isZero()) {
            if (last.distanceSquared(np) < 10.0f*v && np.distanceSquared(mDest) < 10000.0f)
                mDest = Vector2::zero();
            last = np;
            sample = 0.0f;
        }

        constexpr auto unit = 0.0001f;
#define TEST(r,b)\
    while (cnt<=delta*RSC) {\
        r->rotateY((b));\
        auto nd =  dot(r,obj);\
        if (cd < nd)cd = nd,cnt+=unit;\
        else {\
            r->rotateY(-(b));\
            break;\
        }\
    }\


        if (mObject && !point.isZero()) {
            auto obj = point - np;
            obj.normalize();
            auto d = dot(t, obj);
            if (d > 0.999f && count > time && obj.lengthSquared() < dis) {
                if (mIsServer)localServer->attack(mObject, harm);
                count = 0.0f;
            }
            else {
                auto cd = dot(t, obj);
                if (cd < 0.999f) {
                    auto cnt = 0.0f;
                    TEST(t, unit);
                    TEST(t, -unit);
                }
            }
        }
        else {
            auto f = node->getForwardVectorWorld();
            Vector2 obj{ f.x,f.z };
            obj.normalize();
            auto cd = dot(t, obj);
            if (cd < 0.999f) {
                auto cnt = 0.0f;
                TEST(t, unit);
                TEST(t, -unit);
            }
        }

        if (!mDest.isZero()) {

            auto obj = mDest - np;
            obj.normalize();
            float fac = 1.0f;

            auto cd = dot(c, obj);
            if (cd < 0.999f) {
                auto cnt = 0.0f;
                TEST(c, unit);
                TEST(c, -unit);
#undef TEST
                fac = rfac;
            }

            auto d = dot(c, obj);
            auto f = c->getForwardVectorWorld();
            f.normalize();
            auto fd = 1.0f + f.dot(-Vector3::unitY());
            if (d > 0.7f)
                c->translateForward(std::min(delta*v*fd*fac, np.distance(mDest)))
                , sample += delta;
        }
        return true;
    }
};
#undef Init

using factoryFunction = std::function<std::unique_ptr<UnitController>(const Properties *)>;
static std::map<std::string, factoryFunction> factory;

void UnitController::initAllController() {
#define Model(name) factory[#name]=[](auto info){return std::make_unique<name>(info);}
    Model(Tank);
#undef Model
}

std::unique_ptr<UnitController> UnitController::newInstance(const Properties * info) {
    return factory[info->getString("type")](info);
}
