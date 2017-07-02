#include "UnitController.h"
#include "Client.h"
#include "Server.h"
#include <functional>
#include <array>

void UnitController::setMoveTarget(Vector2 dest) { mDest = dest; }

void UnitController::setAttackTarget(uint32_t id) { mObject = id; }

uint32_t UnitController::getAttackTarget() const {
    return mObject;
}

void UnitController::isServer() { mIsServer = true; }

void correct(UnitInstance& instance,float delta,float& cnt) {
    auto node = instance.getNode();
    auto kind = &instance.getKind();
    auto p = node->getTranslation();
    auto b = p + kind->getOffset();
    auto h = localClient->getHeight(b.x, b.z);
    if (b.y  < h) {
        node->setTranslationY(h - kind->getOffset().y);
        auto d = kind->getPlane();

        std::array<Vector2, 4> base =
        { Vector2{ b.x + d.x,b.z + d.y },
            Vector2{ b.x + d.x,b.z - d.y } ,
            Vector2{ b.x - d.x,b.z + d.y } ,
            Vector2{ b.x - d.x,b.z - d.y } };
        std::array<Vector3, 4> sample;
        size_t idx = 0;
        for (auto&&x : base) {
            sample[idx] = Vector3{ x.x,localClient->getHeight(x.x,x.y),x.y };
            ++idx;
        }

        Vector3 mean;
        for (size_t i = 0; i < 4; ++i) {
            auto a = sample[(i + 3) % 4] - sample[i];
            a.normalize();
            auto b = sample[(i + 1) % 4] - sample[i];
            b.normalize();
            Vector3 up;
            Vector3::cross(a, b, &up);
            if (up.y < 0)up.negate();
            up.normalize();
            mean += up / 4.0f;
        }

        mean.normalize();
        auto dot = [&] {
            auto u = node->getUpVectorWorld();
            u.normalize();
            return u.dot(mean);
        };

        correctVector(node, &Node::getUpVector, mean, M_PI_4, false, M_PI_4);
        cnt = 0.0f;
    }
    else {
        node->translateY(-49.0f*(2.0f*cnt+delta)*delta/1e6f);
        cnt += delta;
    }
}

#define Init(name) name(info->getFloat(#name))
struct Tank final :public UnitController {
    float RST, RSC, v, time, harm, dis, count, rfac, sample,range,offset,speed,fcnt;
    Vector2 last;
    std::string bullet;
    Tank(const Properties* info) : Init(RST), Init(RSC), Init(v), Init(time), Init(harm), Init(dis), Init(rfac),
        Init(range) , count(0.0f), sample(0.0f),bullet(info->getString("bullet")),Init(offset),Init(speed) {
        v /= 1000.0f;
        //speed /= 1000.0f;
    }
    auto dot(Node* node, Vector2 obj) {
        auto f = node->getForwardVectorWorld();
        Vector2 x{ f.x,f.z };
        x.normalize();
        return obj.dot(x);
    }

    bool update(UnitInstance& instance, float delta) override {
        if (instance.isDied()) {
            correct(instance,delta,fcnt);
            return false;
        }
        auto node = instance.getNode();
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

        if (mObject && !point.isZero()) {
            auto obj = Vector2{ point.x,point.z } -np;
            obj.normalize();
            auto d = dot(t, obj);
            if (d > 0.999f && count > time && obj.lengthSquared() < dis) {
                if (mIsServer) {
                    auto f = t->getForwardVectorWorld();
                    f.normalize();
                    localServer->newBullet(BulletInstance(bullet, t->getTranslationWorld() +
                        offset*f, point, speed, harm, range,instance.getGroup()));
                }
                count = 0.0f;
            }
            else {
                auto dest = point - now;
                dest.normalize();
                correctVector(t, &Node::getForwardVectorWorld,dest, 0.0f, RST*delta, 0.0f);
            }
        }
        else {
            mObject = 0;
            auto f = node->getForwardVectorWorld();
            f.normalize();
            correctVector(t, &Node::getForwardVectorWorld,f , 0.0f, RST*delta, 0.0f);
        }

        if (!mDest.isZero()) {

            auto obj = mDest - np;
            obj.normalize();
            float fac = 1.0f;

            auto cd = dot(c, obj);
            if (cd < 0.999f) {
                correctVector(c, &Node::getForwardVector, { obj.x,0.0f,obj.y }, 0.0f, RSC*delta, 0.0f);
                fac = rfac;
            }

            auto d = dot(c, obj);
            auto f = c->getForwardVectorWorld();
            f.normalize();
            auto fd = 1.0f + f.dot(-Vector3::unitY());
            if (d > 0.7f) {
                c->translateForward(std::min(delta*v*fd*fac, np.distance(mDest)))
                    , sample += delta;
                correct(instance, delta, fcnt);
                return true;
            }
        }
        correct(instance, delta, fcnt);
        return false;
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
