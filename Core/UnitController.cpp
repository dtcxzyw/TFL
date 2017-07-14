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

void correct(UnitInstance& instance, float delta, float& cnt, float& time) {
    auto node = instance.getNode();
    auto kind = &instance.getKind();
    auto p = node->getTranslation();
    auto b = p + kind->getOffset();
    auto h = localClient->getHeight(b.x, b.z);
    if (b.y < h) {
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
        if (cnt > 250.0f)time = -500.0f;
        cnt = 0.0f;
    }
    else {
        node->translateY(-49.0f*(2.0f*cnt + delta)*delta / 1e6f);
        cnt += delta;
    }
}

auto dot(Node* node, Vector2 obj) {
    auto f = node->getForwardVectorWorld();
    Vector2 x{ f.x,f.z };
    x.normalize();
    return obj.dot(x);
}

bool move(UnitInstance& instance,Vector2 dest,Vector2 np,Node* c,float RSC,float delta,float rfac,
    float v,float& sample,float& fcnt,float& x) {
    if (!dest.isZero()) {

        auto obj = dest - np;
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
            c->translateForward(std::min(delta*v*fd*fac, np.distance(dest)))
                , sample += delta;
            correct(instance, delta, fcnt, x);
            return true;
        }
    }
    correct(instance, delta, fcnt, x);
    return false;
}

#define Init(name) name(info->getFloat(#name))
struct Tank final :public UnitController {
    float RST, RSC, v, time, harm, dis, count, rfac, sample, range, offset, speed, fcnt, x,sy,bt;
    Vector2 last;
    std::string bullet;
    bool onBack;
    Tank(const Properties* info) : Init(RST), Init(RSC), Init(v), Init(time), Init(harm), Init(dis), Init(rfac),
        Init(range), count(0.0f), sample(0.0f), bullet(info->getString("bullet")), Init(offset), Init(speed),
        x(10000.0f),sy(0.0f),bt(0.0f),onBack(false),fcnt(0.0f) {
        v /= 1000.0f;
        time *= 1000.0f;
        //speed /= 1000.0f;
    }

    bool update(UnitInstance& instance, float delta) override {

        if (sy == 0.0f)
            sy = instance.getNode()->getScaleY();

        {
            x += delta;
            float y = 1.0f / (1.0f +std::pow(M_E, std::min(-x / 100.0f,20.0f)));
            auto v = std::abs(y - 0.5f);
            //0.5->1 0->m
            //k=2-2m b=m
            constexpr auto m = 0.8f, k = 2.0f - 2.0f * m, b =m;
            auto s = k*v + b;
            instance.getNode()->setScaleY(s*sy);
        }

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
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
                        offset*f, point, speed, harm, range, instance.getGroup()));
                }
                t->translateForward(-(bt = 15.0f));
                onBack = true;
                count = 0.0f;
            }
            else if(!onBack) {
                auto dest = point - now;
                dest.normalize();
                correctVector(t, &Node::getForwardVectorWorld, dest, 0.0f, RST*delta, 0.0f);
            }
        }
        else if(!onBack) {
            mObject = 0;
            auto f = node->getForwardVectorWorld();
            f.normalize();
            correctVector(t, &Node::getForwardVectorWorld, f, 0.0f, RST*delta, 0.0f);
        }

        if (onBack) {
            bt -= delta/100.0f;
            t->translateForward(delta / 100.0f);
            if (bt < 0.0f) {
                onBack = false;
                t->translateForward(bt);
            }
        }

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x);
    }
};
struct DET final :public UnitController {
    float RSC, RSX, RSY, harm,dis,time,v,rfac,x,sy,fcnt,count,sample,ht,RT;
    Vector2 last;
    DET(const Properties* info):Init(RSC),Init(RSX),Init(RSY),Init(harm),Init(dis),Init(time),Init(v),Init(rfac)
    , x(10000.0f), sy(0.0f), count(0.0f), sample(0.0f), fcnt(0.0f),Init(ht),RT(0.0f){
        v /= 1000.0f;
        time *= 1000.0f;
        ht *= 1000.0f;
    }

    bool update(UnitInstance& instance, float delta) override {

        if (sy == 0.0f)
            sy = instance.getNode()->getScaleY();

        {
            x += delta;
            float y = 1.0f / (1.0f + std::pow(M_E, std::min(-x / 100.0f, 20.0f)));
            auto v = std::abs(y - 0.5f);
            //0.5->1 0->m
            //k=2-2m b=m
            constexpr auto m = 0.8f, k = 2.0f - 2.0f * m, b = m;
            auto s = k*v + b;
            instance.getNode()->setScaleY(s*sy);
        }

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
            return false;
        }
        auto node = instance.getNode();
        count += delta;
        count = std::min(count, time + 0.1f);

        auto c = node;
        auto ty = node->findNode("yr");
        auto t = ty->findNode("xr");
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
            if (d > 0.9999f && count > time && obj.lengthSquared() < dis) {
                count = 0.0f;
                RT = ht;
            }
            else {
                auto dest = point - now;
                dest.normalize();
                correctVector(ty, &Node::getForwardVectorWorld, dest, 0.0f, RSY*delta, 0.0f);
                correctVector(t, &Node::getForwardVectorWorld, dest, RSX*delta, 0.0f, 0.0f);
            }
        }
        else {
            mObject = 0;
            auto f = node->getForwardVectorWorld();
            f.normalize();
            correctVector(ty, &Node::getForwardVectorWorld, f, 0.0f, RSY*delta, 0.0f);
            correctVector(t, &Node::getForwardVectorWorld, f, RSX*delta, 0.0f, 0.0f);
        }

        auto ray = t->findNode("ray");
        if (RT>0.0f && !point.isZero()) {
            if(mIsServer) 
                localServer->attack(mObject, delta*harm);
            else {
                auto len = point.distance(now);
                ray->scaleZ(len / ray->getBoundingSphere().radius);
                ray->translate((point - ray->getTranslationWorld()) / 2.0f);
                ray->setEnabled(true);
                RT -= delta;
            }
        }
        else {
            ray->scaleZ(1.0f / ray->getBoundingSphere().radius);
            ray->setTranslation(Vector3::zero());
            ray->setEnabled(false);
        }

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x);
    }
};
#undef Init

using factoryFunction = std::function<std::unique_ptr<UnitController>(const Properties *)>;
static std::map<std::string, factoryFunction> factory;

void UnitController::initAllController() {
#define Model(name) factory[#name]=[](auto info){return std::make_unique<name>(info);}
    Model(Tank);
    Model(DET);
#undef Model
}

std::unique_ptr<UnitController> UnitController::newInstance(const Properties * info) {
    return factory[info->getString("type")](info);
}
