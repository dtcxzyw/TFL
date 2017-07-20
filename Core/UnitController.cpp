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

void UnitController::onDied(UnitInstance & instance) {}

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

auto checkRay(Vector3 begin, Vector3 end) {
    constexpr auto unit = 20.0f;
    int step = begin.distance(end) / unit + 1;
    for (int w = step - 1; w > 0; --w) {
        auto p = (begin*w + end*(step - w)) / step;
        if (localClient->getHeight(p.x, p.z) > p.y)return p;
    }
    return end;
}

bool move(UnitInstance& instance, Vector2 dest, Vector2 np, Node* c, float RSC, float delta, float rfac,
    float v, float& sample, float& fcnt, float& x, float turn = 0.7f) {
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
        auto base = -Vector3::unitY();
        auto fd = 1.0f + f.dot(base.normalize());
        if (d > turn) {
            c->translateForward(std::min(delta*v*fd*fac, np.distance(dest)))
                , sample += delta;
            correct(instance, delta, fcnt, x);
            return true;
        }
    }
    correct(instance, delta, fcnt, x);
    return false;
}

void scale(UnitInstance& instance, float& sy, float& x, float delta, float m = 0.8f) {
    if (sy == 0.0f)
        sy = instance.getNode()->getScaleY();

    x += delta;
    float y = 1.0f / (1.0f + std::pow(M_E, std::min(-x / 100.0f, 20.0f)));
    auto v = std::abs(y - 0.5f);
    //0.5->1 0->m
    //k=2-2m b=m
    float k = 2.0f - 2.0f * m, b = m;
    auto s = k*v + b;
    instance.getNode()->setScaleY(s*sy);
}

#define Init(name) name(info->getFloat(#name))
struct Tank final :public UnitController {
    float RST, RSC, v, time, harm, dis, rfac, sample, range, offset, speed, fcnt, x, sy, bt;
    Vector2 last;
    std::string bullet;
    bool onBack;
    std::vector<Vector3> fireUnits;
    Tank(const Properties* info) : Init(RST), Init(RSC), Init(v), Init(time), Init(harm), Init(dis), Init(rfac),
        Init(range), sample(0.0f), bullet(info->getString("bullet")), Init(offset), Init(speed),
        x(10000.0f), sy(0.0f), bt(0.0f), onBack(false), fcnt(0.0f) {
        v /= 1000.0f;
        time *= 1000.0f;
        dis *= dis;
        Vector3 mat;
        info->getVector3("mat", &mat);
        if (mat.isZero()) fireUnits.emplace_back();
        else {
            Vector2 offset(mat.z*mat.x, mat.z*mat.y);
            offset *= -0.5f;
            for (uint8_t i = 0; i < mat.x; ++i)
                for (uint8_t j = 0; j < mat.y; ++j)
                    fireUnits.emplace_back(offset.x + i*mat.z, offset.y + j*mat.z, 0.0f);
        }
    }

    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta);

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
            return false;
        }
        auto node = instance.getNode();
        for (auto&& x : fireUnits) {
            x.z += delta;
            x.z = std::min(x.z, time + 0.1f);
        }

        auto c = node;
        auto yr = node->findNode("yr");
        auto t = yr->findNode("turret");
        auto point = localClient->getPos(mObject);
        auto now = node->getTranslation();
        Vector2 np{ now.x,now.z };

        if (sample > 100.0f && !mDest.isZero()) {
            if (last.distanceSquared(np) < 10.0f*v && np.distanceSquared(mDest) < 10000.0f)
                mDest = Vector2::zero();
            last = np;
            sample = 0.0f;
        }

        if (mObject && !point.isZero() && abs(point.y - now.y) <= 100.0f && bt<=15.0f) {
            auto obj = Vector2{ point.x,point.z } -np;
            obj.normalize();
            auto d = dot(t, obj);
            auto f = t->getForwardVectorWorld().normalize();
            auto top =now+ f*now.distance(point)/speed*0.3f;
            if (d > 0.999f && obj.lengthSquared() <= dis
                && checkRay(now, top) == top) {

                auto iter = fireUnits.begin();
                for (; iter != fireUnits.end(); ++iter)
                    if (iter->z >= time) {
                        iter->z = 0.0f;
                        break;
                    }

                if (iter != fireUnits.end()) {
                    if (mIsServer) {
                        auto u = t->getUpVectorWorld().normalize();
                        auto r = t->getRightVectorWorld().normalize();
                        localServer->newBullet(BulletInstance(bullet, t->getTranslationWorld() +
                            offset*f + u*iter->y + r*iter->x, point,f, speed, harm, range, instance.getGroup()));
                    }
                    t->translateForward(bt);
                    t->translateForward(-(bt = 20.0f));
                    onBack = true;
                }
            }
            else if (!onBack) {
                auto dest = point - now;
                correctVector(yr, &Node::getForwardVectorWorld, dest.normalize(), 0.0f, RST*delta, 0.0f);
            }
        }
        else if (!onBack) {
            mObject = 0;
            auto f = node->getForwardVector();
            correctVector(yr, &Node::getForwardVectorWorld, f.normalize(), 0.0f, RST*delta, 0.0f);
        }

        if (onBack) {
            bt -= delta / 100.0f;
            t->translateForward(delta / 100.0f);
            if (bt < 0.0f) {
                onBack = false;
                t->translateForward(bt);
                bt = 0.0f;
            }
        }

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x);
    }
};

struct DET final :public UnitController {
    float RSC, RSX, RSY, harm, dis, v, rfac, x, sy, fcnt, count, sample, add, sub, max, st, time;
    Vector2 last;
    DET(const Properties* info) :Init(RSC), Init(RSX), Init(RSY), Init(harm), Init(dis), Init(add), Init(v), Init(rfac)
        , x(10000.0f), sy(0.0f), count(0.0f), sample(0.0f), fcnt(0.0f), Init(sub), Init(max), Init(st), time(0.0f) {
        v /= 1000.0f;
        st *= 1000.0f;
        dis *= dis;
    }

    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta);

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
            return false;
        }
        auto node = instance.getNode();
        count += add*delta;
        count = std::min(count, max);

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

        float d = 0.0f;
        if (mObject && !point.isZero()) {
            auto obj = point - now; obj.normalize();
            auto f = t->getForwardVectorWorld(); f.normalize();
            d = obj.dot(f);
            correctVector(ty, &Node::getForwardVectorWorld, obj, 0.0f, RSY*delta, 0.0f);
            auto limit = ty->getForwardVectorWorld().normalize()
                .dot(t->getForwardVectorWorld().normalize());
            correctVector(t, &Node::getForwardVectorWorld, obj, std::min(limit, RSX*delta), 0.0f, 0.0f);
        }
        else {
            mObject = 0;
            auto f = node->getForwardVectorWorld();
            f.normalize();
            correctVector(ty, &Node::getForwardVectorWorld, f, 0.0f, RSY*delta, 0.0f);
            correctVector(t, &Node::getForwardVectorWorld, f, RSX*delta, 0.0f, 0.0f);
        }

        auto ray = t->findNode("ray");
        auto p = t->getTranslationWorld();

        if (count >= sub)
            time += st, count -= sub;

        if (time >= delta && !point.isZero() && d > 0.999f && p.distanceSquared(point) <= dis) {
            auto end = checkRay(p, point);
            if (mIsServer) {
                if (end == point)
                    localServer->attack(mObject, delta*harm);
            }
            else {
                auto len = end.distance(p);
                ray->setScaleZ(len*0.05f);
                ray->setEnabled(true);
            }
            time -= delta;
        }
        else ray->setEnabled(false);

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x);
    }
    void onDied(UnitInstance& instance) override {
        instance.getNode()->findNode("ray")->setEnabled(false);
    }
};

struct CBM final :public UnitController {
    float RSC, rfac, time, sy, x, fcnt, count, sample, v, dis, range, harm, speed, angle;
    Vector2 last;
    std::string missile;
    CBM(const Properties* info) :Init(RSC), Init(rfac), Init(time), sy(0.0f), x(0.0f), fcnt(0.0f), count(0.0f),
        sample(0.0f), Init(v), Init(dis), missile(info->getString("missile")), Init(range), Init(harm), Init(speed)
        , Init(angle) {
        v /= 1000.0f;
        time *= 1000.0f;
    }
    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta, 0.9f);

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
            return false;
        }

        auto node = instance.getNode();
        count += delta;
        count = std::min(count, time + 0.1f);

        auto c = node;
        auto point = localClient->getPos(mObject);
        auto now = node->getTranslation();
        Vector2 np{ now.x,now.z };

        if (sample > 100.0f && !mDest.isZero()) {
            if (last.distanceSquared(np) < 10.0f*v && np.distanceSquared(mDest) < 10000.0f)
                mDest = Vector2::zero();
            last = np;
            sample = 0.0f;
        }

        if (!mIsServer)
            node->findNode("missile")->setEnabled(count >= time*0.8f);

        if (mObject && !point.isZero() && count >= time) {
            if (mIsServer) {
                auto m = node->findNode("missile");
                localServer->newBullet(BulletInstance(missile, m->getTranslationWorld(),Vector3::zero(),
                    m->getForwardVectorWorld().normalize(), speed, harm, range, instance.getGroup()
                    , mObject, angle));
            }
            count = 0.0f;
        }

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x, -1.0f);
    }
};

struct CBR final :public UnitController {
    float RSC, rfac, sy, x, fcnt, sample, v, harm;
    Vector2 last;
    std::string missile;
    CBR(const Properties* info) :Init(RSC), Init(rfac), sy(0.0f), x(0.0f), fcnt(0.0f),
        sample(0.0f), Init(v), Init(harm) {
        v /= 1000.0f;
        harm /= 1000.0f;
    }
    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta, 0.9f);

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
            return false;
        }

        auto node = instance.getNode();

        auto c = node;
        auto point = localClient->getPos(mObject);
        auto now = node->getTranslation();
        Vector2 np{ now.x,now.z };

        if (sample > 100.0f && !mDest.isZero()) {
            if (last.distanceSquared(np) < 10.0f*v && np.distanceSquared(mDest) < 10000.0f)
                mDest = Vector2::zero();
            last = np;
            sample = 0.0f;
        }

        if (mObject && !point.isZero() && mIsServer) {
            auto d = now.distanceSquared(point);
            localServer->attack(mObject,delta*harm*d/instance.getKind().getFOV());
        }

        node->findNode("radar")->rotateY(M_PI*2.0f/1000.0f*delta);

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x, -1.0f);
    }
};

void fly(UnitInstance& instance, Vector2 dest, float h, float v, float delta, float RSC, Vector3 now) {
    h += localClient->getHeight(now.x, now.z);

    if (now.y < h - 50.0f && dest.isZero())
        dest = { now.x,now.z };

    if (!dest.isZero()) {
        Vector3 obj(dest.x, h, dest.y);
        auto f = obj - now;
        correctVector(instance.getNode(), &Node::getForwardVector, f.normalize()
            , RSC*delta, RSC*delta, 0.0f);
        instance.getNode()->translateForward(v*delta);
    }
    else {
        instance.getNode()->rotateY(RSC*delta*0.1f);
        instance.getNode()->translateForward(v*delta*0.1f);
    }
    auto f = Vector3::unitY();
    correctVector(instance.getNode(), &Node::getUpVector, f.normalize()
        , dest.isZero() ? RSC*delta : 0.0f, 0.0f, RSC*delta);
}

struct PBM final :public UnitController {
    float time, fcnt, count, v, dis, range, harm, speed, angle, RSC, height;
    std::string missile;
    PBM(const Properties* info) : Init(time), fcnt(0.0f), count(0.0f), Init(RSC), Init(height)
        , Init(v), Init(dis), missile(info->getString("missile")), Init(range), Init(harm), Init(speed), Init(angle) {
        v /= 1000.0f;
        time *= 1000.0f;
    }
    bool update(UnitInstance& instance, float delta) override {

        if (instance.isDied()) {
            float x;
            correct(instance, delta, fcnt, x);
            return false;
        }

        auto node = instance.getNode();
        count += delta;
        count = std::min(count, time + 0.1f);

        auto c = node;
        auto point = localClient->getPos(mObject);
        auto now = node->getTranslation();

        if (mIsServer && mObject && !point.isZero() && count >= time) {
            localServer->newBullet(BulletInstance(missile, now + instance.getKind().getOffset(),
                Vector3::zero(),node->getForwardVectorWorld().normalize(), speed, harm, range,
                instance.getGroup(), mObject, angle));
            count = 0.0f;
        }

        if (now.distanceSquared({ mDest.x,height,mDest.y }) < 40000.0f)
            mDest = Vector2::zero();

        fly(instance, mDest, height, v, delta, RSC, now);
        return true;
    }
};


#undef Init

using factoryFunction = std::function<std::unique_ptr<UnitController>(const Properties *)>;
static std::map<std::string, factoryFunction> factory;

void UnitController::initAllController() {
#define Model(name) factory[#name]=[](auto info){return std::make_unique<name>(info);}
    Model(Tank);
    Model(DET);
    Model(CBM);
    Model(PBM);
#undef Model
}

std::unique_ptr<UnitController> UnitController::newInstance(const Properties * info) {
    return factory[info->getString("type")](info);
}
