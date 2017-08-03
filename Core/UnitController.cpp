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

bool UnitController::isStoped() const {
    return mDest.isZero();
}

void UnitController::isServer() { mIsServer = true; }

void UnitController::onDied(UnitInstance & instance) {}

void correct(UnitInstance& instance, float delta, float& cnt, float& time) {
    auto node = instance.getNode();
    auto&& kind = instance.getKind();
    auto p = node->getTranslation();
    auto offset = kind.getOffset()*node->getDownVector().normalize();
    auto b = p + offset;
    auto h = localClient->getHeight(b.x, b.z);
    if (b.y <= h) {
        {
            Vector3 pos = { b.x,h,b.z };
            node->translateSmooth(pos - offset, delta, 100.0f);
        }

        auto d = kind.getPlane();
        auto f = node->getForwardVector().normalize();
        auto r = node->getRightVector().normalize();

        std::array<Vector2, 4> base = {
            Vector2{ d.x,d.y },
            Vector2{d.x,-d.y } ,
            Vector2{-d.x,d.y } ,
            Vector2{-d.x,-d.y }
        };
        std::array<Vector3, 4> sample;
        size_t idx = 0;
        for (auto&&x : base) {
            auto sp = p+r*x.x+f*x.y;
            sample[idx] = { sp.x,localClient->getHeight(sp.x,sp.z),sp.z };
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
            mean += up;
        }

        mean *= 0.25f;

        correctVector(node, &Node::getUpVector, mean.normalize(), M_PI, 0.0f, M_PI);
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
    return obj.dot(x.normalize());
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
            c->translateForward(std::min(delta*v*fd*fac, np.distance(dest)));
            sample += delta;
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

        if (mObject && abs(point.y - now.y) <= 100.0f && bt <= 15.0f) {
            auto obj = Vector2{ point.x,point.z } -np;
            obj.normalize();
            auto d = dot(yr, obj);
            auto f = t->getForwardVectorWorld().normalize();

            if (!mIsServer) {
                for (auto&& x : fireUnits)
                    if (x.z >= time) {
                        localClient->getAudio().voice(StateType::ready, instance.getID(), now);
                    }
                if (mObject && obj.lengthSquared() <= dis)
                    localClient->getAudio().voice(StateType::in, instance.getID(), now, { mObject });
            }

            if (d > 0.996f && obj.lengthSquared() <= dis) {

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
                            offset*f + u*iter->y + r*iter->x, point, f, speed, harm, range, instance.getGroup()));
                    }
                    else {
                        localClient->getAudio().voice(StateType::fire, instance.getID(), now);
                        localClient->getAudio().play(AudioType::fire, now);
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
            auto f = node->getForwardVector();
            correctVector(yr, &Node::getForwardVectorWorld, f.normalize(), 0.0f, RST*delta, 0.0f);
            mObject = 0;
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
        if (mObject) {
            auto obj = point - now; obj.normalize();
            correctVector(ty, &Node::getForwardVectorWorld, obj, 0.0f, RSY*delta, 0.0f);
            auto limit = ty->getForwardVectorWorld().normalize()
                .dot(t->getForwardVectorWorld().normalize());
            correctVector(t, &Node::getForwardVectorWorld, obj, std::min(limit, RSX*delta), 0.0f, 0.0f);
            auto f = t->getForwardVectorWorld().normalize();
            d = obj.dot(f);
        }
        else {
            auto f = node->getForwardVectorWorld();
            f.normalize();
            correctVector(ty, &Node::getForwardVectorWorld, f, 0.0f, RSY*delta, 0.0f);
            correctVector(t, &Node::getForwardVectorWorld, f, RSX*delta, 0.0f, 0.0f);
        }

        auto ray = t->findNode("ray");
        auto p = t->getTranslationWorld();

        if (count >= sub)
            time += st, count -= sub;

        if (!mIsServer&& mObject && p.distanceSquared(point) <= dis)
            localClient->getAudio().voice(StateType::in, instance.getID(), p, { mObject });

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
    float RSC, rfac, time, sy, x, fcnt, count, sample, v, range, harm, speed, angle, dis;
    Vector2 last;
    std::string missile;
    CBM(const Properties* info) :Init(RSC), Init(rfac), Init(time), sy(0.0f), x(0.0f), fcnt(0.0f), count(0.0f),
        sample(0.0f), Init(v), missile(info->getString("missile")), Init(range), Init(harm), Init(speed)
        , Init(angle), Init(dis) {
        v /= 1000.0f;
        time *= 1000.0f;
        dis *= dis;
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

        if (!mIsServer) {
            node->findNode("missile")->setEnabled(count >= time*0.8f);
            if (count >= time)
                localClient->getAudio().voice(StateType::ready, instance.getID(), now);
            if (mObject && now.distanceSquared(point) <= dis)
                localClient->getAudio().voice(StateType::in, instance.getID(), now, { mObject });
        }

        if (mObject && count >= time && now.distanceSquared(point) <= dis) {
            if (mIsServer) {
                auto m = node->findNode("missile");
                localServer->newBullet(BulletInstance(missile, m->getTranslationWorld(), Vector3::zero(),
                    m->getForwardVectorWorld().normalize(), speed, harm, range, instance.getGroup()
                    , mObject, angle));
            }
            else localClient->getAudio().voice(StateType::fire, instance.getID(), now);
            count = 0.0f;
        }

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x, -1.0f);
    }
};

struct CBR final :public UnitController {
    float RSC, rfac, sy, x, fcnt, sample, v, harm;
    Vector2 last;
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

        if (mObject && mIsServer) {
            auto d = now.distanceSquared(point);
            auto fov = instance.getKind().getFOV();
            localServer->attack(mObject, delta*harm*fov / d);
        }

        node->findNode("radar")->rotateY(M_PI*2.0f / 1000.0f*delta);

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x, -1.0f);
    }
};

struct CBG final :public UnitController {
    float RSC, RSX, RSY, harm, dis, v, rfac, x, sy, fcnt, count, sample, time, range, speed, offset, bt;
    Vector2 last;
    std::string bullet;
    bool onBack, car;
    CBG(const Properties* info) :Init(RSC), Init(RSX), Init(RSY), Init(harm), Init(dis), Init(v), Init(rfac)
        , x(10000.0f), sy(0.0f), count(0.0f), sample(0.0f), fcnt(0.0f), Init(time), Init(range)
        , Init(speed), Init(offset), bullet(info->getString("bullet")), bt(0.0f), onBack(false)
        , car(info->getBool("car", false)) {
        v /= 1000.0f;
        dis *= dis;
        time *= 1000.0f;
    }

    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta);

        if (instance.isDied()) {
            correct(instance, delta, fcnt, x);
            return false;
        }
        auto node = instance.getNode();
        count += delta;
        count = std::min(count, time + 1.0f);

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

        if (mObject) {
            auto obj = point - now; obj.normalize();
            correctVector(ty, &Node::getForwardVectorWorld, obj, 0.0f, RSY*delta, 0.0f);
            if (!onBack) {
                auto limit = ty->getForwardVectorWorld().normalize()
                    .dot(t->getForwardVectorWorld().normalize());
                correctVector(t, &Node::getForwardVectorWorld, obj, std::min(limit, RSX*delta), 0.0f, 0.0f);
            }
            auto f = t->getForwardVectorWorld().normalize();
            if (count >= time && obj.lengthSquared() <= dis &&
                obj.dot(f) >= 0.996f && checkRay(now, point) == point && bt <= 30.0f) {
                if (mIsServer)
                    localServer->newBullet(BulletInstance(bullet, t->getTranslationWorld() +
                        offset*f, point, f, speed, harm, range, instance.getGroup()));
                else {
                    localClient->getAudio().voice(StateType::fire, instance.getID(), now);
                    localClient->getAudio().play(AudioType::fire, now);
                }

                count = 0.0f;
                t->translateForward(bt);
                t->translateForward(-(bt = 40.0f));
                onBack = true;
            }
        }
        else {
            mObject = 0;
            auto f = node->getForwardVector().normalize();
            correctVector(ty, &Node::getForwardVectorWorld, f, 0.0f, RSY*delta, 0.0f);
            if (!onBack)
                correctVector(t, &Node::getForwardVectorWorld, f, RSX*delta, 0.0f, 0.0f);
        }

        if (bt > 0.0f) {
            if (bt >= delta*0.1f)bt -= delta*0.1f, t->translateForward(delta*0.1f);
            else {
                t->translateForward(bt);
                bt = 0.0f;
            }
        }
        else onBack = false;

        return move(instance, mDest, np, c, RSC, delta, rfac, v, sample, fcnt, x, car ? -1.0f : 0.7f);
    }
};

void fly(UnitInstance& instance, Vector2 dest, float h, float v, float delta, float RSC, Vector3 now) {
    h += std::max(localClient->getHeight(now.x, now.z), 0.0f);

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

    constexpr auto fac = 0.1f;
    auto f = Vector3::unitY();
    correctVector(instance.getNode(), &Node::getUpVector, f,
        dest.isZero() ? RSC*delta*fac : 0.0f, 0.0f, RSC*delta*fac);
}

void fall(UnitInstance& instance, float delta) {
    auto node = instance.getNode();
    auto pos = instance.getRoughPos();
    if (pos.y - localClient->getHeight(pos.x, pos.z) < 15.0f)return;
    constexpr auto RSF = 0.005f;
    std::uniform_real_distribution<float> URD(0.0f, RSF*delta);
    node->rotateX(URD(mt));
    node->rotateZ(URD(mt));
}

struct PBM final :public UnitController {
    float time, fcnt, count, v, dis, range, harm, speed, angle, RSC, height;
    std::string missile;
    PBM(const Properties* info) : Init(time), fcnt(0.0f), count(0.0f), Init(RSC), Init(height)
        , Init(v), Init(dis), missile(info->getString("missile")), Init(range), Init(harm), Init(speed), Init(angle) {
        v /= 1000.0f;
        time *= 1000.0f;
        dis *= dis;
    }
    bool update(UnitInstance& instance, float delta) override {

        if (instance.isDied()) {
            fall(instance, delta);
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
        auto in = mObject && now.distanceSquared(point) <= dis;

        if (!mIsServer) {
            if (count >= time)
                localClient->getAudio().voice(StateType::ready, instance.getID(), now);
            if (in)
                localClient->getAudio().voice(StateType::in, instance.getID(), now, { mObject });
            if (count >= time && in)
                count = 0.0f;
        }
        else if (count >= time && in) {
            localServer->newBullet(BulletInstance(missile, now +
                Vector3{ 0.0f, instance.getKind().getOffset(), 0.0f }, Vector3::zero(),
                node->getForwardVectorWorld().normalize(), speed, harm, range,
                instance.getGroup(), mObject, angle));
            count = 0.0f;
        }

        if (mDest.distanceSquared({ now.x,now.z }) < 40000.0f)
            mDest = Vector2::zero();

        fly(instance, mDest, height, v, delta, RSC, now);
        return true;
    }
};

struct TP final :public UnitController {
    float fcnt, v, RSC, height, x, sy;
    Vector2 mStart;
    TP(const Properties* info) : fcnt(0.0f), Init(RSC), Init(height), Init(v), x(10000.0f), sy(0.0f) {
        v /= 1000.0f;
    }
    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta, 0.75f);

        if (instance.isDied()) {
            fall(instance, delta);
            correct(instance, delta, fcnt, x);
            return false;
        }

        if (mIsServer) {
            if (!mDest.isZero()) {
                if (localClient->getHeight(mDest.x, mDest.y) < 0.0f)
                    mDest = Vector2::zero();
                auto node = instance.getNode();
                auto c = node;
                auto now = node->getTranslation();

                if (instance.getLoadSize()) {
                    if (mDest.distanceSquared({ now.x,now.z }) < 10000.0f)
                        localServer->releaseUnit(instance), mDest = mStart;

                    fly(instance, mDest, height, v, delta, RSC, now);
                }
                else {
                    auto dis = mDest.distanceSquared({ now.x,now.z });
                    auto flag = now.y - localClient->getHeight(now.x, now.z) < instance.getKind().getRadius();
                    if (flag && dis < 2.5e5f) {
                        mDest = Vector2::zero();
                        correct(instance, delta, fcnt, x);
                    }
                    else {
                        auto h = std::pow(std::min(dis / 9e6f, 1.0f), 0.25f)*height;
                        fly(instance, mDest, h, v, delta, RSC, now);
                        h += std::max(localClient->getHeight(now.x, now.z), 0.0f);
                        if (h < node->getTranslationY()) {
                            auto p = node->getTranslation();
                            node->translateSmooth({ p.x,h,p.y }, delta, 100.0f);
                            correctVector(instance.getNode(), &Node::getUpVector, Vector3::unitY(),
                                M_PI, 0.0f, M_PI);
                        }
                    }
                }
            }
            else {
                auto p = instance.getRoughPos();
                mStart = { p.x,p.z };
                correct(instance, delta, fcnt, x);
            }
        }

        return true;
    }
};

struct Copter final :public UnitController {
    float RSC, v, height, dis, time, offset, count[2], fcnt, harm, range, speed, ry, sy, x;
    std::string missile;
    Vector3 last;
    bool onCritical;
    Copter(const Properties* info) :Init(RSC), Init(v), Init(height), Init(dis), Init(time), Init(offset),
        missile(info->getString("missile")), fcnt(0.0f), Init(harm), Init(range), Init(speed), ry(0.0f)
        , onCritical(false), sy(0.0f), x(10000.0f) {
        v /= 1000.0f;
        dis *= dis;
        time *= 1000.0f;
        count[0] = count[1] = 0.0f;
    }

    bool update(UnitInstance& instance, float delta) override {

        scale(instance, sy, x, delta);

        constexpr auto w = 0.99f;

        auto node = instance.getNode();

        if (instance.isDied()) {
            fall(instance, delta);
            ry *= w;
            node->findNode("up")->rotateY(ry);
            node->findNode("rotate")->rotateY(ry);
            correct(instance, delta, fcnt, x);
            return false;
        }

        auto now = instance.getNode()->getTranslation();
        auto point = localClient->getPos(mObject);

        for (auto i = 0; i < 2; ++i)
            count[i] = std::min(count[i] + delta, time + 0.1f);

        auto h = localClient->getHeight(now.x, now.z);

        if (mObject) {
            auto f = node->getForwardVector().normalize();
            auto off = point - now;
            off.normalize();
            auto in = point.distanceSquared(now) <= dis && point.y <= now.y + 100.0f
                && f.dot(off) > 0.7f;
            auto ready = count[0] >= time || count[1] >= time;
            if (mIsServer) {
                if (in && ready) {
                    auto left = node->getLeftVector().normalize();
                    for (auto i = 0; i < 2; ++i)
                        if (count[i] >= time) {
                            count[i] = 0.0f;
                            localServer->newBullet(BulletInstance(missile, now + offset*left*(i - 0.5f)*2.0f,
                                Vector3::zero(), f, speed, harm, range,
                                instance.getGroup(), mObject, RSC));
                        }
                }

                if (mDest.isZero() && point.distanceSquared(now) <= instance.getKind().getFOV()) {
                    correctVector(node, &Node::getForwardVector, off, 0.0f, RSC*delta, 0.0f);
                    onCritical = true;
                }
            }
            else {
                if (in)localClient->getAudio().voice(StateType::in, instance.getID(), now, { mObject });
                if (ready)localClient->getAudio().voice(StateType::ready, instance.getID(), now);
                if (in&&ready) {
                    for (auto i = 0; i < 2; ++i)
                        if (count[i] >= time) {
                            count[i] = 0.0f;
                            localClient->getAudio().voice(StateType::fire, instance.getID(), now);
                        }
                }
            }
        }
        else onCritical = false;

        if (mIsServer) {
            if (mDest.isZero()) {
                if (h > 0.0f && !onCritical) correct(instance, delta, fcnt, x);
                else {
                    correctVector(node, &Node::getUpVector, Vector3::unitY(), RSC*delta, 0.0f, RSC*delta);
                    if (node->getTranslationY() < std::max(h, 0.0f) + height)
                        node->translateUp(v*delta);
                }
            }
            else {
                h = std::max(h, 0.0f) + height;
                Vector3 dest = { mDest.x,h,mDest.y };
                auto offset = dest - now;
                offset.y = (offset.y > 0.0f ? -1.0f : 1.0f)*std::min(std::abs(offset.y),
                    std::abs(hypotf(offset.x, offset.z)*0.363f));
                correctVector(node, &Node::getForwardVector, offset.normalize(), RSC*delta, RSC*delta, 0.0f);
                correctVector(node, &Node::getUpVector, Vector3::unitY(), 0.0f, 0.0f, RSC*delta);
                node->translateForward(v*delta);
                auto d = h - now.y;
                node->translateY((d > 0.0f ? 1.0f : -1.0f)*std::min(std::abs(d), v*delta));
                correct(instance, 0.0f, fcnt, x);
                if (dest.distanceSquared(now) <= height*height + 1e4f)
                    mDest = Vector2::zero();
            }
        }
        else {
            auto up = node->getTranslationY() - 10.0f > h ? M_PI*delta : 0.0f;
            auto dis = last.isZero() ? 0.0f : last.distanceSquared(now);
            constexpr auto fac = 0.005f;
            ry = ry*w + (up + dis)*fac*(1.0f - w);
            node->findNode("up")->rotateY(ry);
            last = now;
            node->findNode("rotate")->rotateY(ry);
        }

        return true;
    }
};

struct Submarine final :public UnitController {
    float RSC, height, v, dis, time, harm, range, speed, fcnt, lfac, count;
    std::string bullet, missile;
    Vector3 last;
    Submarine(const Properties* info) :Init(RSC), Init(height), Init(v), Init(dis), Init(time),
        Init(harm), Init(range), Init(speed), fcnt(0.0f), Init(lfac), count(0.0f),
        missile(info->getString("missile")), bullet(info->getString("bullet")) {
        time *= 1000.0f;
        v /= 1000.0f;
        dis *= dis;
    }
    bool update(UnitInstance& instance, float delta) override {

        if (instance.isDied()) {
            float x;
            correct(instance, delta, fcnt, x);
            return false;
        }

        auto node = instance.getNode();
        auto now = instance.getNode()->getTranslation();

        count = std::min(count + delta, time + 0.1f);

        auto point = localClient->getPos(mObject);
        if (mObject) {
            bool in = point.distanceSquared(now) <= dis;
            if (!mIsServer) {
                if (in)
                    localClient->getAudio().voice(StateType::in, instance.getID(), now, { mObject });
                if (count >= time)
                    localClient->getAudio().voice(StateType::ready, instance.getID(), now);
                if (in && count >= time) {
                    count = 0.0f;
                    localClient->getAudio().voice(StateType::fire, instance.getID(), now);
                }
            }
            else if (in && count >= time) {
                if (point.y < 0.0f)
                    localServer->newBullet(BulletInstance(bullet, now - Vector3::unitY()*10.0f
                        , point, node->getForwardVector().normalize()
                        , speed, harm, range, instance.getGroup()));
                else
                    localServer->newBullet(BulletInstance(missile, now + Vector3::unitY()*10.0f,
                        Vector3::zero(), Vector3::unitY(), speed, harm, range,
                        instance.getGroup(), mObject, RSC));
                count = 0.0f;
            }
        }

        {
            float tmp;
            correct(instance, now.y >= 0.0f ? delta : 0.0f, fcnt, tmp);
        }

        if (!mDest.isZero()) {
            if (mDest.distanceSquared({ now.x,now.z }) < 1e3f)
                mDest = Vector2::zero();
            else {
                if (now.y >= 0.0f) {
                    Vector3 pos{ mDest.x,localClient->getHeight(mDest.x,mDest.y),mDest.y };
                    auto offset = pos - now;
                    correctVector(node, &Node::getForwardVector, offset.normalize()
                        , RSC*delta, RSC*delta, 0.0f);
                    node->translateForward(v*delta*lfac);
                }
                else {
                    auto h = localClient->getHeight(mDest.x, mDest.y);
                    Vector3 pos{ mDest.x,h > 0.0f ? h : h*height,mDest.y };
                    auto offset = pos - now;
                    correctVector(node, &Node::getForwardVector, offset.normalize()
                        , RSC*delta, RSC*delta, 0.0f);
                    node->translateForward(v*delta);
                }

                correctVector(node, &Node::getUpVector, Vector3::unitY(), 0.0f, 0.0f, M_PI);

                return true;
            }
        }

        if (!mIsServer) {
            auto d = last.distance(now);
            node->findNode("push")->rotateZ(d*delta);
            last = now;
        }
        return false;
    }
};

struct Ship final :public UnitController {
    float RSC, RST, btime, v, dis, time, harm, range, speed, fcnt, lfac, bcnt, mcnt, offset;
    std::string bullet, missile;
    Ship(const Properties* info) :Init(RSC), Init(RST), Init(btime), Init(v), Init(dis), Init(time), Init(offset),
        Init(harm), Init(range), Init(speed), fcnt(0.0f), Init(lfac), bcnt(0.0f), mcnt(0.0f),
        missile(info->getString("missile")), bullet(info->getString("bullet")) {
        time *= 1000.0f;
        btime *= 1000.0f;
        v /= 1000.0f;
        dis *= dis;
    }
    bool update(UnitInstance& instance, float delta) override {

        if (instance.isDied()) {
            float x;
            correct(instance, delta, fcnt, x);
            return false;
        }

        auto node = instance.getNode();
        auto t = node->findNode("turret");
        auto now = instance.getNode()->getTranslation();

        bcnt = std::min(bcnt + delta, btime + 0.1f);
        mcnt = std::min(mcnt + delta, time + 0.1f);

        auto point = localClient->getPos(mObject);
        if (mObject) {
            bool in = point.distanceSquared(now) <= dis;
            auto off = point - t->getTranslationWorld();
            auto d = dot(t, Vector2{ off.x,off.z }.normalize());
            {
                auto vec = node->getRightVector().normalize();
                auto f = t->getForwardVectorWorld();
                auto limit = std::acos(std::max(f.dot(vec), f.dot(-vec)));
                correctVector(t, &Node::getForwardVectorWorld, off.normalize(),
                    0.0f, std::min(RST*delta, limit), 0.0f);
            }
            if (!mIsServer) {
                if (in)
                    localClient->getAudio().voice(StateType::in, instance.getID(), now, { mObject });
                if (bcnt >= btime || mcnt >= time)
                    localClient->getAudio().voice(StateType::ready, instance.getID(), now);
                if (in && bcnt >= btime && d > 0.996f) {
                    bcnt = 0.0f;
                    localClient->getAudio().voice(StateType::fire, instance.getID(), now);
                }
            }
            else if (in) {
                if (point.y < 0.0f) {
                    if (bcnt >= btime) {
                        localServer->newBullet(BulletInstance(bullet, now - Vector3::unitY()*10.0f
                            , point, node->getForwardVector().normalize()
                            , speed, harm, range, instance.getGroup()));
                        bcnt = 0.0f;
                    }
                }
                else {
                    if (bcnt >= btime && d > 0.996f) {
                        auto f = t->getForwardVector().normalize();
                        localServer->newBullet(BulletInstance(bullet, t->getTranslationWorld() + f*offset
                            , point, f, speed, harm, range, instance.getGroup()));
                        bcnt = 0.0f;
                    }
                    if (mcnt >= time) {
                        localServer->newBullet(BulletInstance(missile, now + Vector3::unitY()*10.0f,
                            Vector3::zero(), Vector3::unitY(), speed, harm, range,
                            instance.getGroup(), mObject, RSC));
                        mcnt = 0.0f;
                    }
                }
            }
        }

        if (!mDest.isZero()) {
            if (mDest.distanceSquared({ now.x,now.z }) < 1e3f)
                mDest = Vector2::zero();
            else {
                auto h = localClient->getHeight(now.x, now.y);
                Vector3 pos{ mDest.x,std::max(h,0.0f),mDest.y };
                auto offset = pos - now;

                correctVector(node, &Node::getForwardVector, offset.normalize()
                    , 0.0f, RSC*delta, 0.0f);
                node->translateForward(v*delta);
            }
        }

        {
            correctVector(node, &Node::getUpVector, Vector3::unitY(), M_PI, 0.0f, M_PI);
            if (node->getTranslationY() < 0.0f)
                node->setTranslationY(0.0f);
            float tmp;
            correct(instance, node->getTranslationY() <= 0.0f ? 0.0f : delta, fcnt, tmp);
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
    Model(DET);
    Model(CBM);
    Model(PBM);
    Model(CBR);
    Model(CBG);
    Model(TP);
    Model(Submarine);
    Model(Ship);
    Model(Copter);
#undef Model
}

std::unique_ptr<UnitController> UnitController::newInstance(const Properties * info) {
    return factory[info->getString("type")](info);
}
