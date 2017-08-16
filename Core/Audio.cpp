#include "Audio.h"
#include <map>
#include "Client.h"
uint16_t audioLevel = 0;
float gain = 1.0f;

void AudioManager::voice(const char * name, Vector3 pos, std::vector<uint32_t> args) {
    if (gain == 0.0f || mSource.size() + mVoice.size() >= mSize || mVoice.size()>=4 ||
        mVoiceFormat.find(name) == mVoiceFormat.cend() 
        || (args.size() && mHistory.find(args.front())!=mHistory.cend()))return;
    if (!pos.isZero()) {
        auto VP = mListener->getCamera()->getViewProjectionMatrix();
        auto NDC = VP*Vector4(pos.x, pos.y, pos.z, 1.0f);
        NDC.x /= NDC.w; NDC.y /= NDC.w; NDC.z /= NDC.w;
        NDC.x /= 2.0f, NDC.y /= 2.0f; NDC.z /= 2.0f;
        NDC.x += 0.5f; NDC.y += 0.5f; NDC.z += 0.5f;
        NDC.y = 1.0f - NDC.y;
        if (NDC.x < 0.0f || NDC.x > 1.0f || NDC.y < 0.0f || NDC.y > 1.0f)return;
    }
        auto&& format = mVoiceFormat[name];
        std::queue<std::string> last;
        size_t cnt = 0;
        for (auto&& x : format) {
            if (x == "arg") {
                if (cnt >= args.size())INFO("Error format");
                else {
                    std::string id = to_string((args[cnt]%typeOffset%10000)+10);
                    for (auto&& y : id)
                        last.push({ y });
                    ++cnt;
                }
            }
            else last.emplace(x);
        }
        mVoice.push_back({ nullptr,last });
        mState[name] = true;
        if (args.size())
            mHistory.insert(args.front());
}

void AudioManager::setScene(Scene* scene) {
    scene->bindAudioListenerToCamera(false);
    mListener = scene->getActiveCamera()->getNode();
    alDistanceModel(AL_NONE);
    {
        mSize = 0;
        ALuint tmp[64];
        for (size_t i = 1; i <= 64; ++i) {
            alGenBuffers(i, tmp);
            if (alGetError() != AL_NO_ERROR)break;
            alDeleteBuffers(i, tmp);
            alGenSources(i, tmp);
            if (alGetError() != AL_NO_ERROR)break;
            alDeleteSources(i, tmp);
            mSize = i;
        }
        INFO("OpenAL max size ", mSize);
    }

    static const char* const info = "res/audio/voice/voice.info";
    if (FileSystem::fileExists(info)) {
        uniqueRAII<Properties> voice = Properties::create(info);
        const char* id;
        std::string format;
        while (id = voice->getNextProperty()) {
            format = voice->getString(id);
            format.push_back('+');
            size_t pos = 0;
            std::vector<size_t> point;
            while (pos != std::string::npos)
                point.emplace_back(pos = format.find_first_of('+', pos + 1));
            point.pop_back();
            size_t last = 0;
            for (auto&& x : point) {
                mVoiceFormat[id].emplace_back(format.substr(last, x - last));
                last = x+1;
            }
        }
    }
}

#define E(name) {Enum::name,#name}
std::ostream& operator<<(std::ostream& out, AudioType type) {
    using Enum = decltype(type);
    static std::map<Enum, const char*> map = {
        E(boom),
        E(fire)
    };
    return out << map[type];
}
auto enum2string(CodeType type) {
    using Enum = decltype(type);
    static std::map<Enum, const char*> map = {
        E(to),
        E(attack),
        E(found),
        E(success)
    };
    return map[type];
}
auto enum2string(StateType type) {
    using Enum = decltype(type);
    static std::map<Enum, const char*> map = {
        E(fire),
        E(ready),
        E(in)
    };
    return map[type];
}
#undef E

void AudioManager::play(AudioType type, Vector3 pos) {
    if (audioLevel < 1 || gain == 0.0f || mSource.size() + mVoice.size() >= mSize)return;
    uniqueRAII<AudioSource> source =
        AudioSource::create(to_string("res/audio/common/", type, ".ogg").c_str());
    source->setLooped(false);
    source->setGain(gain);
    source->play();
    mSource.emplace_back(std::move(source), pos);
}

void AudioManager::voice(CodeType type, std::vector<uint32_t> args) {
    if (audioLevel >= 2)
        voice(enum2string(type), {}, args);
}

void AudioManager::voice(StateType type,uint32_t id, Vector3 pos, std::vector<uint32_t> args) {
    if (audioLevel >= 3 && localClient->isMine(id) && 
        (mLast.find(id)==mLast.cend() || mLast[id]!=type) && !mState[enum2string(type)]) {
        voice(enum2string(type), pos, args);
        mLast[id] = type;
    }
}

void AudioManager::update() {

    mSource.erase(std::remove_if(mSource.begin(), mSource.end(), [](auto&& x) {
        return x.first->getState() == AudioSource::State::STOPPED;
    }), mSource.end());

    auto pos = mListener->getTranslationWorld();
    for (auto&& s : mSource) {
        auto dis = s.second.distance(pos);
        constexpr auto max = 2000.0f;
        auto fac = 1.0f - std::min(dis, max) / max;
        s.first->setGain(gain*fac*(pos.y<0.0f?0.5f:1.0f));
    }

begin:
    for (auto i = mVoice.begin(); i != mVoice.end(); ++i)
        if (!i->current || i->current->getState() == AudioSource::State::STOPPED) {
            if (i->last.empty()) {
                mState[i->type] = false;
                mVoice.erase(i);
                goto begin;
            }
            else {
                i->current = 
                    AudioSource::create(to_string("res/audio/voice/", i->last.front(), ".ogg").c_str());
                i->current->setLooped(false);
                i->current->setGain(gain);
                i->current->play();
                i->last.pop();
            }
        }

}

void AudioManager::clear() {
    mListener = nullptr;
    mSource.clear();
    mVoice.clear();
    mVoiceFormat.clear();
    mHistory.clear();
    mLast.clear();
    mState.clear();
}
