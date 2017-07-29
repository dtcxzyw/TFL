#include "Audio.h"
#include <map>
uint16_t audioLevel = 0;
float gain = 1.0f;

void AudioManager::voice(const char * name, Vector3 pos, std::vector<uint32_t> args) {
    if (gain == 0.0f || mSource.size() + mVoice.size() >= mSize ||
        mVoiceFormat.find(name) == mVoiceFormat.cend())return;
    auto VP = mListener->getCamera()->getViewProjectionMatrix();
    auto NDC = VP*Vector4(pos.x, pos.y, pos.z, 1.0f);
    NDC.x /= NDC.w; NDC.y /= NDC.w; NDC.z /= NDC.w;
    NDC.x /= 2.0f, NDC.y /= 2.0f; NDC.z /= 2.0f;
    NDC.x += 0.5f; NDC.y += 0.5f; NDC.z += 0.5f;
    NDC.y = 1.0f - NDC.y;
    if (NDC.x >= 0.0f && NDC.x <= 1.0f && NDC.y >= 0.0f
        && NDC.y <= 1.0f && NDC.z >= 0.0f && NDC.z <= 1.0f) {
        auto&& format = mVoiceFormat[name];
        std::queue<std::string> last;
        size_t cnt = 0;
        for (auto&& x : format) {
            if (x == "id") {
                if (cnt >= args.size())INFO("Error format");
                std::string id = to_string(args[cnt]);
                for (auto&& y : id)
                    last.emplace(&y);
                ++cnt;
            }
            else last.emplace(x);
        }
        mVoice.push_back({nullptr,last});
    }
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
            for (auto&& x : point)
                mVoiceFormat[id].emplace_back(format.substr(x, x - last));
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

void AudioManager::voice(CodeType type, Vector3 pos, std::vector<uint32_t> args) {
    if (audioLevel >= 2)
        voice(enum2string(type), pos, args);
}

void AudioManager::voice(StateType type, Vector3 pos, std::vector<uint32_t> args) {
    if (audioLevel >= 3)
        voice(enum2string(type), pos, args);
}

void AudioManager::update() {

begin:
    for (auto i = mSource.cbegin(); i != mSource.cend(); ++i)
        if (i->first->getState() == AudioSource::State::STOPPED) {
            mSource.erase(i);
            goto begin;
        }

    auto pos = mListener->getTranslationWorld();
    for (auto&& s : mSource) {
        auto dis = s.second.distance(pos);
        constexpr auto max = 2000.0f;
        auto fac = 1.0f - std::min(dis, max) / max;
        s.first->setGain(gain*fac);
    }

begin:
    for (auto i = mVoice.begin(); i != mVoice.end(); ++i)
        if (!i->current || i->current->getState() == AudioSource::State::STOPPED) {
            if (i->last.empty()) {
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
}
