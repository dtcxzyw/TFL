#include "Audio.h"
#include <map>
uint16_t audioLevel = 0;
float gain = 1.0f;

void AudioManager::setScene(Scene* scene) {
    scene->bindAudioListenerToCamera(false);
    mListener = scene->getActiveCamera()->getNode();
    alDistanceModel(AL_NONE);
    {
        mSize = 0;
        ALuint tmp[64];
        for (size_t i = 1; i <= 64; ++i) {
            alGenBuffers(i,tmp);
            if (alGetError() != AL_NO_ERROR)break;
            alDeleteBuffers(i, tmp);
            alGenSources(i, tmp);
            if (alGetError() != AL_NO_ERROR)break;
            alDeleteSources(i, tmp);
            mSize = i;
        }
        INFO("OpenAL max size ", mSize);
    }
}

#define E(name) {Enum::name,#name}
std::ostream& operator<<(std::ostream& out, AudioType type) {
    using Enum = decltype(type);
    static std::map<AudioType, const char*> map = {
        E(boom),
        E(fire)
    };
    return out << map[type];
}
#undef E

void AudioManager::play(AudioType type, Vector3 pos) {
    if (audioLevel < 1 || gain == 0.0f || mSource.size()>=16)return;
    uniqueRAII<AudioSource> source =
        AudioSource::create(to_string("res/audio/common/", type, ".ogg").c_str());
    source->setLooped(false);
    source->setGain(gain);
    source->play();
    mSource.emplace_back(std::move(source),pos);
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
        auto fac =1.0f - std::min(dis, max) / max;
        s.first->setGain(gain*fac);
    }
}

void AudioManager::clear() {
    mListener = nullptr;
    mSource.clear();
}
