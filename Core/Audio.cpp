#include "Audio.h"
#include <map>
uint16_t audioLevel = 0;
float gain = 1.0f;

void AudioManager::setScene(Scene* scene) {
    mScene = scene;
    mScene->bindAudioListenerToCamera(true);
    alDistanceModel(AL_LINEAR_DISTANCE);
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
    if (audioLevel < 1 || gain == 0.0f)return;
    uniqueRAII<AudioSource> source =
        AudioSource::create(to_string("res/audio/common/", type, ".ogg").c_str(), true);
    source->setLooped(false);
    source->setGain(gain);
    source->play();
    auto src = source->getSource();
    alSourcef(src, AL_REFERENCE_DISTANCE, 1.0f);
    alSourcef(src, AL_ROLLOFF_FACTOR, 1000.0f);
    alSourcef(src, AL_MAX_DISTANCE, 10000.0f);
    mSource.emplace_back(mScene->addNode());
    mSource.back()->setAudioSource(source.get());
    mSource.back()->setTranslation(pos);
}

void AudioManager::update() {

    auto erase = [&](auto i) {
        mScene->removeNode(i->get());
        mSource.erase(i);
    };

begin:
    for (auto i = mSource.cbegin(); i != mSource.cend(); ++i)
        if ((*i)->getAudioSource()->getState() == AudioSource::State::STOPPED) {
            erase(i);
            goto begin;
        }

    while (mSource.size() > 16)
        erase(mSource.begin());

}

void AudioManager::clear() {
    mScene = nullptr;
    mSource.clear();
}
