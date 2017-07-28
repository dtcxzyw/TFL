#pragma once
#include "common.h"
extern uint16_t audioLevel;
extern float gain;

enum class AudioType {
    fire, boom
};

enum class CodeType {
    to, attack, found, help, success, died
};

enum class StateType {
    fire, ready, in
};

class AudioManager final {
private:
    Node* mListener;
    std::list<std::pair<uniqueRAII<AudioSource>,Vector3>> mSource;
    size_t mSize;
public:

    void setScene(Scene* scene);
    void play(AudioType type, Vector3 pos);
    void update();
    void clear();
};
