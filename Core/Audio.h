#pragma once
#include "common.h"
extern uint16_t audioLevel;
extern float gain;

enum class AudioType {
    fire, boom
};

enum class CodeType {
    to, attack, found, success
};

enum class StateType {
    fire, ready, in
};

class AudioManager final {
private:
    Node* mListener;
    std::list<std::pair<uniqueRAII<AudioSource>,Vector3>> mSource;
    struct Voice final {
        uniqueRAII<AudioSource> current;
        std::queue<std::string> last;
    };
    std::list<Voice> mVoice;
    std::map<std::string, std::vector<std::string>> mVoiceFormat;
    std::set<uint32_t> mHistory;
    size_t mSize;

    void voice(const char* name, Vector3 pos, std::vector<uint32_t> args);
public:

    void setScene(Scene* scene);
    void play(AudioType type, Vector3 pos);
    void voice(CodeType type, std::vector<uint32_t> args = {});
    void voice(StateType type, Vector3 pos, std::vector<uint32_t> args = {});
    void update();
    void clear();
};
