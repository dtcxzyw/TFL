#pragma once
#include "common.h"
extern uint8_t audioLevel;

class AudioManager final {
private:
    std::vector<uniqueRAII<AudioSource>> source;
public:
    
};
