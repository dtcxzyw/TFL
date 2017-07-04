#pragma once

#define BT_USE_SSE_IN_API
#include <gameplay.h>
#include <memory>
#include <type_traits>
#include <sstream>
#include <random>

#ifdef WIN32
#include <Windows.h>
#include <direct.h>
#define MKDIR(x) mkdir(x)
#else
#include <dirent.h>
#include <sys/stat.h>
#define MKDIR(x) mkdir((x),0777)
#endif

using namespace gameplay;

extern bool isRunning;

struct Deleter final {
    void operator()(Ref* ptr) const {
        if (isRunning)
            ptr->release();
    }
    template<typename T,typename = std::enable_if_t<!std::is_base_of<Ref,T>::value>>
    void operator()(T* ptr) const {
            delete ptr;
    }
};

template<typename T>
class uniqueRAII final :public std::unique_ptr<T,Deleter> {
public:
    uniqueRAII() = default;
    uniqueRAII(T* ptr) :std::unique_ptr<T, Deleter>(ptr){}
};


inline std::string to_string() {
    return "";
}

template<typename FirstT,typename... ArgsT>
std::string to_string(FirstT&& first, ArgsT&&... args) {
    std::stringstream ss;
    ss << first;
    return ss.str()+ to_string(std::forward<ArgsT>(args)...);
}

#define INFO(...) do \
    { \
        gameplay::Logger::log(gameplay::Logger::LEVEL_INFO, "%s (%s line %d) \n", __current__func__,__FILE__,__LINE__); \
        gameplay::Logger::log(gameplay::Logger::LEVEL_INFO, to_string(__VA_ARGS__).c_str()); \
        gameplay::Logger::log(gameplay::Logger::LEVEL_INFO, "\n"); \
    } while (0)

bool listDirs(const char* dirPath, std::vector<std::string>& dirs);

template<typename T>
void nativeSwap(T& a, T& b) {
    char tmp[sizeof(T)];
    std::memmove(tmp, &a, sizeof(T));
    std::memmove(&a, &b, sizeof(T));
    std::memmove(&b, tmp, sizeof(T));
}

extern std::mt19937_64 mt;

struct BindingResolver final :private RenderState::AutoBindingResolver {
    bool resolveAutoBinding(const char* autoBinding, Node* node, MaterialParameter* parameter) override {
#define CMP(x) (strcmp(autoBinding,x)==0)
        if (CMP("LIGHT_COLOR"))
            parameter->setValue(Vector3::one()*1.5f);
        else if (CMP("LIGHT_DIRECTION"))
            parameter->setValue(-Vector3::one());
        else return false;
#undef CMP
        return true;
    }
};

#ifdef ANDROIDx86
using uint64_t = unsigned long long;
static_assert(sizeof(uint64_t) == 8, "");
#endif

void removeAll(const std::string& path);

void correctVector(Node* node, Vector3(Node::*sampler)() const, Vector3 dest,float x,float y,float z);

constexpr auto mapSize = 6000;
constexpr auto mapSizeF = static_cast<float>(mapSize);
constexpr auto mapSizeHF = mapSizeF/2.0f;

extern uint16_t shadowSize;
