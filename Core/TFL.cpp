#include "common.h"
#include "UI.h"
#include "Server.h"
#include "Client.h"
#include <ctime>
#include <fstream>

bool isRunning = true;
uint64_t pakKey;
std::unique_ptr<BindingResolver> br;

void callback() {
    isRunning = false;
}

std::string getLogPath() {
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    std::stringstream ss;
    ss << "logs/"
        << tm->tm_year + 1900 << '-' << tm->tm_mon + 1 << '-' << tm->tm_mday << ' '
        << tm->tm_hour << ' ' << tm->tm_min << ' ' << tm->tm_sec
        << ".log";
    return ss.str();
}

std::stringstream logtext;
uniqueRAII<Form> label;
double lastTime = 0.0f;
void logCallback(Logger::Level level, const char* message) {
#ifdef WIN32
    static std::ofstream log(getLogPath());
    if (!log.is_open())throw;
    log << message << std::flush;
#else
    static FILE* file = nullptr;
    if (!file) {
        std::string name = getLogPath();
        { uniqueRAII<Stream> log = FileSystem::open(name.c_str(), FileSystem::WRITE); }
        file = FileSystem::openFile(name.c_str(), "wb");
    }
    fwrite(message, strlen(message), 1, file);
    fflush(file);
#endif // WIN32
    logtext << message;
    if (label) {
        auto info = dynamic_cast<Label*>(label->getControl("info"));
        info->setText((std::string(info->getText()) + message).c_str());
        lastTime = Game::getAbsoluteTime();
    }
}

void buildDir(const std::string& name) {
    size_t pos = 0;
    std::vector<size_t> point;
    while (pos != std::string::npos)
        point.emplace_back(pos = name.find_first_of('/', pos + 1));
    point.pop_back();
    if (point.size())
        for (auto&& x : point)
            MKDIR(name.substr(0, x).c_str());
}

void unpackPack(Stream* pak, uint64_t& p, uint64_t all) {
    struct Path final {
        char name[60] = {};
        uint32_t size = 0;
    } tmp;

    while (pak->read(&tmp, sizeof(tmp), 1)) {
        INFO("Unpacking File ", tmp.name);
        buildDir(tmp.name);
        uniqueRAII<Stream> file = FileSystem::open(tmp.name, FileSystem::WRITE);
        std::vector<char> data(tmp.size);
        pak->read(data.data(), tmp.size, 1);
        file->write(data.data(), tmp.size, 1);

        Game::getInstance()->
            clear(Game::CLEAR_COLOR, Vector4::one()*pak->position() / pak->length(), 1.0f, 0);
        Platform::swapBuffers();
    }
}

void unpackAllPacks() {
    INFO("Checking key");
    std::vector<std::string> paks;
    FileSystem::listFiles("paks", paks);
    std::remove_if(paks.begin(), paks.end(),
        [](const std::string& name) {return !(name.size() > 4
        && name.substr(name.size() - 4, 4) == ".pak"); });

    uint64_t key = 0;
    struct PackInfo final {
        uniqueRAII<Stream> data;
        uint8_t height;
        std::string name;
        uint8_t id;
    };
    std::vector<PackInfo> pakData;
    uint8_t cnt = 0;
    uint64_t size = 0;
    for (auto&& p : paks) {
        uniqueRAII<Stream> data = FileSystem::open(("paks/" + p).c_str());
        uint64_t k;
        data->read(&k, sizeof(k), 1);
        key ^= k;
        size += data->length();
        pakData.push_back({ std::move(data), 0,p,cnt });
        ++cnt;
    }

    pakKey = key;

    bool reload = false;
    {
        uniqueRAII<Stream> keyCache = FileSystem::open("key");
        uint64_t oldKey;
        if (!keyCache || keyCache->read(&oldKey, sizeof(oldKey), 1) == 0 || oldKey != key)
            reload = true;
    }

    if (!reload)return;

    removeAll(std::string(FileSystem::getResourcePath())+"res");

    std::map<std::string, std::vector<uint8_t>> parents;
    std::set<std::string> out;
    for (auto&& p : pakData) {
        uint8_t num;
        p.data->read(&num, sizeof(num), 1);
        char str[32];
        for (uint8_t i = 0; i < num; ++i) {
            p.data->read(str, 32, 1);
            parents[p.name].emplace_back(p.id);
            out.insert(str);
        }
    }

    uint64_t pos = 0;
    for (auto&& p : pakData)
        pos += p.data->position();

    if (out.size() == pakData.size())
        GP_ERROR("Circular reference.");

    std::set<uint8_t> unpacked;

    std::function<void(uint8_t)> visit = [&](uint8_t id) {
        for (auto&& c : parents[pakData[id].name])
            visit(c);
        if (unpacked.find(id) == unpacked.cend()) {
            INFO("Unpacking ", pakData[id].name);
            unpackPack(pakData[id].data.get(), pos, size);
            unpacked.insert(id);
        }
    };

    Game::getInstance()->
        clear(Game::CLEAR_COLOR, Vector4::one()*pos / size, 1.0f, 0);
    Platform::swapBuffers();

    for (auto&& x : pakData)
        if (out.find(x.name) == out.cend())
            visit(x.id);

    uniqueRAII<Stream> keyCache = FileSystem::open("key", FileSystem::WRITE);
    keyCache->write(&key, sizeof(key), 1);
}

void readSettings() {
    uniqueRAII<Properties> info = Properties::create("game.settings");
    if (info) {
        shadowSize = info->getFloat("shadowSize");
        if (!shadowSize)shadowSize = 1;
        enableParticle = info->getBool("enableParticle");
    }
}

class TFL final : public Game {
private:
    void initResource() {
        INFO("Initializing Resource...");

#ifdef ANDROID
        bool unpack = false;
        if (FileSystem::fileExists("paks/Core.pak")) {
            uniqueRAII<Stream> old = FileSystem::open("paks/Core.pak");
            uniqueRAII<Stream> now = FileSystem::open("Core.pak");
            uint64_t k1, k2;
            old->read(&k1, sizeof(k1), 1);
            now->read(&k2, sizeof(k2), 1);
            unpack = k1 != k2;
        }
        else unpack = true;

        if (unpack) {
            INFO("Unpacking Core");
            uniqueRAII<Stream> pak = FileSystem::open("Core.pak");
            uniqueRAII<Stream> newPak = FileSystem::open("paks/Core.pak", FileSystem::WRITE);
            std::vector<unsigned char> data(pak->length());
            pak->read(data.data(), data.size(), 1);
            newPak->write(data.data(), data.size(), 1);
        }
#endif // ANDROID

        unpackAllPacks();

        INFO("Done.");
    }

protected:

    void initialize() override {

        std::atexit(callback);

        Logger::set(Logger::LEVEL_ERROR, logCallback);
        Logger::set(Logger::LEVEL_INFO, logCallback);
        Logger::set(Logger::LEVEL_WARN, logCallback);
        Logger::setEnabled(Logger::LEVEL_ERROR, true);
        Logger::setEnabled(Logger::LEVEL_INFO, true);
        Logger::setEnabled(Logger::LEVEL_WARN, true);

        MKDIR("logs");

        INFO("Initializing TFL...");

        initResource();

        {
            label = Form::create("label", Theme::getDefault()->getStyle("Label"));
            label->setAutoSize(Control::AutoSize::AUTO_SIZE_BOTH);
            label->setConsumeInputEvents(false);
            label->setAlignment(Control::Alignment::ALIGN_BOTTOM_LEFT);
            auto info = Label::create("info");
            label->addControl(info);
            info->setAutoSize(Control::AutoSize::AUTO_SIZE_BOTH);
            info->setAlignment(Control::Alignment::ALIGN_BOTTOM_LEFT);
            info->setConsumeInputEvents(false);
        }

        UnitController::initAllController();

        setMultiTouch(false);
        setVsync(true);
        setMultiSampling(true);
        readSettings();

        INFO("Loading resources...");
        br = std::make_unique<BindingResolver>();
        loadAllUnits();
        loadAllBullets();

        UI::push<MainMenu>();

        INFO("Go!!!");
    }

    void finalize() override {}

    void update(float delta) override {
        UI::updateForm(delta);
        auto now = Game::getAbsoluteTime();
        if (now - lastTime > 3000.0)
            dynamic_cast<Label*>(label->getControl("info"))->setText("");
        label->update(delta);
    }

    void render(float delta) override {
#ifdef WIN32
        if (!(getWidth() && getHeight()))
            return;
#endif

        clear(CLEAR_COLOR_DEPTH, Vector4::zero(), 1.0f, 0);
        if (localClient)localClient->render();
        else setViewport(gameplay::Rectangle(getWidth(), getHeight()));
        UI::render();
        label->draw();
    }
public:

    bool mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta) override {
        constexpr float unit = 100.0f;
        if (localClient) {
            switch (evt) {
            case Mouse::MOUSE_PRESS_LEFT_BUTTON:localClient->beginPoint(x, y);
                break;
            case Mouse::MOUSE_RELEASE_LEFT_BUTTON:localClient->endPoint(x, y);
                break;
            case Mouse::MOUSE_PRESS_RIGHT_BUTTON:localClient->cancel();
            case Mouse::MOUSE_MOVE:localClient->mousePos(x, y); break;
            case Mouse::MOUSE_WHEEL:localClient->scaleEvent(wheelDelta*unit);
                break;
            default:
                break;
            }
        }
        return true;
    }
    void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) override {
        static int lx = 0, ly, px, py;
        static bool press = false,flag;

        if (localClient) {
            switch (evt) {
            case Touch::TOUCH_PRESS:lx = px = x, ly = py = y, flag = false;
                break;
            case Touch::TOUCH_RELEASE:
            {
                flag = true;
                if (localClient->hasChoosed()) {
                    localClient->beginPoint(x, y);
                    localClient->endPoint(x, y);
                }
                else {
                    if ((px - x)*(px - x) + (py - y)*(py - y)<25) {
                        if (press) localClient->endPoint(x, y);
                        else localClient->beginPoint(x, y);
                        press = !press;
                    }
                }
            }
            break;
            case Touch::TOUCH_MOVE:
            {
                constexpr auto unit = 0.001f;
                if ((px - x)*(px-x) + (py - y)*(py - y)>25|| flag) {
                    localClient->moveEvent((lx - x)*unit, (ly - y)*unit);
                    press = false, flag = true;
                }
                lx = x, ly = y;
            }
            break;
            default:
                break;
            }
        }
    }
} game;
