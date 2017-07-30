#pragma once
#include "common.h"
#include <stack>
#include <list>
#include <sstream>
#include <RakPeer.h>

class UI :protected Control::Listener {
private:
    static std::stack<std::unique_ptr<UI>> stack;
    static std::list<UI*> parents;
    void addListener(Control* control);
    void addListener();
    void removeListener(Control* control);
    void removeListener();
protected:
    using Event = Control::Listener::EventType;
    uniqueRAII<Form> mForm;
    UI(const std::string& name);
    template<typename T>
    auto get(const char* id) {
        return dynamic_cast<T*>(mForm->getControl(id));
    }
    void UI::clearControls(Container* c);
public:

    template<typename T>
    static void push() {
        if (stack.size())
            parents.push_back(stack.top().get());
        stack.push(std::make_unique<T>());
    }

    static void pop();
    static void updateForm(float delta);
    static void render();

    void controlEvent(Control* control, EventType evt) override;

    virtual void event(Control* control, EventType evt) = 0;
    virtual void update(float delta);
    virtual ~UI() = default;
};

class MainMenu final :public UI {
public:
    MainMenu();
    void event(Control* control, Event evt) override;
};

class AboutMenu final :public UI {
public:
    AboutMenu();
    void event(Control* control, Event evt) override;
};

class PlayMenu final :public UI {
private:
    RakNet::RakPeer mPeer;
    std::set<std::string> mServers;
public:
    PlayMenu();
    void event(Control* control, Event evt) override;
    void update(float = {}) override;
    ~PlayMenu();
};

class SettingsMenu final :public UI {
public:
    SettingsMenu();
    void event(Control* control, Event evt) override;
    void read();
    ~SettingsMenu();
};

class ServerMenu final :public UI {
public:
    ServerMenu();
    void event(Control* control, Event evt) override;
    void update(float) override;
    ~ServerMenu();
};

class ClientMenu final :public UI {
public:
    ClientMenu();
    void event(Control* control, Event evt) override;
    void update(float) override;
    ~ClientMenu();
};

class GameMain final :public UI {
private:
    std::string mCurrent;
    void choose(const char* type);
public:
    GameMain();
    void event(Control* control, Event evt) override;
    void update(float delta) override;
    ~GameMain();
};
