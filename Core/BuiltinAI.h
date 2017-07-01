#pragma once
#include <future>
void AIMain(bool* flag);
extern std::unique_ptr<std::future<void>> aiFuture;
