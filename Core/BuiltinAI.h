#pragma once
#include <future>
void AIMain(bool* flag,uint8_t level);
extern std::unique_ptr<std::future<void>> aiFuture;
