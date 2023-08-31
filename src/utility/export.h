#pragma once

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#elif defined(__unix__)
#define EXPORT __attribute__((visibility("default")))
#endif
