#ifndef _log_utility_H_
#define _log_utility_H_

#include <stdio.h>
#include <string.h>

// #define GSMPL_DEBUG

#define __FILENAME__ (strrchr(__FILE__, '/') + 1)

#ifdef GSMPL_DEBUG
#define LOGD(format, ...)                                                      \
    printf("[%s][%s][%d]: " format "\n", __FILENAME__, __FUNCTION__, __LINE__, \
           ##__VA_ARGS__)

#else
#define LOGD(fromat, ...)
#endif

#endif // _log_utility_H_
