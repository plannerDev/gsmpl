#pragma once

#include "../tools/distance/distance.h"
#include "state.h"

namespace gsmpl {
using Path = std::vector<State>;
double pathLength(const Path& path, const DistanceBasePtr distance);
void printPath(const Path& path);
} // namespace gsmpl
