#include "gsmpl/base/path.h"

#include <assert.h>

namespace gsmpl {
double pathLength(const Path& path, const DistanceBasePtr distance) {
    assert(path.size() > 1);
    assert(distance);
    double length = 0.0;
    State q1 = path[0];
    for (int i = 1; i < path.size(); i++) {
        State q2 = path[i];
        length += distance->distance(q1, q2);
        q1 = q2;
    }
    return length;
}

void printPath(const Path& path) {
    for (const auto& p : path)
        p.printState();
}
} // namespace gsmpl
