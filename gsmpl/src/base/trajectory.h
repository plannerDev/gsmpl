#pragma once

#include <map>
#include <iostream>
#include "path.h"

namespace gsmpl {
struct Trajectory
{
    // struct Waypoint
    // {
    //     State jps;
    //     double timestamp;
    // };
    using Segment = std::vector<State>;
    using Waypoints = std::vector<State>;

    Waypoints waypoints;
    std::vector<Segment> segments;

    void print() const
    {
        for (int i = 0; i < waypoints.size() - 1; i++) {
            std::cout << "segment" << i << " size " << segments[i].size() << std::endl;
        }
    }
};
} // namespace gsmpl
