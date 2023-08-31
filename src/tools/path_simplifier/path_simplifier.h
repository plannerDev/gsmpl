#pragma once

#include <algorithm>
#include "../../utility/log_utility.h"
#include "../../base/math_utility.h"
#include "../../planner_data/planner_solution.h"
#include "../../base/trajectory.h"
#include "../local_planner/local_planner.h"

namespace gsmpl {
class PathSimplifier
{
public:
    struct SolutionRecord
    {
        Trajectory simplified;
        Trajectory smoothed;
        Trajectory simplified_2;
    };

    PathSimplifier(const LocalPlannerBasePtr& localPlanner, const DistanceBasePtr& distance)
        : localPlanner_(localPlanner), distance_(distance)
    {
    }

    // rangeRatio the maximum distance between states a connection is attempted,
    // as a fraction relative to the total number of state (between 0 ~ 1)
    Path reduceVertices(const Path& rawPath, double stepSizeJps, double stepSizeTcp,
                        unsigned int maxSteps = 0, unsigned int maxEmptySteps = 10,
                        double rangeRatio = 0.33);
    Path collapseCloseVertices(const Path& rawPath, double stepSizeJps, double stepSizeTcp,
                               unsigned int maxSteps = 0, unsigned int maxEmptySteps = 10) const;
    Path smoothBSpline(const Path& rawPath, unsigned int maxSteps, double minChange,
                       double stepSizeJps, double stepSizeTcp) const;
    // Add a state at the middle of each segment
    Path subdividePath(const Path& path) const;
    Trajectory generateTrajector(const Path& path, double stepSizeJps, double stepSizeTcp) const;

private:
    RNG rng_;
    LocalPlannerBasePtr localPlanner_;
    DistanceBasePtr distance_;
};
} // namespace gsmpl
