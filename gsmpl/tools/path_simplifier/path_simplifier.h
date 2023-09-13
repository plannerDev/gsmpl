#pragma once

#include <algorithm>
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/base/math_utility.h"
#include "gsmpl/planner_data/planner_solution.h"
#include "gsmpl/tools/trajectory_processing/trajectory.h"
#include "gsmpl/tools/local_planner/local_planner.h"

namespace gsmpl {
class PathSimplifier {
public:
    struct SolutionRecord {
        Path simplified;
        Path smoothed;
        Path simplified_2;
    };

    PathSimplifier(const LocalPlannerBasePtr& local_planner,
                   const DistanceBasePtr& distance)
        : localPlanner_(local_planner), distance_(distance) {}

    // rangeRatio the maximum distance between states a connection is attempted,
    // as a fraction relative to the total number of state (between 0 ~ 1)
    Path reduceVertices(const Path& rawPath, double step_size_jps,
                        double stepSizeTcp, unsigned int maxSteps = 0,
                        unsigned int maxEmptySteps = 10,
                        double rangeRatio = 0.33);
    Path collapseCloseVertices(const Path& rawPath, double step_size_jps,
                               double stepSizeTcp, unsigned int maxSteps = 0,
                               unsigned int maxEmptySteps = 10) const;
    Path smoothBSpline(const Path& rawPath, unsigned int maxSteps,
                       double minChange, double step_size_jps,
                       double stepSizeTcp) const;
    // Add a state at the middle of each segment
    Path subdividePath(const Path& path) const;

private:
    RNG rng_;
    LocalPlannerBasePtr localPlanner_;
    DistanceBasePtr distance_;
};
} // namespace gsmpl
