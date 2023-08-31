#pragma once

#include <vector>
#include "../base/state.h"
#include "../base/path.h"
#include "../utility/log_utility.h"
#include "../utility/export.h"
#include "../base/trajectory.h"
#include "planner_param.h"

namespace gsmpl {
enum class PlannerStatus {
    InvalidStart,
    InvalidGoal,
    TimeOut,
    ApproximateSolution,
    ExactSolution,
    Crash,
    Abort,
    Initialized,
};

struct EXPORT PlannerSolution
{
    bool isValid() const
    {
        return status == PlannerStatus::ApproximateSolution ||
               status == PlannerStatus::ExactSolution;
    }

    PlannerStatus status = PlannerStatus::Initialized;
    Path path;
    Trajectory trajectory;
    double cost = 0.0;
    bool approximate = false;
};
} // namespace gsmpl
