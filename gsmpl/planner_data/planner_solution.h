#pragma once

#include <vector>
#include "gsmpl/base/state.h"
#include "gsmpl/base/path.h"
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/utility/export.h"
#include "gsmpl/tools/trajectory_processing/trajectory.h"
#include "gsmpl/planner_data/planner_param.h"

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

struct EXPORT PlannerSolution {
    bool isValid() const {
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
