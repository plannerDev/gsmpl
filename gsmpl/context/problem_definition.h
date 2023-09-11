#pragma once

#include "gsmpl/goal/goal.h"
#include "gsmpl/base/state.h"
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/context/optimization_objective.h"

namespace gsmpl {
struct ProblemDefinition {
    State start;
    GoalBasePtr goal;
    OptiObjectiveBasePtr opt_obj;
};
} // namespace gsmpl
