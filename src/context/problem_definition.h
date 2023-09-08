#pragma once

#include "../goal/goal.h"
#include "../base/state.h"
#include "../utility/class_forward.h"
#include "optimization_objective.h"

namespace gsmpl {
struct ProblemDefinition {
    State start;
    GoalBasePtr goal;
    OptiObjectiveBasePtr optObj;
};
} // namespace gsmpl
