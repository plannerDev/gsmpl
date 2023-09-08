#pragma once

#include "../base/tree.h"
#include "../base/state.h"
#include "../goal/goal.h"
#include "../planner_data/planner_solution.h"
#include "../planner_data/planner_record.h"
#include "../planner_data/planner_param.h"
#include "../utility/class_forward.h"
#include "../tools/checker/state_checker_base.h"
#include "space_information.h"
#include "problem_definition.h"
#include "../planner_data/planner_context.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(Planner)

class Planner {
public:
    Planner(const SpaceInformationBasePtr& si, const ProblemDefinition& pd)
        : si_(si), start_(pd.start), goal_(pd.goal), opti_(pd.optObj) {}
    virtual ~Planner() = default;

    virtual const PlannerSolution& solve() = 0;
    virtual PlannerRecord plannerRecord() const = 0;

protected:
    SpaceInformationBasePtr si_;
    State start_;
    GoalBasePtr goal_;
    OptiObjectiveBasePtr opti_;
    PlannerSolution solution_;
};
} // namespace gsmpl
