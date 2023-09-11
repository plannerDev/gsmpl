#pragma once

#include "gsmpl/base/tree.h"
#include "gsmpl/base/state.h"
#include "gsmpl/goal/goal.h"
#include "gsmpl/planner_data/planner_solution.h"
#include "gsmpl/planner_data/planner_record.h"
#include "gsmpl/planner_data/planner_param.h"
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/tools/checker/state_checker_base.h"
#include "gsmpl/context/space_information.h"
#include "gsmpl/context/problem_definition.h"
#include "gsmpl/planner_data/planner_context.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(Planner)

class Planner {
public:
    Planner(const SpaceInformationBasePtr& si, const ProblemDefinition& pd)
        : si_(si), start_(pd.start), goal_(pd.goal), opti_(pd.opt_obj) {}
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
