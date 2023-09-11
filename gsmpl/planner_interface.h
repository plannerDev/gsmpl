#pragma once

#include "gsmpl/planner_data/planner_solution.h"
#include "gsmpl/planner_data/planner_param.h"
#include "gsmpl/planner_data/planner_record.h"
#include "gsmpl/utility/export.h"
#include "gsmpl/planner/planner.h"
#include "gsmpl/context/problem_definition.h"
#include "gsmpl/context/space_information.h"
#include "gsmpl/planner/rrt/rrt.h"
#include "gsmpl/planner/rrt_star/rrt_star.h"
#include "gsmpl/planner/bi_rrt/bi_rrt.h"
#include "gsmpl/planner/informed_rrt_star/informed_rrt_star.h"

namespace gsmpl {
class EXPORT PlannerInterface {
public:
    PlannerInterface(const PlannerContext& context, const ProblemDefinition& pd,
                     const PlannerRecord::VisualFunction& vf,
                     const PlannerRecord::VisualPoseFunction& vpf = nullptr);

    bool plan(PlannerSolution& solution);

    void visualize() const { vf_(record_); }

    const PlannerRecord& plannerRecord() const { return record_; }

private:
    bool allocPlanner(const PlannerContext& context);
    void allocSi(const PlannerContext& context);

    PlannerGeneralParamters param_;
    ProblemDefinition pd_;
    PlannerPtr planner_;
    SpaceInformationBasePtr si_;
    PlannerRecord record_;
    PlannerRecord::VisualFunction vf_;
    PlannerRecord::VisualPoseFunction vpf_;
};
} // namespace gsmpl
