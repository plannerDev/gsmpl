#pragma once

#include "planner_data/planner_solution.h"
#include "planner_data/planner_param.h"
#include "planner_data/planner_record.h"
#include "utility/export.h"
#include "context/planner.h"
#include "context/problem_definition.h"
#include "context/space_information.h"
#include "rrt/rrt.h"
#include "rrt_star/rrt_star.h"
#include "bi_rrt/bi_rrt.h"

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
