#pragma once

#include <optional>
#include "../planner.h"
#include "gsmpl/tools/nearest_neighbor/nearest_neighbor.h"
#include "gsmpl/planner_data/planner_record.h"
#include "gsmpl/planner/rrt/rrt_param.h"

namespace gsmpl {
class RRT : public Planner {
public:
    enum Status { Reached, Advanced, Trapped };

    RRT(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
        const PlannerContext& context, const PlannerRecord::VisualFunction& vf);

    const PlannerSolution& solve() override;
    PlannerRecord plannerRecord() const override;
    void visualize() const { vf_(plannerRecord()); }

private:
    std::optional<VertexPtr> extend(const State& qSampled);
    void update(const VertexPtr& edge);
    State newState(const State& qSampled, const State& qNear,
                   double step_size) const;
    Path generatePath(const VertexPtr& v) const;

    RRTParam param_;
    NearestNeighborBasePtr nearest_neighbor_;
    Tree tree_;
    State current_goal_;
    PlannerRecord::VisualFunction vf_;
};
} // namespace gsmpl
