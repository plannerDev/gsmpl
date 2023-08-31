#pragma once

#include <optional>
#include "../context/planner.h"
#include "../tools/nearest_neighbor/nearest_neighbor.h"
#include "../planner_data/planner_record.h"
#include "rrt_param.h"

namespace gsmpl {
class RRT : public Planner
{
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
    State newState(const State& qSampled, const State& qNear, double stepSize) const;
    Path generatePath(const VertexPtr& v) const;

    RRTParam param_;
    NearestNeighborBasePtr nearestNeighbor_;
    Tree tree_;
    State currentGoal_;
    PlannerRecord::VisualFunction vf_;
};
} // namespace gsmpl
