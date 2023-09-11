#pragma once

#include <optional>
#include "gsmpl/planner/planner.h"
#include "gsmpl/tools/nearest_neighbor/nearest_neighbor.h"
#include "gsmpl/planner_data/planner_record.h"
#include "gsmpl/planner/rrt_star/rrt_star_param.h"

namespace gsmpl {
class RRTStar : public Planner {
public:
    enum Status { Reached, Advanced, Trapped };

    RRTStar(const SpaceInformationBasePtr &si, const ProblemDefinition &pd,
            const PlannerContext &context,
            const PlannerRecord::VisualFunction &vf,
            const PlannerRecord::VisualPoseFunction &vpf = nullptr);

    const PlannerSolution &solve() override;
    PlannerRecord plannerRecord() const override;
    void visualize() const;
    void visualize(const State &qNew, const State &qNear) const;

private:
    void update(const VertexPtr &edge);
    State steer(const State &qSampled, const State &qNear,
                double step_size) const;
    Path generatePath(const VertexPtr &v) const;
    bool isValidEdge(const State &q1, const State &q2) {
        if (si_->checkers->isValid(q2)) {
            if (si_->local_planner->validInterpolatePath(
                    q1, q2, param_.local_planner_step_size_jps,
                    param_.local_planner_step_size_tcp))
                return true;
        }
        return false;
    }

    double edgeCost(const State &q1, const State &q2) const {
        return si_->distance->distance(q1, q2);
    }

    double kRRT(double rewireFactor) const;
    double rRRT(double rewireFactor) const;

    RRTStarParam param_;
    NearestNeighborBasePtr nearest_neighbor_;
    Tree tree_;
    State current_goal_;
    PlannerRecord::VisualFunction vf_;
    PlannerRecord::VisualPoseFunction vpf_;
    double pruned_measure_;
};
} // namespace gsmpl
