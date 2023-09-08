#pragma once

#include <optional>
#include "../planner.h"
#include "../../tools/nearest_neighbor/nearest_neighbor.h"
#include "../../planner_data/planner_record.h"
#include "rrt_star_param.h"

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
                double stepSize) const;
    Path generatePath(const VertexPtr &v) const;
    bool isValidEdge(const State &q1, const State &q2) {
        if (si_->checkers->isValid(q2)) {
            if (si_->localPlanner->validInterpolatePath(
                    q1, q2, param_.localPlannerStepSizeJps,
                    param_.localPlannerStepSizeTcp))
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
    NearestNeighborBasePtr nearestNeighbor_;
    Tree tree_;
    State currentGoal_;
    PlannerRecord::VisualFunction vf_;
    PlannerRecord::VisualPoseFunction vpf_;
    double prunedMeasure_;
};
} // namespace gsmpl
