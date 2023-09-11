#pragma once

#include <optional>

#include "gsmpl/planner/planner.h"

#include "gsmpl/tools/nearest_neighbor/nearest_neighbor.h"
#include "gsmpl/planner_data/planner_record.h"
#include "gsmpl/tools/sampler/informed_sampler.h"
#include "gsmpl/planner/bi_rrt/bi_rrt.h"
#include "gsmpl/planner/informed_rrt_star/informed_rrt_star_param.h"

namespace gsmpl {
class InformedRRTStar : public Planner {
public:
    enum Status { Reached, Advanced, Trapped };

    InformedRRTStar(const SpaceInformationBasePtr& si,
                    const ProblemDefinition& pd, const PlannerContext& context,
                    const PlannerRecord::VisualFunction& vf,
                    const PlannerRecord::VisualPoseFunction& vpf = nullptr);

    const PlannerSolution& solve() override;
    PlannerRecord plannerRecord() const override;
    void visualize() const;

private:
    struct NearKEdgeCost {
        NearKEdgeCost(const VertexPtr& vp, double cost) : v(vp), eCost(cost) {}

        VertexPtr v{nullptr};
        double eCost{0};
    };

    void prune(Tree& tree, double costThreshold);
    double heuristic(const VertexPtr& v) const;
    void addVertex(const VertexPtr& v);
    std::optional<BiRRT::Record> findFeasiblePath();
    bool initialCheck();
    void initNearestNeighbor(const Tree& tree);
    void updateNN(NearestNeighborBasePtr& nn, const VertexPtr& v);
    Path generatePath(const VertexPtr& v) const;
    State steer(const State& qSampled, const State& qNear,
                double step_size) const;
    bool isValidEdge(const State& q1, const State& q2) const;
    double edgeCost(const State& q1, const State& q2) const;
    double kRRT(double rewireFactor) const;
    double rRRT(double rewireFactor) const;
    BiRRTParam toBirrtParam(const InformedRRTStarParam& param) const;
    void visualize(const State& qNew, const State& qNear) const;

    InformedRRTStarParam param_;
    NearestNeighborBasePtr nearest_neighbor_;
    Tree tree_;
    State current_goal_;
    State v1_;
    State v2_;
    PlannerRecord::VisualFunction vf_;
    PlannerRecord::VisualPoseFunction vpf_;
    double pruned_measure_;
    BiRRT bi_rrt_;
};
} // namespace gsmpl
