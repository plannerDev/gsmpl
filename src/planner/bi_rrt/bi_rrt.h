#pragma once

#include <optional>
#include "../planner.h"
#include "../../tools/nearest_neighbor/nearest_neighbor.h"
#include "bi_rrt_param.h"

namespace gsmpl {
class BiRRT : public Planner {
public:
    enum class Status { Reached, Advanced, Trapped };
    struct Record {
        VertexPtr vStart;
        VertexPtr vGoal;
        VertexPtr v1;
        VertexPtr v2;
        Tree tree;
        PlannerRecord pr;
    };

    BiRRT(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
          const PlannerContext& context,
          const PlannerRecord::VisualFunction& vf);
    BiRRT(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
          const BiRRTParam& param, const PlannerRecord::VisualFunction& vf);

    const PlannerSolution& solve() override;
    PlannerRecord plannerRecord() const override { return record_.pr; }
    const Record& record() const { return record_; }
    void visualize() const { vf_(record_.pr); }
    void visualizePose(const State& q) const {
        PlannerRecord pr;
        pr.start = q;
        vf_(pr);
    }

private:
    enum class TreeType { Start, Goal };

    struct TreeTuple {
        TreeTuple(Tree* t, const NearestNeighborBasePtr& n, TreeType treeType)
            : tree(t), nearest(n), type(treeType) {}
        void update(const VertexPtr& vNew);

        Tree* tree;
        NearestNeighborBasePtr nearest;
        TreeType type;
    };

    std::optional<VertexPtr> extend(const State& qSampled,
                                    const NearestNeighborBasePtr& nearest);
    State newState(const State& qSampled, const State& qNear,
                   const double stepSize) const;
    Path generatePath(const VertexPtr& v) const;
    std::optional<VertexPtr> connectTree(const VertexPtr& vNew,
                                         TreeTuple& treeTuple) const;
    VertexPtr mergeTree(Tree* tree1, const VertexPtr& v1, const VertexPtr& v2);
    PlannerSolution recordToSolution() const;
    void generateRecord();

    BiRRTParam param_;
    NearestNeighborBasePtr sNearest_;
    NearestNeighborBasePtr gNearest_;
    Tree sTree_;
    Tree gTree_;
    State currentGoal_;
    Record record_;
    PlannerRecord::VisualFunction vf_;
};
} // namespace gsmpl
