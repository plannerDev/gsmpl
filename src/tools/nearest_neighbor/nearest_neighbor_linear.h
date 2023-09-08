#pragma once

#include "../distance/distance.h"
#include "nearest_neighbor.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(NearestNeighborLinear)

class NearestNeighborLinear : public NearestNeighborBase {
public:
    NearestNeighborLinear(const DistanceBasePtr& distance)
        : NearestNeighborBase(), distance_(distance) {}

    std::size_t size() const override { return vertexes_.size(); }
    void build(const VertexPtr& root) override;
    void update(const VertexPtr& v) override;
    VertexPtr nearest(const State& q) const override;
    std::vector<VertexPtr> nearestK(const State& q,
                                    std::size_t k) const override;
    std::vector<VertexPtr> nearestR(const State& q, double r) const override;

protected:
    struct ElemSort {
        ElemSort(const State& e, const DistanceBasePtr& df) : e_(e), df_(df) {}

        bool operator()(const VertexPtr& a, const VertexPtr& b) const {
            return df_->distance(a->state, e_) < df_->distance(b->state, e_);
        }

        const State e_;
        const DistanceBasePtr df_;
    };

    const DistanceBasePtr distance_;
    std::vector<VertexPtr> vertexes_;
    VertexPtr root_;
};
} // namespace gsmpl
