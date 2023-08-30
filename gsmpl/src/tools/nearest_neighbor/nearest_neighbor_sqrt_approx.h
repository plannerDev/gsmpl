#pragma once

#include "../../base/math_utility.h"
#include "nearest_neighbor_linear.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(NearestNeighborSqrtApprox)

class NearestNeighborSqrtApprox : public NearestNeighborLinear
{
public:
    NearestNeighborSqrtApprox(const DistanceBasePtr& distance) : NearestNeighborLinear(distance)
    {
    }

    void build(const VertexPtr& root) override;
    void update(const VertexPtr& v) override;
    VertexPtr nearest(const State& q) const override;

private:
    inline void updateCheckCount()
    {
        checks_ = 1 + (std::size_t)floor(sqrt((double)vertexes_.size()));
    }

    std::size_t checks_{0};
    mutable std::size_t offset_{0};
};
} // namespace gsmpl
