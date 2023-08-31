#include <limits.h>
#include <chrono>
#include <assert.h>
#include "../../utility/log_utility.h"
#include "nearest_neighbor_sqrt_approx.h"

namespace gsmpl {
void NearestNeighborSqrtApprox::build(const VertexPtr& root)
{
    NearestNeighborLinear::build(root);
    updateCheckCount();
}

void NearestNeighborSqrtApprox::update(const VertexPtr& v)
{
    NearestNeighborLinear::update(v);
    updateCheckCount();
}

VertexPtr NearestNeighborSqrtApprox::nearest(const State& q)const
{
    auto startTime = std::chrono::steady_clock::now();
    assert(vertexes_.size() >= 1);
    double cost = std::numeric_limits<double>::max();
    std::size_t n = vertexes_.size();
    unsigned minIndex = 0;
    for (size_t i = 0; i < checks_; i++) {
        std::size_t j = (i * checks_ + offset_) % n;
        double dis = distance_->distance(vertexes_[j]->state, q);
        if (cost > dis) {
            cost = dis;
            minIndex = j;
        }
    }
    offset_ = (offset_ + 1) % checks_;
    LOGD("nearestVertex cost: %f, index: %d", cost, minIndex);
    auto endTime = std::chrono::steady_clock::now();
    time_ += std::chrono::duration<double>(endTime - startTime).count();
    times_++;
    return vertexes_[minIndex];
}
} // namespace gsmpl
