#include <limits.h>
#include <chrono>
#include <assert.h>
#include "../../utility/log_utility.h"
#include "nearest_neighbor_linear.h"

namespace gsmpl {
void NearestNeighborLinear::build(const VertexPtr& root)
{
    vertexes_.clear();
    vertexes_.push_back(root);
    root_ = root;
}

void NearestNeighborLinear::update(const VertexPtr& v) { vertexes_.push_back(v); }

VertexPtr NearestNeighborLinear::nearest(const State& q) const
{
    auto startTime = std::chrono::steady_clock::now();
    assert(vertexes_.size() >= 1);
    double cost = std::numeric_limits<double>::max();
    unsigned minIndex = 0;
    for (size_t i = 0; i < vertexes_.size(); i++) {
        double dis = distance_->distance(vertexes_[i]->state, q);
        if (cost > dis) {
            cost = dis;
            minIndex = i;
        }
    }
    // LOGD("nearestVertex cost: %f, index: %d", cost, minIndex);
    auto endTime = std::chrono::steady_clock::now();
    time_ += std::chrono::duration<double>(endTime - startTime).count();
    times_++;
    return vertexes_[minIndex];
}

std::vector<VertexPtr> NearestNeighborLinear::nearestK(const State& q, std::size_t k) const
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<VertexPtr> out = vertexes_;
    if (out.size() > k) {
        std::partial_sort(out.begin(), out.begin() + k, out.end(), ElemSort(q, distance_));
        out.resize(k);
    } else {
        std::sort(out.begin(), out.end(), ElemSort(q, distance_));
    }

    auto endTime = std::chrono::steady_clock::now();
    time_ += std::chrono::duration<double>(endTime - startTime).count();
    times_++;

    return out;
}
std::vector<VertexPtr> NearestNeighborLinear::nearestR(const State& q, double r) const
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<VertexPtr> out;
    for (const auto& v : vertexes_)
        if (distance_->distance(v->state, q) <= r)
            out.push_back(v);
    std::sort(out.begin(), out.end(), ElemSort(q, distance_));

    auto endTime = std::chrono::steady_clock::now();
    time_ += std::chrono::duration<float>(endTime - startTime).count();
    times_++;

    return out;
}
} // namespace gsmpl
