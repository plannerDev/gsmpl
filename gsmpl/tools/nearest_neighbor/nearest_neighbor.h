#pragma once

#include "gsmpl/utility/class_forward.h"
#include "gsmpl/base/state.h"
#include "gsmpl/base/tree.h"
#include "gsmpl/utility/global.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(NearestNeighborBase)

class NearestNeighborBase {
public:
    virtual ~NearestNeighborBase() {
        std::cout << " NearestNeighbor Processing time: " << time_
                  << " times: " << times_ << " averageTime: " << time_ / times_
                  << std::endl;
    }

    virtual std::size_t size() const = 0;
    virtual void build(const VertexPtr& root) = 0;
    virtual void update(const VertexPtr& v) = 0;
    virtual VertexPtr nearest(const State& q) const = 0;
    virtual std::vector<VertexPtr> nearestK(const State& q,
                                            std::size_t k) const = 0;
    virtual std::vector<VertexPtr> nearestR(const State& q, double r) const = 0;

protected:
    mutable float time_{0};
    mutable int times_{0};
};
} // namespace gsmpl
