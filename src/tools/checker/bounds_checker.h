#pragma once

#include "assert.h"
#include "state_checker_base.h"
#include "../../base/bounds.h"

namespace gsmpl {
class BoundsChecker : public StateCheckerBase {
public:
    BoundsChecker(const Bounds& b) : bounds_(b) {}
    bool isValid(const State& q) override {
        assert(q.size() == bounds_.size());

        for (std::size_t i = 0; i < q.size(); i++) {
            double l = -bounds_[i].bound + bounds_[i].offset;
            double h = bounds_[i].bound + bounds_[i].offset;
            if (q[i] >= h || q[i] <= l)
                return false;
        }
        return true;
    }

private:
    const Bounds bounds_;
};
} // namespace gsmpl
