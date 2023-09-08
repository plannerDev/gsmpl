#pragma once

#include "../../robot_algo/fk.h"
#include "distance.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(DistanceSe3)

class EXPORT DistanceSe3 : public DistanceLInfinity {
public:
    DistanceSe3(const Bounds& b, const FKBasePtr& fk)
        : DistanceLInfinity(b), fk_(fk) {}

    double distance(const State& q1, const State& q2) const override;

private:
    FKBasePtr fk_;
};
} // namespace gsmpl
