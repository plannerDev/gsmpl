#include <assert.h>
#include "gsmpl/base/math_utility.h"
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/tools/distance/distance.h"

namespace gsmpl {
double DistanceLInfinity::distance(const State& q1, const State& q2) const {
    assert(q1.size() == dimension);
    assert(q2.size() == dimension);

    auto q1Mapping = bounds.mapping(q1);
    auto q2Mapping = bounds.mapping(q2);

    assert(q1Mapping);
    assert(q2Mapping);
    const State& q1Value = q1Mapping.value();
    const State& q2Value = q2Mapping.value();

    double distance = 0.0;
    for (size_t i = 0; i < dimension; i++) {
        double residuum = 0;
        switch (bounds[i].type) {
            case JointType::Revolute:
                // TODO bound = 720, compare distance among dis(s1, s2), dis(s1,
                // bound_l), dis(s1, bound_h)
                residuum =
                    abs(normalizeAngle(q2Value[i] - q1Value[i])) * rWeight_;
                // double residuum = abs(qm2[i] - qm1[i]);
                break;
            case JointType::Prismatic:
                residuum = abs(q2Value[i] - q1Value[i]) * pWeight_;
        }
        if (distance < residuum)
            distance = residuum;
    }
    return distance;
}
bool DistanceLInfinity::isEquivalent(const State& q1, const State& q2) const {
    assert(q1.size() == dimension);
    assert(q2.size() == dimension);
    for (std::size_t i = 0; i < dimension; i++) {
        if (q1[i] != q2[i])
            return false;
    }
    return true;
}

double DistanceL2::distance(const State& q1, const State& q2) const {
    assert(q1.size() == dimension);
    assert(q2.size() == dimension);

    auto q1Mapping = bounds.mapping(q1);
    auto q2Mapping = bounds.mapping(q2);

    assert(q1Mapping);
    assert(q2Mapping);
    const State& q1Value = q1Mapping.value();
    const State& q2Value = q2Mapping.value();

    double out = 0;
    double residuum = 0;
    for (size_t i = 0; i < dimension; i++) {
        switch (bounds[i].type) {
            case JointType::Revolute:
                residuum = (normalizeAngle(q2Value[i] - q1Value[i])) * rWeight_;
                break;
            case JointType::Prismatic:
                residuum = abs(q2Value[i] - q1Value[i]) * pWeight_;
        }
        out += residuum * residuum;
    }
    return std::sqrt(out);
}
} // namespace gsmpl
