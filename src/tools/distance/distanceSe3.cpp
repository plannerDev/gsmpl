#include "../../base/math_utility.h"
#include "../../utility/log_utility.h"
#include "distanceSe3.h"

namespace gsmpl {
double DistanceSe3::distance(const State& q1, const State& q2) const {
    assert(q1.size() == dimension);
    assert(q2.size() == dimension);

    Eigen::Isometry3d p1 = fk_->tcpPose(q1);
    Eigen::Isometry3d p2 = fk_->tcpPose(q2);
    double trans = (p2.translation() - p1.translation()).norm();
    Eigen::Quaterniond quat1(p1.rotation());
    Eigen::Quaterniond quat2(p2.rotation());
    double angle = abs(quat1.angularDistance(quat2));

    return trans + angle;
}
} // namespace gsmpl
