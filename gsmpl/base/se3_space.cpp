#include "gsmpl/base/se3_space.h"

namespace gsmpl {
// s1: se3 s2: se3 translation part distance
double Se3Space::distanceTrans(const Eigen::Isometry3d& p1,
                               const Eigen::Isometry3d& p2) const {
    return (p2.translation() - p1.translation()).norm();
}
bool Se3Space::isEquivalent(const Eigen::Isometry3d& p1,
                            const Eigen::Isometry3d& p2) const {
    Eigen::Vector3d v1(p1.translation());
    Eigen::Vector3d v2(p2.translation());
    Eigen::Quaterniond quat1(p1.rotation());
    Eigen::Quaterniond quat2(p2.rotation());
    quat1.normalize();
    quat2.normalize();
    Eigen::Vector4d qv1(quat1.w(), quat1.x(), quat1.y(), quat1.z());
    Eigen::Vector4d qv2(quat2.w(), quat2.x(), quat2.y(), quat2.z());
    if (v1 == v2)
        if (qv1 == qv2 || qv1 == -qv2)
            return true;
    return false;
}
std::optional<Se3Space::SE3Poses> Se3Space::InterpolatePath(
    const Eigen::Isometry3d& p1, const Eigen::Isometry3d& p2,
    double step_size) const {
    SE3Poses poses;
    double distance = distanceTrans(p1, p2);

    unsigned int steps = floor(static_cast<unsigned int>(distance / step_size));
    double deltaTime = 1.0 / (double)steps;
    poses.push_back(p1);
    for (double i = 1; i < steps; i++) {
        auto waypoint = interpolateSe3(p1, p2, i * deltaTime);
        poses.push_back(waypoint);
    }
    poses.push_back(p2);
    return poses;
}

Eigen::Isometry3d Se3Space::interpolateSe3(const Eigen::Isometry3d& p1,
                                           const Eigen::Isometry3d& p2,
                                           double time) const {
    Eigen::Vector3d v1 = p1.translation();
    Eigen::Vector3d v2 = p2.translation();
    Eigen::Vector3d v = (1 - time) * v1 + time * v2;

    Eigen::Quaterniond quat1(p1.rotation());
    Eigen::Quaterniond quat2(p2.rotation());
    Eigen::Quaterniond quat = quat1.slerp(time, quat2);
    auto pose = Eigen::Isometry3d::Identity();
    pose.translate(v);
    pose.rotate(quat);
    return pose;
}

Eigen::Isometry3d Se3Space::state2Se3(const State& q) const {
    assert(q.size() == 7);
    auto pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d v(q[0], q[1], q[2]);
    Eigen::Quaterniond quat(q[3], q[4], q[5], q[6]);
    pose.translate(v);
    pose.rotate(quat);
    return pose;
}

State Se3Space::se32State(const Eigen::Isometry3d& p) const {
    Eigen::Vector3d v = p.translation();
    Eigen::Quaterniond quat(p.rotation());
    return State({v[0], v[1], v[2], quat.w(), quat.x(), quat.y(), quat.z()});
}
} // namespace gsmpl
