#pragma once

#include <optional>
#include <Eigen/Geometry>

#include "gsmpl/base/state.h"

namespace gsmpl {
class Se3Space {
public:
    using SE3Poses = std::vector<Eigen::Isometry3d>;

    // s1: se3 s2: se3 linearPart distance
    double distanceTrans(const Eigen::Isometry3d& p1,
                         const Eigen::Isometry3d& p2) const;
    bool isEquivalent(const Eigen::Isometry3d& p1,
                      const Eigen::Isometry3d& p2) const;
    std::optional<SE3Poses> InterpolatePath(const Eigen::Isometry3d& p1,
                                            const Eigen::Isometry3d& p2,
                                            double step_size) const;
    Eigen::Isometry3d interpolateSe3(const Eigen::Isometry3d& p1,
                                     const Eigen::Isometry3d& p2,
                                     double time) const;

    Eigen::Isometry3d state2Se3(const State& q) const; // TODO vector2Se3
    State se32State(const Eigen::Isometry3d& p) const;
};
} // namespace gsmpl
