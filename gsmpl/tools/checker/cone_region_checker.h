#pragma once

#include <Eigen/Geometry>

#include "gsmpl/robot_algo/fk.h"
#include "gsmpl/tools/checker/state_checker_base.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(ZConeRegionChecker)

class ZConeRegionChecker : public StateCheckerBase {
public:
    ZConeRegionChecker(const FKBasePtr& fk, double threshold)
        : StateCheckerBase(), threshold_(threshold), fk_(fk) {}

    bool isValid(const State& q) override {
        Eigen::Vector3d zAxis(0, 0, 1);
        Eigen::Vector3d minusZaxis(0, 0, -1);
        Eigen::Isometry3d tcpPose = fk_->tcpPose(q);

        Eigen::Vector3d tcpZAxis = tcpPose.rotation() * zAxis;
        double cos = tcpZAxis.dot(minusZaxis);

        if (cos < -1)
            cos = -1;
        else if (cos > 1)
            cos = 1;

        double angle = std::acos(cos);
        return abs(angle) <= threshold_;
    }

private:
    double threshold_;
    FKBasePtr fk_;
};
} // namespace gsmpl
