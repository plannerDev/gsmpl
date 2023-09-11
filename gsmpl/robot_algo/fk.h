#pragma once

#include <Eigen/Geometry>
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/base/state.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(FKBase)

class FKBase {
public:
    virtual ~FKBase() = default;
    virtual Eigen::Isometry3d tcpPose(const State& q) = 0;
};
} // namespace gsmpl
