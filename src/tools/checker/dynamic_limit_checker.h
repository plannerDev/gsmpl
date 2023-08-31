#pragma once

#include "state_checker_base.h"
#include "../../utility/class_forward.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(DynamicLimitChecker)

class DynamicLimitChecker : public StateCheckerBase
{
public:
    DynamicLimitChecker() : StateCheckerBase() {}
    bool isValid(const State& q) override { return true; };
};
} // namespace gsmpl
