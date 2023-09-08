#pragma once

#include <vector>
#include "../../utility/class_forward.h"
#include "../../utility/export.h"
#include "../../utility/log_utility.h"
#include "state_checker_base.h"
#include "dynamic_limit_checker.h"
#include "bounds_checker.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(StateCheckerGroup)

class EXPORT StateCheckerGroup : public StateCheckerBase {
public:
    StateCheckerGroup(const Bounds& b) {
        checkers_.push_back(std::make_shared<BoundsChecker>(b));
        checkers_.push_back(std::make_shared<DynamicLimitChecker>());
    }

    void addChecker(const StateCheckerBasePtr& checker) {
        checkers_.push_back(checker);
    }

    bool isValid(const State& q) override {
        for (auto& checker : checkers_)
            if (!checker->isValid(q))
                return false;
        return true;
    }

private:
    std::vector<StateCheckerBasePtr> checkers_;
};
} // namespace gsmpl
