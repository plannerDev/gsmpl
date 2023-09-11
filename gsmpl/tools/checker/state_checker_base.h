#pragma once

#include "gsmpl/base/state.h"
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/utility/export.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(StateCheckerBase)

class EXPORT StateCheckerBase {
public:
    virtual ~StateCheckerBase() = default;
    virtual bool isValid(const State& q) { return true; }; // TODO: const
};
} // namespace gsmpl
