#pragma once

#include "../../base/state.h"
#include "../../utility/class_forward.h"
#include "../../utility/export.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(StateCheckerBase)

class EXPORT StateCheckerBase {
public:
    virtual ~StateCheckerBase() = default;
    virtual bool isValid(const State& q) { return true; }; // TODO: const
};
} // namespace gsmpl
