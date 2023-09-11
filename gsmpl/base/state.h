#pragma once

#include "gsmpl/utility/class_forward.h"
#include "gsmpl/utility/export.h"

#include <vector>
#include <iostream>

namespace gsmpl {
GSMPL_STRUCT_FORWARD(State)

enum class JointType { Revolute, Prismatic };

struct EXPORT State {
    State(const std::vector<double>& v) : values(v) {}
    State(size_t size) { reserve(size); }
    State() = default;

    std::vector<double> values;

    void push_back(double v) { values.push_back(v); }
    void resize(std::size_t size) { values.resize(size); }
    void reserve(size_t size) { values.reserve(size); }

    double operator[](std::size_t index) const { return values[index]; }

    void printState(const std::string& desc = {}) const {
        std::cout << desc << std::endl;
        for (const auto& v : values)
            std::cout << v << ", ";
        std::cout << std::endl;
    }

    std::size_t size() const { return values.size(); }
};
} // namespace gsmpl
