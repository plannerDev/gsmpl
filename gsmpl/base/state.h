#pragma once

#include "gsmpl/utility/class_forward.h"
#include "gsmpl/utility/export.h"

#include <vector>
#include <iostream>

namespace gsmpl {
GSMPL_STRUCT_FORWARD(State)

enum class JointType { Revolute, Prismatic };

struct EXPORT State {
    State(const std::vector<double>& p)
        : position(p),
          velocity(p.size(), 0),
          acceleration(p.size(), 0),
          torque(p.size(), 0) {}
    State(size_t size) { reserve(size); }
    State() = default;

    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> torque;

    void resize(std::size_t size) {
        position.resize(size);
        velocity.resize(size);
        acceleration.resize(size);
        torque.resize(size);
    }
    void reserve(size_t size) {
        position.reserve(size);
        velocity.reserve(size);
        acceleration.reserve(size);
        torque.reserve(size);
    }

    double operator[](std::size_t index) const { return position[index]; }
    void setVelZero() { velocity = std::vector<double>(position.size(), 0); }
    void setAccZero() {
        acceleration = std::vector<double>(position.size(), 0);
    }
    void setTorqueZero() { torque = std::vector<double>(position.size(), 0); }
    void setVelAccTorqueZero() {
        setVelZero();
        setAccZero();
        setTorqueZero();
    }

    void printState(const std::string& desc = {}) const {
        std::cout << desc << std::endl;
        std::cout << "position" << std::endl;
        for (const auto& p : position)
            std::cout << p << ", ";
        std::cout << std::endl;
        std::cout << "vel" << std::endl;
        for (const auto& vel : velocity)
            std::cout << vel << ", ";
        std::cout << std::endl;
        std::cout << "acc" << std::endl;
        for (const auto& acc : acceleration)
            std::cout << acc << ", ";
        std::cout << std::endl;
    }

    std::size_t size() const { return position.size(); }
};
} // namespace gsmpl
