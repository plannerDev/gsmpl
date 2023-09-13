#pragma once

#include <map>
#include <iostream>
#include "gsmpl/base/path.h"
#include "gsmpl/tools/local_planner/joint_interpolate.h"

namespace gsmpl {
class Trajectory {
public:
    using Waypoints = std::vector<State>;

    Trajectory(const LocalPlannerBasePtr& local_planner = nullptr)
        : local_planner_(local_planner) {}
    Trajectory(const LocalPlannerBasePtr& local_planner, const Path& path,
               double step_size_jps, double stepSizeTcp, double dt)
        : local_planner_(local_planner) {
        generateTrajector(path, step_size_jps, stepSizeTcp, dt);
    }

    Waypoints waypoints;
    Waypoints dense_waypoints;

    void print() const {
        std::cout << "waypoints " << waypoints.size() << " dense_waypoints "
                  << dense_waypoints.size() << std::endl;
    }

    void generateTrajector(const Path& path, double step_size_jps,
                           double stepSizeTcp, double dt) {
        assert(path.size() > 1);
        assert(local_planner_);

        waypoints = path;
        dense_waypoints.clear();
        dense_waypoints.push_back(path[0]);
        for (size_t i = 1; i < path.size(); i++) {
            Waypoints segment = local_planner_->interpolatePath(
                path[i - 1], path[i], step_size_jps, stepSizeTcp);
            dense_waypoints.insert(dense_waypoints.end(), segment.begin(),
                                   segment.end());
        }
        dense_waypoints.push_back(path.back());
        dense_waypoints.back().setVelAccTorqueZero();

        for (size_t i = 1; i < dense_waypoints.size(); i++) {
            State& q1 = dense_waypoints[i - 1];
            State& q2 = dense_waypoints[1];
            q1.velocity.clear();
            for (size_t j = 0; j < q1.size(); j++) {
                q1.velocity.push_back((q2[j] - q1[j]) / dt);
            }
        }
        for (size_t i = 1; i < dense_waypoints.size(); i++) {
            State& q1 = dense_waypoints[i - 1];
            State& q2 = dense_waypoints[i];
            q1.acceleration.clear();
            for (size_t j = 0; j < q1.size(); j++)
                q1.acceleration.push_back((q2.velocity[j] - q1.velocity[j]) /
                                          dt);
        }
    }

private:
    LocalPlannerBasePtr local_planner_;
};
} // namespace gsmpl
