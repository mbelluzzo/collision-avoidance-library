/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <limits>
#include <iostream>

#include <glm/glm.hpp>

#include "avoidance/QuadCopterShiftAvoidance.hh"
#include "common/common.hh"
#include "common/math.hh"

namespace defaults
{
const double safe_distance = 8.0;
const double lowest_altitude = 1.0;
const double detour_wp_angle = M_PI/3.0;
}

QuadCopterShiftAvoidance::QuadCopterShiftAvoidance(
    std::shared_ptr<MavQuadCopter> quadcopter)
{
    this->vehicle = quadcopter;
}

bool QuadCopterShiftAvoidance::has_obstacle(const std::vector<Obstacle> &obstacles)
{
    double closest;

    if (obstacles.size()) {
        closest = defaults::safe_distance + 1;
        for (Obstacle obs : obstacles) {
            if (obs.center.x < closest) {
                closest = obs.center.x;
            }
        }

        if (closest > defaults::safe_distance) {
            return false;
        }

        // Check if the current mission waypoint is closer than the closest obstacle
        if (mavlink_vehicles::math::ground_dist(
                vehicle->mav->get_global_position_int(),
                vehicle->mav->get_mission_waypoint()) <= closest) {
            return true;
        }
    }

    return false;
}

void QuadCopterShiftAvoidance::avoid(const std::vector<Obstacle> &obstacles)
{
    Pose vehicle_pose = vehicle->vehicle_pose();

    if (!vehicle->mav->is_ready()) {
        return;
    }

    vehicle->mav->set_autorotate_during_mission(true);
    vehicle->mav->set_autorotate_during_detour(false);

    switch (this->avoidance_state) {
    case avoid_state::looking:
        if (vehicle->mav->is_rotation_active()) {
            std::cout << "[avoid] waiting rotation..." << std::endl;
            break;
        }
        std::cout << "[avoid] rotation finished" << std::endl;

        if (this->has_obstacle(obstacles)) {
            std::cout << "[avoid] state = looking around once more..." << std::endl;
            vehicle->mav->rotate(defaults::detour_wp_angle, false); //TODO: ir no max ate 60, abortar depois disso
        } else {
            // Calculate the lookat vector
            glm::dvec3 wp_dir = glm::normalize(
                glm::dvec3(-sin(vehicle_pose.yaw()), cos(vehicle_pose.yaw()), 0));

            // Calculate the global position of the waypoint
            Pose wp = {
                vehicle_pose.pos + glm::abs(defaults::safe_distance) * wp_dir,
                glm::dquat(0, 0, 0, 0) };

            // Send the detour waypoint to the vehicle
            vehicle->set_target_pose(wp);

            // Print info
            std::cout << "[avoid] state = detouring..." << std::endl;

            std::cout << "[avoid] from (x, y, z): (" <<
                vehicle_pose.pos.x << ", " <<
                vehicle_pose.pos.y << ", " <<
                vehicle_pose.pos.z << ")" << std::endl;

            std::cout << "[avoid] to   (x, y, z): (" <<
                wp.pos.x << ", " <<
                wp.pos.y << ", " <<
                wp.pos.z << ")" << std::endl;

            this->avoidance_state = avoid_state::detouring;
        }

        break;
    case avoid_state::detouring:
        if(vehicle->detour_finished()) {
            this->avoidance_state = avoid_state::moving;
            vehicle->mav->set_autorotate_during_mission(true);
            std::cout << "[avoid] state = moving..." << std::endl;
        }

        break;
    case avoid_state::moving:
        // Check if we are too close to ground
        if (vehicle_pose.pos.z < defaults::lowest_altitude) {
            break;
        }

        // The vehicle is moving to the target. We need to detect if an
        // obstacle is found in order to start a detour.
        if (!this->has_obstacle(obstacles)) {
            break;
        }

        std::cout << "[avoid] state = Obstacle found. Looking around..." << std::endl;
        vehicle->mav->brake(false);
        vehicle->mav->rotate(defaults::detour_wp_angle, false); //TODO: 30 graus
        this->avoidance_state = avoid_state::looking;
        break;
    }

    return;
}

