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

#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "avoidance/Avoidance.hh"
#include "vehicles/MavQuadCopter.hh"

class QuadCopterShiftAvoidance
    : public CollisionAvoidanceStrategy<MavQuadCopter>
{
public:
    QuadCopterShiftAvoidance(std::shared_ptr<MavQuadCopter> quadcopter);
    void avoid(const std::vector<Obstacle> &obstacles) override;

private:
    std::chrono::time_point<std::chrono::system_clock> wp_sent_time =
        std::chrono::system_clock::from_time_t(0);

    enum class avoid_state { looking, moving, detouring };
    avoid_state avoidance_state = avoid_state::moving;

    bool has_obstacle(const std::vector<Obstacle> &obstacles);
};
