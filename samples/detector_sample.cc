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

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include <coav/coav.hh>

using namespace std;

class DepthImageSimpleDetector : public Detector<DepthCamera>
{
  public:
    DepthImageSimpleDetector(shared_ptr<DepthCamera> depth_camera, double threshold_m = 1.5);

    const vector<Obstacle> &detect() override;

  private:
    vector<Obstacle> obstacles = {};
    double threshold;
};

DepthImageSimpleDetector::DepthImageSimpleDetector(
    shared_ptr<DepthCamera> depth_camera,
    double threshold_m)
{
    this->sensor = depth_camera;
    this->threshold = threshold_m;
}

const vector<Obstacle> &DepthImageSimpleDetector::detect()
{
    // Obtain camera depth buffer and camera properties
    vector<uint16_t> depth_buffer = this->sensor->get_depth_buffer();

    unsigned int height = this->sensor->get_height();
    unsigned int width = this->sensor->get_width();

    double scale = this->sensor->get_scale();

    double hfov = this->sensor->get_horizontal_fov();
    double vfov = this->sensor->get_vertical_fov();
    double base_phi = (M_PI - hfov) / 2;
    double base_theta = (M_PI - vfov) / 2;

    this->obstacles.clear();

    // Return if depth buffer is empty
    if(depth_buffer.size() == 0) {
        return this->obstacles;
    }

    // Detect "obstacles" in the image
    unsigned int min = UINT16_MAX;
    unsigned int min_i = 0, min_j = 0;

    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            uint16_t depth_value = depth_buffer[i * width + j];
            if (depth_value == 0 ||
                depth_value >= this->threshold / scale) {
                continue;
            }

            // If no obstacle was found yet, create a new one right in
            // front of the vehicle. The distance will be set later.
            if (obstacles.size() == 0) {
                Obstacle obs = {0, glm::dvec3(0, 0, 0)};
                this->obstacles.push_back(obs);
            }

            // save the minimum value
            if (depth_value < min) {
                min = depth_value;
                min_i = i;
                min_j = j;
            }
        }
    }

    // If an obstacle was found, set its distance to the minimum distance
    if (obstacles.size() != 0) {
        obstacles[0].center.x = (double) min * scale;

        // Cartesian to spherical
        obstacles[0].center.y = (((double)min_i / height) * vfov) + base_theta;
        obstacles[0].center.z = ((1.0 - ((double)min_j / width)) * hfov) + base_phi;
    }

    return this->obstacles;
}

int main(int argc, char **argv)
{
    // Initialize Depth Camera
    shared_ptr<DepthCamera> depth_camera =
        make_shared<RealSenseCamera>(640, 480, 30);

    // Initialize Detector
    shared_ptr<DepthImageObstacleDetector> obstacle_detector =
        make_shared<DepthImageObstacleDetector>(depth_camera, 5.0);

    while (true) {

        // Sense
        auto sensed_elements = obstacle_detector->detect();

        // Print found Obstacle information
        if (sensed_elements.size()) {
            for (Obstacle o : sensed_elements) {
                cout << "Closest [r = " << o.center.x <<
                    " | theta = " << o.center.y <<
                    " | phi = " << o.center.z <<
                    " ]" << endl;
            }
        } else {
                cout << "No Obstacle" << endl;
        }
    }
}

