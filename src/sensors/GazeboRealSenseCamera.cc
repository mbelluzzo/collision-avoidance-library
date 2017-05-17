/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo/gazebo_client.hh>

#include "utils/DebugUtils.hh"
#include "sensors/GazeboRealSenseCamera.hh"

// TODO: The camera configuration should be aggregated on the gazebo .sdf model
// file or retrieved from the librealsense. This feature is not implemented
// yet, so those values are stored here until then.
#define GZ_RS_STREAM_DEPTH_TOPIC "~/gzsitl_quadcopter_rs/rs/stream/depth"
#define DEPTH_CAM_WIDTH 640
#define DEPTH_CAM_HEIGHT 480
#define DEPTH_CAM_HFOV M_PI / 3.0
#define DEPTH_CAM_VFOV 0.757608
#define DEPTH_CAM_SCALE 0.001
// =============
// GazeboContext
// =============

class GazeboContext
{
  public:
    ~GazeboContext();
    gazebo::transport::Node *node();
    static std::shared_ptr<GazeboContext> instance();

  private:
    GazeboContext();
};

GazeboContext::GazeboContext()
{
    gazebo::client::setup();
    std::cout << "[GazeboContext] Gazebo client has been set up" << std::endl;
}

GazeboContext::~GazeboContext()
{
    gazebo::client::shutdown();
    std::cout << "[GazeboContext] Gazebo client has been shut down" << std::endl;
}

std::shared_ptr<GazeboContext> GazeboContext::instance() {
    static std::weak_ptr<GazeboContext> _instance;
    if (auto ptr = _instance.lock()) { // .lock() returns a shared_ptr and increments the refcount
        return ptr;
    }
    // Does not support std::make_shared<GazeboContext> because of
    // the Resource private constructor.
    auto ptr = std::shared_ptr<GazeboContext>(new GazeboContext());
    _instance = ptr;
    return ptr;
}

gazebo::transport::Node *GazeboContext::node()
{
    return new gazebo::transport::Node();
}

// =====================
// GazeboRealSenseCamera
// =====================

GazeboRealSenseCamera::GazeboRealSenseCamera()
{
    // Start communication with Gazebo
    std::cout << "[GazeboRealSenseCamera] Waiting for Gazebo..." << std::endl;
    this->gazebo_context = GazeboContext::instance();

    // Create our node for communication
    this->gznode.reset(gazebo_context->node());
    this->gznode->Init();

    std::cout << "[GazeboRealSenseCamera] Gazebo Initialized" << std::endl;

    // TODO: Retrieve camera data straight from topic or camera plugin
    this->width = DEPTH_CAM_WIDTH;
    this->height = DEPTH_CAM_HEIGHT;
    this->hfov = DEPTH_CAM_HFOV;
    this->vfov = DEPTH_CAM_VFOV;
    this->scale = DEPTH_CAM_SCALE;

    // TODO: Find RealSense camera topic and parameters automatically
    this->rs_depth_sub = this->gznode->Subscribe(
        GZ_RS_STREAM_DEPTH_TOPIC, &GazeboRealSenseCamera::on_stream_depth_recvd,
        this);

    visualization(true);
}

GazeboRealSenseCamera::~GazeboRealSenseCamera()
{
}

std::vector<uint16_t> &GazeboRealSenseCamera::get_depth_buffer()
{
    std::lock_guard<std::mutex> locker(depth_buffer_mtx);
    if (this->visualization_on)
        this->visualize();

    return depth_buffer[this->current_buffer ^= 1];
}

void GazeboRealSenseCamera::on_stream_depth_recvd(ConstImageStampedPtr &_msg)
{
    if (!this->camera_exists) {
        this->camera_exists = true;
        std::cout << "[GazeboRealSenseCamera] Real Sense Initialized" << std::endl;
    }

    std::lock_guard<std::mutex> locker(depth_buffer_mtx);

    uint16_t *data = (uint16_t *) _msg->image().data().c_str();
    uint buffer_size = _msg->image().width() * _msg->image().height();
    depth_buffer[this->current_buffer ^ 1] = std::vector<uint16_t>(data, data + buffer_size);
}

void GazeboRealSenseCamera::visualization(bool onoff)
{
    if (!onoff) {
        this->visualization_on = false;
        glfwDestroyWindow(this->win);
        glfwTerminate();
        delete this->frame_buffer;
        return;
    }

    if (!this->win) {
        glfwInit();
        this->win = glfwCreateWindow(this->width, this->height, "GazeboRealSense Depth Buffer", 0, 0);
        this->frame_buffer = new uint8_t[this->width * this->height * 3];

        glGenTextures(1, &this->texture);
        this->visualization_on = true;
    }

    printf("Scale:\n");
    printf("null  : BLACK\n");
    printf("0.01 m: RED\n");
    printf("1.25 m: YELLOW\n");
    printf("2.50 m: GREEN\n");
    printf("3.75 m: CYAN\n");
    printf("5.0+ m: BLUE\n");
}

void GazeboRealSenseCamera::visualize(void)
{
    glfwMakeContextCurrent(this->win);

    glViewport(0, 0, this->width, this->height);
    glClear(GL_COLOR_BUFFER_BIT);

    glPushMatrix();
    glOrtho(0, this->width, this->height, 0, -1, +1);

    glBindTexture(GL_TEXTURE_2D, this->texture);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, this->width);

    for (unsigned int i = 0; i < depth_buffer[current_buffer].size(); i++) {
        uint8_t *rgb = this->frame_buffer + (3 * i);
        rainbow_scale(((double)depth_buffer[current_buffer][i] * this->scale) / 5.0, rgb);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, this->width, this->height, 0, GL_RGB,
        GL_UNSIGNED_BYTE, reinterpret_cast<const GLvoid *>(this->frame_buffer));

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindTexture(GL_TEXTURE_2D, this->texture);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);

    // Order matters :/
    glTexCoord2f(0, 0); glVertex2f(0, 0);
    glTexCoord2f(1, 0); glVertex2f(this->width, 0);
    glTexCoord2f(1, 1); glVertex2f(this->width, this->height);
    glTexCoord2f(0, 1); glVertex2f(0, this->height);

    glEnd();
    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    glPopMatrix();
    glfwSwapBuffers(this->win);
}
