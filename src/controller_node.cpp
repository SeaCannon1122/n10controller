#include "controller_node.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <stdio.h>

#include "gui/pixel_char.h"
#include "gui/menu.h"
#include "screen_elements.h"

//globals

struct controller_node_feedback_data node_data;

using namespace std::chrono_literals;

#include "platform.h"

#undef None

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class N10Controller : public rclcpp::Node
{
  public:
    N10Controller() : Node("n10controller") {
      vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10);
      wheels_motor_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10, std::bind(&N10Controller::motor_vel_callback, this, std::placeholders::_1));
      wheels_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_wheels", 10, std::bind(&N10Controller::servo_cmd_wheels_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
      10ms, std::bind(&N10Controller::timer_callback, this));
    }

    void motor_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

      node_data.wheel_speed_0 = msg->data[0];
      node_data.wheel_speed_1 = msg->data[1];
      node_data.wheel_speed_2 = msg->data[2];
      node_data.wheel_speed_3 = msg->data[3];
      node_data.wheel_speed_4 = msg->data[4];
      node_data.wheel_speed_5 = msg->data[5];

    }

    void servo_cmd_wheels_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

      node_data.wheel_angle_0 = msg->data[0];
      node_data.wheel_angle_1 = msg->data[1];
      node_data.wheel_angle_2 = msg->data[2];
      node_data.wheel_angle_3 = msg->data[3];
      node_data.wheel_angle_4 = msg->data[4];
      node_data.wheel_angle_5 = msg->data[5];

    }

  private:
    void timer_callback() {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = node_data.lin_x;
      message.linear.y = node_data.lin_y;
      message.linear.z = 0;

      message.angular.x = 0;
      message.angular.y = 0;
      message.angular.z = node_data.ang_z;
      vel_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheels_motor_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheels_angle_subscriber_;
};


int controller_node_main(int argc, const char* const* argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<N10Controller>();

    int width = 900;
    int height = 700;

    int window = window_create(100, 100, width, height, (unsigned char*)"n10controller");

    width = 1;
    height = 1;

    screen_elements_init();

    unsigned int* pixels = (unsigned int*)malloc(sizeof(unsigned int) * width * height);

    while(get_key_state(KEY_ESCAPE) == 0 && window_is_active(window)) {

        rclcpp::spin_some(node);

        if(get_key_state('W') & 0b1) node_data.lin_x = 1.;
        else if(get_key_state('S') & 0b1) node_data.lin_x = -1.;
        else node_data.lin_x = 0.;
        if(get_key_state('A') & 0b1) node_data.lin_y = 1.;
        else if(get_key_state('D') & 0b1) node_data.lin_y = -1.;
        else node_data.lin_y = 0.;
        if(get_key_state('Q') & 0b1) node_data.ang_z = 1.;
        else if(get_key_state('E') & 0b1) node_data.ang_z = -1.;
        else node_data.ang_z = 0.;
    
    
        int new_width = window_get_width(window);
        int new_height = window_get_height(window);

        if(new_width != width || new_height != height) {
        
            width = new_width;
            height = new_height;
            free(pixels);
            pixels = (unsigned int*)malloc(sizeof(unsigned int) * width * height);
        }


        if(draw_screen_elements(&node_data, pixels, width, height)) window_draw(window, pixels, width, height, 1);

        window_poll_events();

        //rclcpp::spin_some(node);

        sleep_for_ms(2);
    }

    rclcpp::shutdown();

    window_destroy(window);

    return 0;
}