#include "controller_node.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <stdio.h>

#include "gui/pixel_char.h"
#include "gui/menu.h"
#include "screen_elements.h"

#define M_PI 3.14159265

struct controller_node_feedback_data node_data;

using namespace std::chrono_literals;

#include "platform.h"

#undef None

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class N10Controller : public rclcpp::Node
{
  public:
    N10Controller() : Node("n10controller") {
      vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10);
      wheels_motor_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10, std::bind(&N10Controller::motor_vel_callback, this, std::placeholders::_1));
      wheels_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_wheels", 10, std::bind(&N10Controller::servo_cmd_wheels_callback, this, std::placeholders::_1));

      arm_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/n10/arm_state", 10);
      arm_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/servo_cmd_arm", 10, std::bind(&N10Controller::servo_cmd_arm_callback, this, std::placeholders::_1));

      enable_service_ = create_client<std_srvs::srv::SetBool>("/eduard/enable");

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

    void servo_cmd_arm_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {


    }

    void timer_callback() {
      auto twist_message = geometry_msgs::msg::Twist();
      twist_message.linear.x = node_data.lin_x * node_data.lin_factor;
      twist_message.linear.y = node_data.lin_y * node_data.lin_factor;

      twist_message.angular.x = 0;
      twist_message.angular.y = 0;
      twist_message.angular.z = node_data.ang_z * node_data.ang_factor;
      vel_publisher_->publish(twist_message);

      auto arm_angle_msg = std_msgs::msg::Float32MultiArray();
      arm_angle_msg.data.resize(4); 

      arm_angle_msg.data[0] = node_data.arm_x;
      arm_angle_msg.data[1] = node_data.arm_y;
      arm_angle_msg.data[2] = node_data.ground_angle;
      arm_angle_msg.data[3] = 0;
      arm_publisher_->publish(arm_angle_msg);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheels_motor_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheels_angle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_subscriber_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_service_;
};


int controller_node_main(int argc, const char* const* argv) {

    int frame_count = 0;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<N10Controller>();

    int width =1100;
    int height = 700;

    int window = window_create(100, 100, width, height, (unsigned char*)"n10controller");

    width = 1;
    height = 1;

    screen_elements_init();

    unsigned int* pixels = (unsigned int*)malloc(sizeof(unsigned int) * width * height);

    while(get_key_state(KEY_ESCAPE) == 0 && window_is_active(window)) {

        rclcpp::spin_some(node);

        if(node_data.arm_mode) {
          if(get_key_state('G') == 0b11) node_data.arm_mode = 0;

          if(get_key_state('D') & 0b1) {
            if(node_data.arm_x + 0.001 <= 0.3001) node_data.arm_x += 0.001;
          }
          else if(get_key_state('A') & 0b1) {
            if(node_data.arm_x - 0.001 >= -0.0501) node_data.arm_x -= 0.001;
          }

          if(get_key_state('W') & 0b1) {
            if(node_data.arm_y + 0.001 <= 0.3001) node_data.arm_y += 0.001;
          }
          else if(get_key_state('S') & 0b1) {
            if(node_data.arm_y - 0.001 >= -0.1501) node_data.arm_y -= 0.001;
          }

          if(get_key_state('E') & 0b1) {
            if(node_data.ground_angle + 0.01 <= M_PI / 2) node_data.ground_angle += 0.01;
          }
          else if(get_key_state('Q') & 0b1) {
            if(node_data.ground_angle - 0.01 >= -M_PI / 2) node_data.ground_angle -= 0.01;
          }

        }

        else {

          if(get_key_state('G') == 0b11) node_data.arm_mode = 1;

          if(get_key_state('W') & 0b1) node_data.lin_x = 1.;
          else if(get_key_state('S') & 0b1) node_data.lin_x = -1.;
          else node_data.lin_x = 0.;
          if(get_key_state('A') & 0b1) node_data.lin_y = 1.;
          else if(get_key_state('D') & 0b1) node_data.lin_y = -1.;
          else node_data.lin_y = 0.;
          if(get_key_state('Q') & 0b1) node_data.ang_z = 1.;
          else if(get_key_state('E') & 0b1) node_data.ang_z = -1.;
          else node_data.ang_z = 0.;

          if(get_key_state('2') == 0b11) {
            if(node_data.lin_factor + 0.05 <= 1.02) node_data.lin_factor += 0.05;
          }
          else if(get_key_state('1') == 0b11) {
            if(node_data.lin_factor - 0.05 >= -0.02) node_data.lin_factor -= 0.05;
          }

          if(get_key_state('4') == 0b11) {
            if(node_data.ang_factor + 0.05 <= 1.02) node_data.ang_factor += 0.05;
          }
          else if(get_key_state('3') == 0b11) {
            if(node_data.ang_factor - 0.05 >= -0.02) node_data.ang_factor -= 0.05;
          }

          bool f_state = get_key_state('F') == 0b11;
          bool r_state = get_key_state('R') == 0b11;

          if(f_state || r_state) {

            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

            if(f_state) request->data = true;
            else request->data = false;
            
            auto future = node->enable_service_->async_send_request(request);

            bool success = false;
            switch (future.wait_for(1s)) {
            
            case std::future_status::ready:
              std::cout << future.get()->message << std::endl;
              success = future.get()->success;
              break;
            case std::future_status::deferred: std::cerr << "Request deferred" << std::endl; break;
            case std::future_status::timeout: std::cerr << "Request timeout" << std::endl; break;
            
            }
          }

        }

        
    
        int new_width = window_get_width(window);
        int new_height = window_get_height(window);

        if(new_width != width || new_height != height) {
        
            width = new_width;
            height = new_height;
            free(pixels);
            pixels = (unsigned int*)malloc(sizeof(unsigned int) * width * height);
        }


        if(draw_screen_elements(&node_data, pixels, width, height)) {
          window_draw(window, pixels, width, height, 1);
          printf("frame %d\n", frame_count);
          frame_count++;
        }

        window_poll_events();

        rclcpp::spin_some(node);

        sleep_for_ms(3);
    }

    rclcpp::shutdown();

    window_destroy(window);

    return 0;
}