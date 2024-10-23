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

struct controller_node_data node_data;


#include "platform.h"

#undef None

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher() : Node("minimal_publisher") {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/n10/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
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
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

void node_thread_funtion() {
  rclcpp::spin(std::make_shared<MinimalPublisher>());
}

int controller_node_main(int argc, const char* const* argv) {

    int width = 900;
    int height = 700;

    int window = window_create(100, 100, width, height, (unsigned char*)"n10controller");

    rclcpp::init(argc, argv);

    void* node_thread = create_thread((void(*)(void*))node_thread_funtion, NULL);

    width = 1;
    height = 1;

    screen_elements_init();

    unsigned int* pixels = (unsigned int*)malloc(sizeof(unsigned int) * width * height);

    while(get_key_state(KEY_ESCAPE) == 0 && window_is_active(window)) {
    
        if(get_key_state('W') & 0b1) node_data.lin_x = 1.;
        else if(get_key_state('S') & 0b1) node_data.lin_x = -1.;
        else node_data.lin_x = 0.;
        if(get_key_state('A') & 0b1) node_data.lin_y = 1.;
        else if(get_key_state('D') & 0b1) node_data.lin_y = -1.;
        else node_data.lin_y = 0.;
    
    
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
        sleep_for_ms(5);
    }

    rclcpp::shutdown();

    join_thread(node_thread);

    window_destroy(window);

    return 0;
}