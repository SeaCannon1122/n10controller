#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "platform.h"

#undef None
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static int running = true;

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

void node_thread_funtion() {
  rclcpp::spin(std::make_shared<MinimalPublisher>());
}

int main(int argc, char * argv[])
{

  platform_init();


  int window = window_create(100, 100, 500, 300, (unsigned char*)"n10controller");

  rclcpp::init(argc, argv);

  void* node_thread = create_thread((void(*)(void*))node_thread_funtion, NULL);

  while(get_key_state(KEY_ESCAPE) == 0 && window_is_active(window)) {
    window_poll_events();

    sleep_for_ms(100);
  }

  rclcpp::shutdown();

  join_thread(node_thread);

  window_destroy(window);

  platform_exit();

  return 0;
}