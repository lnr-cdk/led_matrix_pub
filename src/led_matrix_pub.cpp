#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "custom_8x8_led_matrix/msg/custom8x8_led_matrix_msg.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LedMatrixPub : public rclcpp::Node
{
public:
  LedMatrixPub()
      : Node("led_matrix_pub")
  {
    publisher_ = this->create_publisher<custom_8x8_led_matrix::msg::Custom8x8LedMatrixMsg>("/led_matrix", 10);
    init_dictionary();
    input_loop_thread_ = std::thread(&LedMatrixPub::input_loop, this);
  }
  ~LedMatrixPub()
  {
    if (input_loop_thread_.joinable())
    {
      input_loop_thread_.join();
    }
  }

private:
  rclcpp::Publisher<custom_8x8_led_matrix::msg::Custom8x8LedMatrixMsg>::SharedPtr publisher_;
  std::unordered_map<std::string, std::array<uint8_t, 8>> dictionary_;
  std::thread input_loop_thread_;

  void init_dictionary()
  {
    dictionary_["smile"] = {
        0b00111100,
        0b01000010,
        0b10100101,
        0b10000001,
        0b10100101,
        0b10011001,
        0b01000010,
        0b00111100};

    dictionary_["heart"] = {
        0b00001010,
        0b00011111,
        0b00111111,
        0b00111110,
        0b00111110,
        0b00011100,
        0b00001000,
        0b00000000};

    dictionary_["cross"] = {
        0b10000001,
        0b01000010,
        0b00100100,
        0b00011000,
        0b00011000,
        0b00100100,
        0b01000010,
        0b10000001};
  }

  void input_loop()
  {
    std::string input;
    while (rclcpp::ok())
    {
      std::cout << "Enter command: ";
      std::getline(std::cin, input);

      if (dictionary_.find(input) != dictionary_.end())
      {
        auto msg = custom_8x8_led_matrix::msg::Custom8x8LedMatrixMsg();
        msg.rows = dictionary_[input];
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published pattern for '%s'", input.c_str());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", input.c_str());
      }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedMatrixPub>());
  rclcpp::shutdown();
  return 0;
}
