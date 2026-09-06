#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class SerialPortPublisher : public rclcpp::Node {
public:
  SerialPortPublisher() : Node("xycar_ultrasonic"), serial_fd_(-1) {
    declare_parameter<std::string>("port", "/dev/ttySonic");
    const auto port = get_parameter("port").as_string();
    open_serial(port);
    publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "xycar_ultrasonic", 10);
    timer_ = create_wall_timer(33ms, std::bind(&SerialPortPublisher::poll, this));
    RCLCPP_INFO(get_logger(), "Ultrasonic serial opened: %s at 115200 8N1", port.c_str());
  }

  ~SerialPortPublisher() override {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  void open_serial(const std::string & port) {
    serial_fd_ = open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      throw std::runtime_error(
        "failed to open " + port + ": " + std::strerror(errno));
    }

    termios settings{};
    if (tcgetattr(serial_fd_, &settings) != 0) {
      const auto error = std::string(std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      throw std::runtime_error("tcgetattr failed: " + error);
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CRTSCTS;
    if (tcsetattr(serial_fd_, TCSANOW, &settings) != 0) {
      const auto error = std::string(std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      throw std::runtime_error("tcsetattr failed: " + error);
    }
    tcflush(serial_fd_, TCIFLUSH);
  }

  static bool parse_frame(
    const std::string & frame,
    std::vector<int32_t> & values)
  {
    std::istringstream stream(frame);
    std::string token;
    values.clear();
    while (std::getline(stream, token, ',') && values.size() < 8) {
      while (!token.empty() && std::isspace(
          static_cast<unsigned char>(token.front())))
      {
        token.erase(token.begin());
      }
      if (token.empty() || !std::isdigit(
          static_cast<unsigned char>(token.front())))
      {
        return false;
      }
      try {
        const auto value = std::stoi(token);
        values.push_back(value >= 0 && value <= 140 ? value : 0);
      } catch (const std::exception &) {
        return false;
      }
    }
    return values.size() == 8;
  }

  void poll() {
    std::array<char, 512> chunk{};
    for (;;) {
      const auto count = read(serial_fd_, chunk.data(), chunk.size());
      if (count > 0) {
        receive_buffer_.append(chunk.data(), static_cast<std::size_t>(count));
        continue;
      }
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ultrasonic serial read failed: %s", std::strerror(errno));
      }
      break;
    }

    std::size_t separator = std::string::npos;
    while ((separator = receive_buffer_.find_first_of("\r\n")) !=
      std::string::npos)
    {
      const auto frame = receive_buffer_.substr(0, separator);
      receive_buffer_.erase(0, separator + 1);
      if (frame.empty()) {
        continue;
      }
      std::vector<int32_t> values;
      if (parse_frame(frame, values)) {
        std_msgs::msg::Int32MultiArray message;
        message.data = values;
        publisher_->publish(message);
      }
    }
    if (receive_buffer_.size() > 4096) {
      RCLCPP_WARN(get_logger(), "Discarding malformed ultrasonic serial data");
      receive_buffer_.clear();
    }
  }

  int serial_fd_;
  std::string receive_buffer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SerialPortPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("xycar_ultrasonic"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
