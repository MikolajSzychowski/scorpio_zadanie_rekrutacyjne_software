// You can remove or add any includes you need
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include<std_msgs/msg/u_int16.hpp>
#include <cmath>
#include <algorithm>
#include <queue>

//ros2 launch scorpio_zadanie_rekrutacyjne_software platform.launch.py input_file:=src/commands.txt

namespace solution 
{
  static constexpr int ENCODER_RESOLUTION = 4096;
  static constexpr int ALLOWED_ROTATION_ERROR = 2;

  struct Pid 
  {
    double kp = 0.1, ki = 0.0, kd = 0.0;
    double integral = 0.0, prev_error = 0.0;
    rclcpp::Time prev_time;
    bool first = true;

    void reset() 
    { 
      integral = 0.0; 
      first = true; 
    }

    double update(double error, const rclcpp::Time& now)
    {
      if (first) 
      {
        first = false;
        prev_time = now;
        prev_error = error;
        return kp * error;
      }
      const double dt = (now - prev_time).seconds();
      prev_time = now;

      if (dt <= 1e-6) return kp * error;

      integral = integral + error * dt;

      const double derivative = (error - prev_error) / dt;
      prev_error = error;
      return kp * error + ki * integral + kd * derivative;
    }
  };

class Solution final : public rclcpp::Node 
{
public:
  explicit Solution(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("solution", options) {

    // Short example you can remove it
    const auto preempt = declare_parameter<bool>("preempt", false);
    RCLCPP_INFO(get_logger(), "%s", preempt ? "Preempt" : "Queue");

    //std::cout << std::atan2(5, 5) << " " << std::atan2(5, -5) << " " << std::atan2(-5, 5) << " " << std::atan2(-5, -5) << std::endl;
    _motor1 = create_publisher<std_msgs::msg::Int8>("motor_1/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    _motor2 = create_publisher<std_msgs::msg::Int8>("motor_2/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

    _encoder1 = create_subscription<std_msgs::msg::UInt16>("motor_1/encoder", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const std_msgs::msg::UInt16 reading) 
    {
      //std::cout << "Encoder 1: " << reading.data << std::endl;
      encoder_1_reading = reading.data;
      if (!has_target) return;

      const int error = shortest_error(target_encoder_yaw, encoder_1_reading);
      std_msgs::msg::Int8 command;

      command.data = (std::abs(error) <= ALLOWED_ROTATION_ERROR) ? 0 : to_command(pid_yaw.update(error, now()));
      _motor1->publish(command);

    });

    _encoder2 = create_subscription<std_msgs::msg::UInt16>("motor_2/encoder", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const std_msgs::msg::UInt16 reading) 
    {
      //std::cout << "Encoder 2: " << reading.data << std::endl;
      encoder_2_reading = reading.data;
      if (!has_target) return;

      const int error = shortest_error(target_encoder_pitch, encoder_2_reading);
      std_msgs::msg::Int8 command;

      command.data = (std::abs(error) <= ALLOWED_ROTATION_ERROR) ? 0 : to_command(pid_pitch.update(error, now()));
      _motor2->publish(command);

    });

    _target = create_subscription<geometry_msgs::msg::Point>("target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::Point::SharedPtr point) 
      {
        RCLCPP_INFO(get_logger(), "Target: (%g, %g, %g)", point->x, point->y, point->z);
        has_target = true;

        //std::cout << "New target: " << point->x << " " << point->y << " " << point->z << std::endl;

        calculate_target_rotation(point->x, point->y, point->z);

        pid_yaw.reset();
        pid_pitch.reset();
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor1;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor2;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder1;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder2;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _target;
  int target_encoder_yaw = 0, target_encoder_pitch = 0, encoder_1_reading = 0, encoder_2_reading = 0;
  bool has_target = false;
  Pid pid_yaw{2.0, 0.01, 0.001};
  Pid pid_pitch{2.0, 0.01, 0.001};
  std::queue<std::pair<int, int>> target_queue;

  void calculate_target_rotation(double x, double y, double z)
  {
    if (std::sqrt(x * x + y * y + z * z) < 1e-9) 
    {
      return;   
    }

    double h = std::hypot(x, y);

    const double yaw = std::atan2(-y, x); 
    const double pitch = std::atan2(z, h);

    int yaw_a = h < 1e-9 ? encoder_1_reading : angle_to_encoder(yaw);
    int pitch_a = angle_to_encoder(pitch);

    int yaw_b = h < 1e-9 ? encoder_1_reading : angle_to_encoder(yaw + M_PI);
    int pitch_b = angle_to_encoder(M_PI - pitch);


    int error_a = std::max(std::abs(shortest_error(yaw_a, encoder_1_reading)), std::abs(shortest_error(pitch_a, encoder_2_reading)));
    int error_b = std::max(std::abs(shortest_error(yaw_b, encoder_1_reading)), std::abs(shortest_error(pitch_b, encoder_2_reading)));

    if(error_a <= error_b)
    {
      target_encoder_yaw = yaw_a;
      target_encoder_pitch = pitch_a;
    }
    else
    {
      target_encoder_yaw = yaw_b;
      target_encoder_pitch = pitch_b;
    }


    

    RCLCPP_INFO(get_logger(), "Encoder target: yaw=%d pitch=%d",target_encoder_yaw, target_encoder_pitch);
  }

  int reduce_to_encoder(int value)
  {
    return ((value % ENCODER_RESOLUTION) + ENCODER_RESOLUTION) % ENCODER_RESOLUTION;
  }

  int angle_to_encoder(double angle)
  {
    return reduce_to_encoder(static_cast<int>(std::round((angle / (2.0 * M_PI)) * ENCODER_RESOLUTION)));
  }

  int shortest_error(int target, int reading)
  {
    return reduce_to_encoder(target - reading + ENCODER_RESOLUTION / 2) - (ENCODER_RESOLUTION / 2);
  }
  static int8_t to_command(double num)
  {
    return static_cast<int8_t>(std::clamp(std::round(num), -128.0, 127.0));
  }

};

}  // namespace solution

RCLCPP_COMPONENTS_REGISTER_NODE(solution::Solution)
