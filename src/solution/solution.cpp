// You can remove or add any includes you need
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include<std_msgs/msg/u_int16.hpp>
#include <cmath>
#include <algorithm>
#include <queue>

namespace solution 
{
  static constexpr int ENCODER_RESOLUTION = 4096;
  static constexpr int ALLOWED_ROTATION_ERROR = 10;
  static constexpr bool CONSTRAINT_CAMERA_UP = false;
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
    preempt = declare_parameter<bool>("preempt", false);
    RCLCPP_INFO(get_logger(), "%s", preempt ? "Preempt" : "Queue");

    _motor1 = create_publisher<std_msgs::msg::Int8>("motor_1/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    _motor2 = create_publisher<std_msgs::msg::Int8>("motor_2/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

    _encoder1 = create_subscription<std_msgs::msg::UInt16>("motor_1/encoder", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const std_msgs::msg::UInt16 reading) 
    {
      encoder_1_reading = reading.data;
      check_reached();

      std_msgs::msg::Int8 command;
      command.data = 0;

      if (!target_queue.empty())
      {
        const int error = shortest_error(target_queue.front().first, encoder_1_reading);
        command.data = (std::abs(error) <= ALLOWED_ROTATION_ERROR) ? 0 : to_command(pid_yaw.update(error, now()));
      }
      
      _motor1->publish(command);

    });

    _encoder2 = create_subscription<std_msgs::msg::UInt16>("motor_2/encoder", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const std_msgs::msg::UInt16 reading) 
    {
      encoder_2_reading = reading.data;
      check_reached();

      std_msgs::msg::Int8 command;
      command.data = 0;

      if (!target_queue.empty())
      {
        const int error = shortest_error(target_queue.front().second, encoder_2_reading);
        command.data = (std::abs(error) <= ALLOWED_ROTATION_ERROR) ? 0 : to_command(pid_pitch.update(error, now()));
      }
   
      _motor2->publish(command);

    });

    _target = create_subscription<geometry_msgs::msg::Point>("target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::Point::SharedPtr point) 
      {
        RCLCPP_INFO(get_logger(), "Target: (%g, %g, %g)", point->x, point->y, point->z);

        calculate_target_rotation(point->x, point->y, point->z);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor1;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor2;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder1;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder2;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _target;
  int target_encoder_yaw = 0, target_encoder_pitch = 0, encoder_1_reading = 0, encoder_2_reading = 0;
  Pid pid_yaw{2.0, 0.01, 0.001};
  Pid pid_pitch{2.0, 0.01, 0.001};
  std::queue<std::pair<int, int>> target_queue;
  bool preempt = false;

  void check_reached()
  {
    if (target_queue.size() < 2) return; 
    if (std::abs(shortest_error(target_queue.front().first, encoder_1_reading)) > ALLOWED_ROTATION_ERROR) return;
    if (std::abs(shortest_error(target_queue.front().second, encoder_2_reading)) > ALLOWED_ROTATION_ERROR) return;

    target_queue.pop();
    pid_yaw.reset();
    pid_pitch.reset();
  }

  void calculate_target_rotation(double x, double y, double z)
  {
    if (std::sqrt(x * x + y * y + z * z) < 1e-9) 
    {
      return;   
    }

    const std::pair<int, int> start_point = target_queue.empty() || preempt ? std::make_pair(encoder_1_reading, encoder_2_reading) : target_queue.back();

    double h = std::hypot(x, y);

    const double yaw = std::atan2(-y, x); 
    const double pitch = std::atan2(z, h);

    int yaw_a = h < 1e-9 ? start_point.first : angle_to_encoder(yaw);
    int pitch_a = angle_to_encoder(pitch);

    int yaw_b = h < 1e-9 ? start_point.first : angle_to_encoder(yaw + M_PI);
    int pitch_b = angle_to_encoder(M_PI - pitch);


    int error_a = std::max(std::abs(shortest_error(yaw_a, start_point.first)), std::abs(shortest_error(pitch_a, start_point.second)));
    int error_b = std::max(std::abs(shortest_error(yaw_b, start_point.first)), std::abs(shortest_error(pitch_b, start_point.second)));

    if(CONSTRAINT_CAMERA_UP)
    {
      error_b = error_a;
    }

    if(preempt)
    {
      while(!target_queue.empty())
      {
        target_queue.pop();
      }
    }

    if(target_queue.empty())
    {
      pid_yaw.reset();
      pid_pitch.reset();
    }

    if(error_a <= error_b)
    {
      target_queue.push({yaw_a, pitch_a});
    }
    else
    {
      target_queue.push({yaw_b, pitch_b});
    }
    RCLCPP_INFO(get_logger(), "Encoder target: yaw=%d pitch=%d",target_queue.back().first, target_queue.back().second);
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
