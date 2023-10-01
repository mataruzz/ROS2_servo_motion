#ifndef MICRO_SERVO_HARDWARE__S90_SERVO_HPP_
#define MICRO_SERVO_HARDWARE__S90_SERVO_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.hpp"

#include "micro_servo.hpp"

namespace micro_servo_hardware
{
class S90ServoSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(S90ServoSystemPositionOnlyHardware)

    MICRO_SERVO_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    
    MICRO_SERVO_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    MICRO_SERVO_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MICRO_SERVO_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MICRO_SERVO_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    MICRO_SERVO_HARDWARE_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

    MICRO_SERVO_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    MICRO_SERVO_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the S90 servo simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<int> GPIO_PINs;

  std::vector<std::unique_ptr<microServo>> servos;
}; 
} // namespace micro_servo_hardware



#endif // MICRO_SERVO_HARDWARE__S90_SERVO_HPP_