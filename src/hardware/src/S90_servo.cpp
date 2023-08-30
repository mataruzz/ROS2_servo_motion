#include "../include/hardware/S90_servo.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware
{
hardware_interface::CallbackReturn S90ServoSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // S90ServoSystemPositionOnly has exactly one state and command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"),
                "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"),
                "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"),
            "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
        } 
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn S90ServoSystemPositionOnlyHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Configuring ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "%.1f seconds left...",
        hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    
    // reset values always when configuring hardware
    for (uint i = 0; i < hw_states_.size(); i++)
    {
        hw_states_[i] = 0;
        hw_commands_[i] = 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
S90ServoSystemPositionOnlyHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
S90ServoSystemPositionOnlyHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn S90ServoSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "%.1f seconds left...",
        hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // command and state should be equal when starting
    for (uint i = 0; i < hw_states_.size(); i++)
    {
        hw_commands_[i] = hw_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn S90ServoSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Deactivating ...please wait...");

    for (int i = 0; i < hw_stop_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "%.1f seconds left...",
        hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type S90ServoSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Reading...");

    for (uint i = 0; i < hw_states_.size(); i++)
    {
        // Simulate S90 servo's movement
        hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Got state %.5f for joint %d!",
        hw_states_[i], i);
    }
    RCLCPP_INFO(rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Joints successfully read!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type S90ServoSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Writing...");

    for (uint i = 0; i < hw_commands_.size(); i++)
    {
        // Simulate sending commands to the hardware
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",
        hw_commands_[i], i);
    }
    RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Joints successfully written!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
}

}  // namespace hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hardware::S90ServoSystemPositionOnlyHardware, hardware_interface::SystemInterface)
