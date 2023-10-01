#include "../include/micro_servo_hardware/S90_servo.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// #define GPIO_PIN 5

namespace micro_servo_hardware
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
    
    // Setting up RPi's pin definition
    wiringPiSetupGpio();

    // Parameters declaration 
    hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec_param"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec_param"]);
    hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown_param"]);

    for (uint i=0; i < info_.joints.size(); i++)
    {
        int num = i+1;
        GPIO_PINs.push_back(stoi(info_.hardware_parameters["pin"+ std::to_string(num)]));
        pins_mode.push_back((bool)stoi(info_.hardware_parameters["pin"+ std::to_string(num)+"_mode"]));
        servos.emplace_back(std::make_unique<microServo>(GPIO_PINs[i], pins_mode[i]));
    }
    // GPIO_PIN = stoi(info_.hardware_parameters["pin"]);
    // pin_mode = (bool)stoi(info_.hardware_parameters["pin_mode"]);
    
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

     //1:= OUTPUT, 0:= INPUT

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
    RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Configuring ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "%.1f seconds left...",
        hw_start_sec_ - i);
    }
    
    // reset values always when configuring hardware since no encoder is present
    for (uint i = 0; i < hw_states_.size(); i++)
    {
        hw_states_[i] = 1.5707; //0; -> set to 90°
        hw_commands_[i] = 1.5707; // 0; -> set to 90°
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
    RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
        rclcpp::get_logger("S90ServoSystemPositionOnlyHardware"), "%.1f seconds left...",
        hw_start_sec_ - i);
    }

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

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type S90ServoSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Read the command and keep it known 
    for (uint i = 0; i < hw_states_.size(); i++)
    {
        // Simulate S90 servo's movement
        hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;

    }

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type S90ServoSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    /*Transforming rad to pwm using the class micro_servo*/
    for (uint i = 0; i < hw_commands_.size(); i++)
    {            
            float angleRad = hw_commands_[i];
            servos[i]->goToAngle(angleRad);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace micro_servo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  micro_servo_hardware::S90ServoSystemPositionOnlyHardware, hardware_interface::SystemInterface)
