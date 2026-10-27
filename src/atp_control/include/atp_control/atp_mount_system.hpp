// AtpMountSystem: hardware_interface::SystemInterface de la montura Az/El.
//
// Dos juntas (az, el). Interfaces (las declara el URDF, ver atp_description):
//   comando : effort (N m), position (rad)
//   estado  : position (rad), velocity (rad/s), effort (N m, el mando aplicado)
// Con `effort` reclamado por un controlador, el mando va directo a la montura
// (el lazo lo cierra AtpTrackingController); con `position` reclamado, un servo
// PD interno (servo_kp, servo_kd) convierte la consigna en par: asi sirve
// tambien joint_trajectory_controller. Sin controlador activo, par cero.
//
// Backend (parametro `backend` de <hardware>): `sim` o `serial`; ver
// mount_backend.hpp. Es el mismo plugin y el mismo controlador para ambos.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "atp_control/mount_backend.hpp"
#include "hardware_interface/system_interface.hpp"

namespace atp_control
{

class AtpMountSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum Mode {kNone = 0, kEffort = 1, kPosition = 2};

  std::unique_ptr<MountBackend> backend_;
  std::array<std::string, 2> joint_;
  std::array<int, 2> mode_{kNone, kNone};
  Vec2 pos_{0, 0}, vel_{0, 0}, u_applied_{0, 0};
  double u_max_ = 8.0, servo_kp_ = 100.0, servo_kd_ = 20.0;
};

}  // namespace atp_control
