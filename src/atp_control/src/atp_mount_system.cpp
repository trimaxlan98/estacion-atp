#include "atp_control/atp_mount_system.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace atp_control
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn AtpMountSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(get_logger(), "AtpMountSystem necesita exactamente 2 juntas (az, el); hay %zu",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }
  Vec2 initial{0.0, 0.0};
  u_max_ = 0.0;
  for (size_t i = 0; i < 2; ++i) {
    const auto & j = info_.joints[i];
    joint_[i] = j.name;
    bool has_effort = false, has_position = false;
    for (const auto & ci : j.command_interfaces) {
      has_effort |= ci.name == "effort";
      has_position |= ci.name == "position";
      if (ci.name == "effort" && !ci.max.empty()) {
        u_max_ = std::max(u_max_, std::fabs(std::stod(ci.max)));
      }
    }
    if (!has_effort || !has_position) {
      RCLCPP_ERROR(get_logger(), "la junta '%s' debe declarar command_interface effort y position",
        j.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & si : j.state_interfaces) {
      if (si.name == "position" && !si.initial_value.empty()) {
        initial[i] = std::stod(si.initial_value);
      }
    }
  }
  if (u_max_ <= 0.0) {
    u_max_ = 8.0;      // sin <param name="max"> en effort: el tope por defecto de mount_sim
  }
  const HwParams p(info_.hardware_parameters);
  servo_kp_ = p.num("servo_kp", 100.0);
  servo_kd_ = p.num("servo_kd", 20.0);
  try {
    backend_ = make_backend(p.str("backend", "sim"));
    if (!backend_->configure(p, initial, u_max_)) {
      RCLCPP_ERROR(get_logger(), "no se pudo configurar el backend '%s'",
        p.str("backend", "sim").c_str());
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  pos_ = initial;
  RCLCPP_INFO(get_logger(), "AtpMountSystem [%s, %s] u_max=%.2f N m", info_.name.c_str(),
    backend_->describe().c_str(), u_max_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn AtpMountSystem::on_activate(const rclcpp_lifecycle::State &)
{
  mode_ = {kNone, kNone};
  u_applied_ = {0.0, 0.0};
  if (!backend_->activate()) {
    RCLCPP_ERROR(get_logger(), "el backend no activo (sin respuesta del microcontrolador?)");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AtpMountSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  backend_->deactivate();
  return CallbackReturn::SUCCESS;
}

return_type AtpMountSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  // cada junta admite un solo modo a la vez
  std::array<int, 2> next = mode_;
  auto apply = [&](const std::vector<std::string> & list, bool on) {
      for (const auto & key : list) {
        for (size_t i = 0; i < 2; ++i) {
          if (key == joint_[i] + "/effort") {next[i] = on ? kEffort : kNone;}
          if (key == joint_[i] + "/position") {next[i] = on ? kPosition : kNone;}
        }
      }
    };
  apply(stop, false);
  // dos interfaces de la MISMA junta en start: conflicto
  for (size_t i = 0; i < 2; ++i) {
    const int n = std::count(start.begin(), start.end(), joint_[i] + "/effort") +
      std::count(start.begin(), start.end(), joint_[i] + "/position");
    if (n > 1) {
      RCLCPP_ERROR(get_logger(), "la junta %s no puede estar en effort y position a la vez",
        joint_[i].c_str());
      return return_type::ERROR;
    }
  }
  apply(start, true);
  return return_type::OK;
}

return_type AtpMountSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  for (const auto & key : stop) {
    for (size_t i = 0; i < 2; ++i) {
      if (key == joint_[i] + "/effort" || key == joint_[i] + "/position") {mode_[i] = kNone;}
    }
  }
  for (const auto & key : start) {
    for (size_t i = 0; i < 2; ++i) {
      if (key == joint_[i] + "/effort") {mode_[i] = kEffort;}
      if (key == joint_[i] + "/position") {mode_[i] = kPosition;}
    }
  }
  return return_type::OK;
}

return_type AtpMountSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::string why;
  if (!backend_->read(period.seconds(), pos_, vel_, why)) {
    RCLCPP_ERROR(get_logger(), "%s", why.c_str());
    return return_type::ERROR;
  }
  for (size_t i = 0; i < 2; ++i) {
    set_state(joint_[i] + "/position", pos_[i]);
    set_state(joint_[i] + "/velocity", vel_[i]);
    set_state(joint_[i] + "/effort", u_applied_[i]);
  }
  return return_type::OK;
}

return_type AtpMountSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  Vec2 u{0.0, 0.0};
  for (size_t i = 0; i < 2; ++i) {
    if (mode_[i] == kEffort) {
      const double c = get_command(joint_[i] + "/effort");
      u[i] = std::isfinite(c) ? c : 0.0;
    } else if (mode_[i] == kPosition) {
      const double c = get_command(joint_[i] + "/position");
      if (std::isfinite(c)) {
        u[i] = servo_kp_ * (c - pos_[i]) - servo_kd_ * vel_[i];
      }
    }
    u[i] = std::min(std::max(u[i], -u_max_), u_max_);
  }
  u_applied_ = u;
  if (!backend_->write(u, period.seconds())) {
    RCLCPP_ERROR(get_logger(), "fallo al escribir el mando");
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace atp_control

PLUGINLIB_EXPORT_CLASS(atp_control::AtpMountSystem, hardware_interface::SystemInterface)
