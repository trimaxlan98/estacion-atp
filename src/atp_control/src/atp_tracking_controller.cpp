#include "atp_control/atp_tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "pluginlib/class_list_macros.hpp"

namespace atp_control
{

using controller_interface::CallbackReturn;
using controller_interface::return_type;

void RefHistory::push(int64_t s, const Vec2 & p)
{
  if (n > 0 && s <= stamp_ns[0]) {
    return;               // repetida o atrasada: se ignora
  }
  stamp_ns[2] = stamp_ns[1];
  pos[2] = pos[1];
  stamp_ns[1] = stamp_ns[0];
  pos[1] = pos[0];
  stamp_ns[0] = s;
  pos[0] = p;
  n = std::min(n + 1, 3);
}

Vec2 reference_at(const RefHistory & h, int64_t t_ns, bool extrapolate)
{
  if (h.n == 0) {
    return {0.0, 0.0};
  }
  const double dt = (t_ns - h.stamp_ns[0]) * 1e-9;
  // sin extrapolar si: se pidio asi, el sello coincide (lockstep) o la espera es absurda
  if (!extrapolate || h.n < 2 || dt <= 0.0 || dt > 0.25) {
    return h.pos[0];
  }
  const double hs = (h.stamp_ns[0] - h.stamp_ns[1]) * 1e-9;
  if (hs <= 0.0) {
    return h.pos[0];
  }
  // t medido desde la muestra mas nueva (<= 0 en las anteriores)
  const double t1 = -hs;
  const bool cuadratico = h.n >= 3 && h.stamp_ns[1] > h.stamp_ns[2] &&
    (h.stamp_ns[1] - h.stamp_ns[2]) * 1e-9 < 5.0 * hs;
  Vec2 r;
  for (int i = 0; i < 2; ++i) {
    if (cuadratico) {
      // polinomio de Lagrange por las 3 ultimas muestras con SUS sellos (no se supone
      // paso uniforme: con el reloj de pared los sellos de la referencia tiemblan
      // ~1 ms y eso, extrapolado a 50 ms, sesgaria la aceleracion que usa el LQR)
      const double t2 = -(h.stamp_ns[0] - h.stamp_ns[2]) * 1e-9;
      const double l0 = (dt - t1) * (dt - t2) / ((0.0 - t1) * (0.0 - t2));
      const double l1 = (dt - 0.0) * (dt - t2) / ((t1 - 0.0) * (t1 - t2));
      const double l2 = (dt - 0.0) * (dt - t1) / ((t2 - 0.0) * (t2 - t1));
      r[i] = l0 * h.pos[0][i] + l1 * h.pos[1][i] + l2 * h.pos[2][i];
    } else {
      r[i] = h.pos[0][i] + (h.pos[0][i] - h.pos[1][i]) / hs * dt;
    }
  }
  return r;
}

controller_interface::InterfaceConfiguration AtpTrackingController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    c.names.push_back(j + "/effort");
  }
  return c;
}

controller_interface::InterfaceConfiguration AtpTrackingController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    c.names.push_back(j + "/position");
  }
  return c;
}

CallbackReturn AtpTrackingController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {"az", "el"});
    auto_declare<std::string>("law", "pid");
    auto_declare<std::string>("estimator", "none");
    auto_declare<std::string>("reference_topic", "/atp/reference");
    auto_declare<std::string>("torque_topic", "/atp/torque");
    auto_declare<bool>("extrapolate_ref", true);
    auto_declare<bool>("sync_publish", false);
    auto_declare<std::string>("trace_file", "");
    auto_declare<double>("rate_hz", 20.0);
    auto_declare<double>("u_max", 8.0);
    auto_declare<double>("J", 2.0);
    auto_declare<double>("b", 0.5);
    auto_declare<double>("kp", 25.0);
    auto_declare<double>("ki", 2.0);
    auto_declare<double>("kd", 8.0);
    auto_declare<double>("tau_derivada", 0.08);
    auto_declare<bool>("antiwindup", true);
    auto_declare<double>("q_theta", 1.0e4);
    auto_declare<double>("q_omega", 1.0);
    auto_declare<double>("q_int", 1.0e3);
    auto_declare<double>("r_u", 1.0);
    auto_declare<bool>("feedforward", true);
    auto_declare<bool>("use_dhat", true);
    auto_declare<double>("enc_sigma_deg", 0.0);
    auto_declare<int>("enc_bits", 0);
    auto_declare<double>("kf_q_acc", 1.0e-4);
    auto_declare<double>("kf_q_par", 1.0e-2);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AtpTrackingController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  auto g = [&](const char * k) {return node->get_parameter(k);};
  joints_ = g("joints").as_string_array();
  if (joints_.size() != 2) {
    RCLCPP_ERROR(node->get_logger(), "se esperan 2 juntas (az, el), hay %zu", joints_.size());
    return CallbackReturn::ERROR;
  }
  params_ = LawParams();
  params_.law = g("law").as_string();
  params_.estimator = g("estimator").as_string();
  params_.rate_hz = g("rate_hz").as_double();
  params_.u_max = g("u_max").as_double();
  params_.J = g("J").as_double();
  params_.b = g("b").as_double();
  params_.kp = g("kp").as_double();
  params_.ki = g("ki").as_double();
  params_.kd = g("kd").as_double();
  params_.tau_derivada = g("tau_derivada").as_double();
  params_.antiwindup = g("antiwindup").as_bool();
  params_.q_theta = g("q_theta").as_double();
  params_.q_omega = g("q_omega").as_double();
  params_.q_int = g("q_int").as_double();
  params_.r_u = g("r_u").as_double();
  params_.feedforward = g("feedforward").as_bool();
  params_.use_dhat = g("use_dhat").as_bool();
  params_.enc_sigma_deg = g("enc_sigma_deg").as_double();
  params_.enc_bits = static_cast<int>(g("enc_bits").as_int());
  params_.kf_q_acc = g("kf_q_acc").as_double();
  params_.kf_q_par = g("kf_q_par").as_double();
  ref_topic_ = g("reference_topic").as_string();
  torque_topic_ = g("torque_topic").as_string();
  extrapolate_ = g("extrapolate_ref").as_bool();
  sync_publish_ = g("sync_publish").as_bool();
  trace_path_ = g("trace_file").as_string();
  if (params_.rate_hz <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "rate_hz debe ser > 0");
    return CallbackReturn::ERROR;
  }
  dt_ns_ = static_cast<int64_t>(std::llround(1e9 / params_.rate_hz));
  try {
    law_ = make_law(params_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }

  // cola profunda: la referencia no debe perderse (el monitor y el lazo la esperan)
  sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    ref_topic_, rclcpp::QoS(200),
    std::bind(&AtpTrackingController::on_reference, this, std::placeholders::_1));
  pub_raw_ = node->create_publisher<sensor_msgs::msg::JointState>(torque_topic_, rclcpp::QoS(10));
  pub_ = std::make_shared<TorquePub>(pub_raw_);
  RCLCPP_INFO(node->get_logger(), "%s | referencia %s -> mando %s",
    law_->describe().c_str(), ref_topic_.c_str(), torque_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn AtpTrackingController::on_activate(const rclcpp_lifecycle::State &)
{
  // el estado de la ley (integral, filtros) arranca limpio en cada activacion
  law_ = make_law(params_);
  t_last_ns_ = -1;
  u_ = {0.0, 0.0};
  for (auto & c : command_interfaces_) {
    std::ignore = c.set_value(0.0);
  }
  if (!trace_path_.empty()) {
    trace_ = std::fopen(trace_path_.c_str(), "w");
    if (trace_) {
      std::fprintf(trace_, "t_ns,ref_stamp_ns,ref_az,ref_el,meas_az,meas_el,u_az,u_el\n");
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AtpTrackingController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (trace_) {
    std::fclose(trace_);
    trace_ = nullptr;
  }
  for (auto & c : command_interfaces_) {
    std::ignore = c.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

bool AtpTrackingController::reference_connected() const
{
  return sub_ && sub_->get_publisher_count() >= 1;
}

void AtpTrackingController::on_reference(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // por nombre si vienen, si no por orden
  Vec2 p{0.0, 0.0};
  for (size_t i = 0; i < 2; ++i) {
    size_t k = i;
    for (size_t m = 0; m < msg->name.size(); ++m) {
      if (msg->name[m] == joints_[i]) {k = m;}
    }
    if (k >= msg->position.size()) {
      return;
    }
    p[i] = msg->position[k];
  }
  const int64_t s = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    msg->header.stamp.nanosec;
  RefHistory h = *ref_.readFromNonRT();
  h.push(s, p);
  ref_.writeFromNonRT(h);
  latest_stamp_ns_.store(h.stamp_ns[0]);
}

return_type AtpTrackingController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const RefHistory & h = *ref_.readFromRT();
  const int64_t now = time.nanoseconds();
  last_update_ns_.store(now);
  // un tick de la ley cada dt_ns de tiempo del controller_manager. La tolerancia es medio
  // periodo del controller_manager: si el ciclo se atrasa o adelanta unos ms (los
  // sellos de Gazebo y del reloj de pared tiemblan), el tick sigue siendo el N-esimo
  // ciclo y el paso entre ticks queda en dt_ns +- el temblor, no en dt_ns +- un ciclo.
  // En lockstep (periodo = dt_ns) es un tick por ciclo.
  const int64_t tol = std::min<int64_t>(dt_ns_ / 2, std::max<int64_t>(1000, period.nanoseconds() / 2));
  if (h.n > 0 && (t_last_ns_ < 0 || now - t_last_ns_ >= dt_ns_ - tol)) {
    // medida del instante (posicion de los encoders)
    Vec2 meas{0.0, 0.0};
    for (size_t i = 0; i < 2; ++i) {
      const auto v = state_interfaces_[i].get_optional();
      if (!v.has_value() || !std::isfinite(*v)) {
        return return_type::ERROR;
      }
      meas[i] = *v;
    }
    t_last_ns_ = now;
    const Vec2 ref = reference_at(h, now, extrapolate_);
    u_ = law_->compute(meas, ref);
    if (trace_) {   // diagnostico (E/S de archivo: no apto para tiempo real)
      std::fprintf(trace_, "%lld,%lld,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
        static_cast<long long>(now), static_cast<long long>(h.stamp_ns[0]), ref[0], ref[1],
        meas[0], meas[1], u_[0], u_[1]);
      std::fflush(trace_);
    }
    if (sync_publish_) {
      // lockstep: el mando debe salir ANTES de que el lazo confirme el paso (el
      // generador arranca el pase con el primer mando); el publicador de tiempo
      // real lo saca desde otro hilo, sin garantia de orden
      sensor_msgs::msg::JointState m;
      m.header.stamp = time;
      m.name = joints_;
      m.effort = {u_[0], u_[1]};
      pub_raw_->publish(m);
    } else if (pub_->trylock()) {
      auto & m = pub_->msg_;
      m.header.stamp = time;
      m.name = joints_;
      m.effort = {u_[0], u_[1]};
      pub_->unlockAndPublish();
    }
  }
  // retencion de orden cero entre ticks de la ley
  for (size_t i = 0; i < 2; ++i) {
    if (!command_interfaces_[i].set_value(u_[i])) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

}  // namespace atp_control

PLUGINLIB_EXPORT_CLASS(atp_control::AtpTrackingController, controller_interface::ControllerInterface)
