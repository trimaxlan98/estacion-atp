// AtpTrackingController: controlador de seguimiento de la Estacion ATP.
//
// Lee /atp/reference (sensor_msgs/JointState, rad) y cierra el lazo sobre las
// juntas az y el con la ley de la casa (PID o LQR integral, con derivada
// filtrada o Kalman; ver laws.hpp) a `rate_hz` (20 Hz). Escribe el par en la
// interfaz de comando `effort`; lee `position` (NO la velocidad real: como los
// controladores Python, la estima). Publica el mando en /atp/torque
// (JointState, effort, mismo sello) para el grabador y para el generador de
// referencia, que espera el primer mando para arrancar el pase.
//
// El controlador es independiente del hardware: funciona igual con
// AtpMountSystem (sim o serial) y con gz_ros2_control/GazeboSimSystem.
//
// Referencia y retardo: el controller_manager puede actualizarse mas rapido que
// la referencia (20 Hz) y la referencia del instante t puede llegar DESPUES
// de la actualizacion en t (en Gazebo, siempre). Con `extrapolate_ref` la
// referencia se lleva al instante de calculo con un polinomio de 2.o orden de
// sus ultimas muestras (error O(Ts^3)); si el sello coincide (lockstep) no
// se extrapola y el calculo es identico al del nodo Python.
#pragma once

#include <atomic>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "atp_control/laws.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace atp_control
{

/// Ultimas 3 muestras de la referencia (la mas nueva primero).
struct RefHistory
{
  int64_t stamp_ns[3] = {-1, -1, -1};
  Vec2 pos[3] = {{0, 0}, {0, 0}, {0, 0}};
  int n = 0;
  void push(int64_t s, const Vec2 & p);
};

/// Referencia en el instante `t_ns` a partir del historial (extrapolacion causal).
/// Si t_ns <= sello mas nuevo, o `extrapolate` es falso, devuelve la mas nueva.
Vec2 reference_at(const RefHistory & h, int64_t t_ns, bool extrapolate);

class AtpTrackingController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Sello (ns) de la ultima referencia recibida; -1 si no ha llegado ninguna.
  int64_t latest_reference_stamp_ns() const {return latest_stamp_ns_.load();}
  /// Sello (ns) del ultimo update() ejecutado (el controller_manager puede saltarse un
  /// ciclo si la lista de controladores esta tomada por otro hilo).
  int64_t last_update_ns() const {return last_update_ns_.load();}
  /// true si la suscripcion a la referencia tiene ya su publicador emparejado.
  bool reference_connected() const;
  std::string law_description() const {return law_ ? law_->describe() : "";}

private:
  void on_reference(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::vector<std::string> joints_;
  LawParams params_;
  std::string ref_topic_, torque_topic_;
  bool extrapolate_ = true;
  bool sync_publish_ = false;
  std::string trace_path_;
  std::FILE * trace_ = nullptr;
  std::unique_ptr<TrackingLaw> law_;
  int64_t dt_ns_ = 50000000;
  int64_t t_last_ns_ = -1;
  Vec2 u_{0.0, 0.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<RefHistory> ref_;
  std::atomic<int64_t> latest_stamp_ns_{-1};
  std::atomic<int64_t> last_update_ns_{-1};
  using TorquePub = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> pub_raw_;
  std::shared_ptr<TorquePub> pub_;
};

}  // namespace atp_control
