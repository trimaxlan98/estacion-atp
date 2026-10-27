// lockstep_node: el controller_manager de ros2_control movido por el reloj /clock
// de la Estacion (sim_clock), para correr el MISMO plugin de hardware y el MISMO
// controlador del montaje real en la simulacion determinista del CONTRATO 8.
//
// `ros2_control_node` (el estandar) mueve el lazo con el reloj de pared y no puede
// esperar a nadie. Este ejecutable cambia SOLO eso: en cada /clock t_k hace
//   read(t_k) -> [espera la referencia con sello t_k] -> update -> write -> ack t_k
// y confirma el paso en /atp/step_ack para que sim_clock avance. Es el mismo
// protocolo de mount_sim (monturas, controlador y monitor no notan la diferencia).
// Con lockstep:=false no espera ni confirma: un paso por cada /clock (reloj libre,
// como sim_clock con time_scale 1) y el controlador extrapola la referencia.
//
// Servicios y spawners (load/configure/switch) funcionan igual que en
// ros2_control_node: el ejecutor gira en su propio hilo y el lazo en el principal.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "atp_control/atp_tracking_controller.hpp"
#include "controller_manager/controller_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

class Runner
{
public:
  Runner(
    std::shared_ptr<controller_manager::ControllerManager> cm,
    std::shared_ptr<rclcpp::Node> node)
  : cm_(std::move(cm)), node_(std::move(node))
  {
    lockstep_ = node_->declare_parameter<bool>("lockstep", true);
    // nombres separados por comas (launch convierte una lista en tupla YAML no valida)
    const std::string lista = node_->declare_parameter<std::string>("expected_controllers", "");
    for (size_t i = 0, j; i < lista.size(); i = j + 1) {
      j = lista.find(',', i);
      if (j == std::string::npos) {j = lista.size();}
      if (j > i) {expected_.push_back(lista.substr(i, j - i));}
    }
    ref_timeout_s_ = node_->declare_parameter<double>("ref_timeout_s", 10.0);
    // paso minimo de PARED: los nodos pasivos (monitor, grabador) no frenan el reloj
    // y si el lazo corre mucho mas rapido que ellos, la cola de escritura de DDS
    // (profundidad 10) puede pisar muestras que aun no recibieron
    min_step_wall_s_ = node_->declare_parameter<double>("min_step_wall_s", 0.0);
    clock_sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(10), [this](rosgraph_msgs::msg::Clock::SharedPtr m) {
        const int64_t t = static_cast<int64_t>(m->clock.sec) * 1000000000LL + m->clock.nanosec;
        std::lock_guard<std::mutex> lk(mtx_);
        clock_ns_ = t;
        cv_.notify_all();
      });
    // barrera de /joint_states: el joint_state_broadcaster publica desde un hilo
    // propio y DESCARTA una muestra si el hilo sigue ocupado con la anterior; a
    // maxima velocidad eso pasaba. Antes de confirmar el paso se espera a ver
    // llegar el estado de ese instante, que ademas asegura que el monitor lo recibe
    // antes de que el reloj avance.
    js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(200), [this](sensor_msgs::msg::JointState::SharedPtr m) {
        const int64_t t = static_cast<int64_t>(m->header.stamp.sec) * 1000000000LL +
        m->header.stamp.nanosec;
        int64_t prev = js_ns_.load();
        while (t > prev && !js_ns_.compare_exchange_weak(prev, t)) {}
      });
    ack_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/atp/step_ack", rclcpp::QoS(10));
    rclcpp::QoS listo(50);
    listo.transient_local();
    listo_pub_ = node_->create_publisher<std_msgs::msg::String>("/atp/listo", listo);
  }

  void run()
  {
    startup();
    RCLCPP_INFO(node_->get_logger(), "controladores activos: %s | %s",
      lockstep_ ? "lockstep" : "reloj libre", tracking_ ? tracking_->law_description().c_str() : "");
    int64_t t_prev = -1;
    uint64_t pasos = 0, reintentos = 0, republicados = 0;
    while (rclcpp::ok()) {
      int64_t t;
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait_for(lk, 50ms, [&] {return clock_ns_ > t_prev && clock_ns_ > 0;});
        t = clock_ns_;
      }
      // /clock repetido (latido de sim_clock), viejo o aun en t = 0: nada que hacer
      if (t <= t_prev || t <= 0) {continue;}
      const auto inicio_paso = std::chrono::steady_clock::now();
      const rclcpp::Time now(t, RCL_ROS_TIME);
      const rclcpp::Duration period =
        rclcpp::Duration::from_nanoseconds(t_prev < 0 ? 0 : t - t_prev);
      t_prev = t;
      cm_->read(now, period);
      if (lockstep_ && !wait_reference(t)) {
        RCLCPP_WARN(node_->get_logger(), "sin referencia con sello t=%.3f tras %.0f s: sigo sin ella",
          t * 1e-9, ref_timeout_s_);
      }
      // update() se salta el ciclo sin avisar si otro hilo (diagnosticos, servicios)
      // tiene tomada la lista de controladores; en tiempo real se pierde un ciclo y
      // ya, pero en lockstep seria un paso sin mando ni estado: se reintenta (un
      // ciclo saltado no ejecuta nada, repetirlo es seguro)
      for (int intento = 0;; ++intento) {
        cm_->update(now, period);
        if (!tracking_ || tracking_->last_update_ns() == t || intento >= 2000) {break;}
        if (++reintentos == 1) {
          RCLCPP_WARN(node_->get_logger(), "t=%.3f: el controller_manager salto el ciclo (lista de controladores tomada); se reintenta", t * 1e-9);
        }
        std::this_thread::sleep_for(500us);
      }
      cm_->write(now, period);
      // si el broadcaster descarto la muestra (su hilo de publicacion seguia
      // ocupado) se repite update(): es idempotente para el controlador (la ley
      // solo corre una vez por periodo) y el broadcaster publica el mismo estado
      for (int intento = 0; lockstep_ && !wait_joint_states(t, 20ms); ++intento) {
        if (intento >= 50) {
          RCLCPP_WARN(node_->get_logger(),
            "el estado de t=%.3f no llego a /joint_states (el broadcaster lo descarto)", t * 1e-9);
          break;
        }
        ++republicados;
        cm_->update(now, period);
      }
      if (lockstep_) {
        rosgraph_msgs::msg::Clock ack;
        ack.clock.sec = static_cast<int32_t>(t / 1000000000LL);
        ack.clock.nanosec = static_cast<uint32_t>(t % 1000000000LL);
        ack_pub_->publish(ack);
      }
      ++pasos;
      if (min_step_wall_s_ > 0.0) {
        std::this_thread::sleep_until(
          inicio_paso + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(min_step_wall_s_)));
      }
    }
    RCLCPP_INFO(node_->get_logger(), "fin: %lu pasos, %lu reintentos de update (ciclos saltados por el controller_manager), "
      "%lu republicaciones de /joint_states",
      static_cast<unsigned long>(pasos), static_cast<unsigned long>(reintentos),
      static_cast<unsigned long>(republicados));
  }

private:
  // Hasta que los controladores estan activos hay que girar update(): activar un
  // controlador (switch_controller) espera a que el lazo de tiempo real procese
  // el cambio. Sin tiempo simulado que avance, se llama con periodo 0.
  // OJO: get_loaded_controllers() toma el cerrojo de la lista de controladores y
  // load_controller espera a que el hilo "de tiempo real" avance: llamarlo desde
  // ESTE hilo bloquea ambos (interbloqueo). Lo consulta un hilo aparte.
  void startup()
  {
    std::atomic<bool> activos{false};
    std::thread vigia([&]() {
        while (rclcpp::ok() && !activos.load()) {
          if (all_active()) {activos.store(true);}
          std::this_thread::sleep_for(5ms);
        }
      });
    int ciclos = 0;
    while (rclcpp::ok()) {
      int64_t t;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        t = clock_ns_;
      }
      // (update rechaza un tiempo 0 sin reloj recibido: 1 ns basta como "aun no hay reloj")
      cm_->update(rclcpp::Time(std::max<int64_t>(t, 1), RCL_ROS_TIME), rclcpp::Duration(0, 0));
      if (activos.load() && (!tracking_ || tracking_->reference_connected() || !lockstep_)) {
        // saludo de arranque (CONTRATO 8.4): el generador espera este nombre
        // para publicar la primera referencia
        std_msgs::msg::String s;
        s.data = node_->get_name();
        listo_pub_->publish(s);
        break;
      }
      if (++ciclos % 5000 == 0) {
        RCLCPP_INFO(node_->get_logger(), "arranque: esperando controladores activos (%d ciclos)", ciclos);
      }
      std::this_thread::sleep_for(2ms);
    }
    activos.store(true);
    vigia.join();
  }

  bool all_active()
  {
    const auto specs = cm_->get_loaded_controllers();
    if (specs.size() < std::max<size_t>(expected_.size(), 1)) {return false;}
    for (const auto & n : expected_) {
      bool hay = false;
      for (const auto & s : specs) {hay |= s.info.name == n;}
      if (!hay) {return false;}
    }
    std::shared_ptr<atp_control::AtpTrackingController> trk;
    for (const auto & s : specs) {
      if (s.c->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return false;
      }
      if (!trk) {
        trk = std::dynamic_pointer_cast<atp_control::AtpTrackingController>(s.c);
      }
    }
    tracking_ = trk;
    return true;
  }

  // espera (de pared) a que llegue la referencia del instante t
  bool wait_reference(int64_t t)
  {
    if (!tracking_) {return true;}
    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (tracking_->latest_reference_stamp_ns() >= t) {return true;}
      if (std::chrono::steady_clock::now() - t0 > std::chrono::duration<double>(ref_timeout_s_)) {
        return false;
      }
      std::this_thread::sleep_for(50us);
    }
    return false;
  }

  // espera (de pared) a ver pasar por /joint_states el estado con sello >= t
  bool wait_joint_states(int64_t t, std::chrono::nanoseconds max_espera)
  {
    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (js_ns_.load() >= t) {
        // el mensaje ya llego, pero el hilo del publicador de tiempo real del
        // broadcaster aun puede estar soltando su cerrojo: si el siguiente
        // update() lo encontrara tomado, DESCARTARIA esa muestra
        std::this_thread::sleep_for(300us);
        return true;
      }
      if (std::chrono::steady_clock::now() - t0 > max_espera) {return false;}
      std::this_thread::sleep_for(20us);
    }
    return false;
  }

  std::shared_ptr<controller_manager::ControllerManager> cm_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<atp_control::AtpTrackingController> tracking_;
  bool lockstep_ = true;
  std::vector<std::string> expected_;
  double ref_timeout_s_ = 10.0;
  double min_step_wall_s_ = 0.0;
  std::mutex mtx_;
  std::condition_variable cv_;
  int64_t clock_ns_ = -1;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::atomic<int64_t> js_ns_{-1};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr listo_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 4);
  // como ros2_control_node: el controller_manager no usa los argumentos globales,
  // hay que pasarle los de la linea de comandos (archivos de parametros, -p ...)
  rclcpp::NodeOptions cm_options = controller_manager::get_cm_node_options();
  std::vector<std::string> node_arguments = cm_options.arguments();
  for (int i = 1; i < argc; ++i) {
    node_arguments.emplace_back(argv[i]);
  }
  cm_options.arguments(node_arguments);
  auto node = std::make_shared<rclcpp::Node>("lockstep_runner");
  // el URDF llega como parametro `robot_description` (como en ros2_control_node)
  const std::string urdf = node->declare_parameter<std::string>("robot_description", "");
  if (urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "falta el parametro robot_description");
    return 1;
  }
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, urdf, true, "controller_manager", "", cm_options);
  executor->add_node(cm);
  executor->add_node(node);
  std::thread spin([executor]() {executor->spin();});
  {
    Runner runner(cm, node);
    runner.run();
  }
  executor->cancel();
  spin.join();
  rclcpp::shutdown();
  return 0;
}
