#include "atp_control/mount_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace atp_control
{

double HwParams::num(const std::string & k, double def) const
{
  const auto it = m_.find(k);
  return it == m_.end() || it->second.empty() ? def : std::stod(it->second);
}

int HwParams::integer(const std::string & k, int def) const
{
  const auto it = m_.find(k);
  return it == m_.end() || it->second.empty() ? def : static_cast<int>(std::stod(it->second));
}

std::string HwParams::str(const std::string & k, const std::string & def) const
{
  const auto it = m_.find(k);
  return it == m_.end() ? def : it->second;
}

PlantParams plant_params_from(const HwParams & p)
{
  constexpr double kDeg = M_PI / 180.0;
  PlantParams q;
  q.J = p.num("J", 2.0);
  q.b = p.num("b", 0.5);
  const double v = p.num("vel_max_deg_s", 0.0);
  if (v > 0.0) {
    q.w_max = v * kDeg;
  }
  q.h = p.num("substep_s", 0.002);
  q.coulomb_nm = p.num("coulomb_nm", 0.0);
  q.stribeck_nm = p.num("stribeck_nm", 0.0);
  q.v_stribeck = p.num("stribeck_vel_deg_s", 0.5) * kDeg;
  q.v_eps = p.num("friction_veps_deg_s", 0.0573) * kDeg;
  q.backlash_rad = p.num("backlash_deg", 0.0) * kDeg;
  q.frac_motor = p.num("backlash_motor_frac", 0.5);
  q.restitution = p.num("backlash_restitution", 0.0);
  q.mgd_nm = p.num("unbalance_nm", 0.0);
  return q;
}

namespace
{

// ---------------------------------------------------------------- sim

class SimBackend : public MountBackend
{
public:
  bool configure(const HwParams & p, const Vec2 & initial, double u_max) override
  {
    plant_ = std::make_unique<MountPlant>(plant_params_from(p));
    plant_->engage(initial);
    bits_ = p.integer("encoder_bits", 0);
    motor_side_ = p.str("encoder_side", "load") == "motor";
    u_max_ = u_max;
    return true;
  }
  bool activate() override
  {
    first_ = true;     // la primera lectura no integra (el enganche es en reposo)
    u_ = {0.0, 0.0};
    return true;
  }
  bool read(double dt, Vec2 & pos, Vec2 & vel, std::string &) override
  {
    if (!first_ && dt > 0.0) {
      // retencion de orden cero del mando escrito en el ciclo anterior
      plant_->advance(u_, {0.0, 0.0}, dt);
    }
    first_ = false;
    const Vec2 & th = plant_->encoder_angle(motor_side_);
    for (int i = 0; i < 2; ++i) {
      pos[i] = quantize_angle(th[i], bits_);
      vel[i] = plant_->omega()[i];
    }
    return true;
  }
  bool write(const Vec2 & u, double) override
  {
    for (int i = 0; i < 2; ++i) {
      u_[i] = std::min(std::max(u[i], -u_max_), u_max_);
    }
    return true;
  }
  std::string describe() const override
  {
    const auto & q = plant_->params();
    std::ostringstream s;
    s << "sim: J=" << q.J << " b=" << q.b << " subpaso " << q.h << " s"
      << (q.coulomb_nm > 0 || q.stribeck_nm > 0 ? " +friccion" : "")
      << (q.backlash_rad > 0 ? " +backlash" : "") << (q.mgd_nm != 0 ? " +desbalance" : "")
      << (bits_ > 0 ? " +encoder" : "");
    return s.str();
  }

private:
  std::unique_ptr<MountPlant> plant_;
  Vec2 u_{0.0, 0.0};
  int bits_ = 0;
  bool motor_side_ = false, first_ = true;
  double u_max_ = 8.0;
};

// ---------------------------------------------------------------- serial

class SerialBackend : public MountBackend
{
public:
  explicit SerialBackend(std::unique_ptr<SerialPort> port = nullptr) : port_(std::move(port)) {}
  bool configure(const HwParams & p, const Vec2 & initial, double u_max) override
  {
    cpr_ = p.num("serial_cpr", 65536.0);
    timeout_ms_ = p.num("serial_timeout_ms", 250.0);
    tau_vel_ = p.num("serial_vel_tau_s", 0.05);
    FakeMcuConfig f;
    f.plant = plant_params_from(p);
    f.initial = initial;
    f.cpr = cpr_;
    f.u_max = u_max;
    f.report_rate = p.integer("serial_fake_rate", 1) != 0;
    if (!port_) {
      port_ = make_port(p.str("serial_port", "fake"), p.integer("serial_baud", 921600), f);
    }
    return port_->open();
  }
  bool activate() override
  {
    // saludo: mando de par nulo y espera de la primera trama de estado
    seq_ = 0;
    have_ = false;
    send(0.0, 0.0, 0.05);
    const auto t0 = std::chrono::steady_clock::now();
    while (!have_) {
      poll();
      if (have_) {break;}
      if (std::chrono::steady_clock::now() - t0 > std::chrono::duration<double, std::milli>(
          std::max(timeout_ms_, 500.0)))
      {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    silent_s_ = 0.0;
    return true;
  }
  void deactivate() override
  {
    if (port_ && port_->is_open()) {
      send(0.0, 0.0, 0.05);     // par cero antes de soltar el bus
    }
  }
  bool read(double dt, Vec2 & pos, Vec2 & vel, std::string & why) override
  {
    const bool nuevo = poll();
    // vigilancia en tiempo del LAZO (el `dt` que da el controller_manager), no de
    // pared: asi vale igual en tiempo real y en la simulacion en lockstep
    if (nuevo) {
      silent_s_ = 0.0;
    } else {
      silent_s_ += dt;
      if (silent_s_ * 1e3 > timeout_ms_) {
        why = "sin tramas de estado del microcontrolador en " +
          std::to_string(static_cast<int>(silent_s_ * 1e3)) + " ms";
        return false;
      }
    }
    if (st_.flags & StateFrame::kFault) {
      why = "el microcontrolador reporta falla (paro de emergencia)";
      return false;
    }
    const double k = 2.0 * M_PI / cpr_;
    for (int i = 0; i < 2; ++i) {
      pos[i] = static_cast<double>(st_.cnt[i]) * k;
      if (st_.flags & StateFrame::kRateValid) {
        vel[i] = static_cast<double>(st_.rate[i]) * k;
      } else if (nuevo && have_prev_ && dt > 0.0) {
        // el MCU no mide velocidad: derivada de la posicion con filtro de 1.er orden
        const double a = dt / (tau_vel_ + dt);
        vel_f_[i] += a * ((pos[i] - pos_prev_[i]) / dt - vel_f_[i]);
        vel[i] = vel_f_[i];
      } else {
        vel[i] = vel_f_[i];
      }
      pos_prev_[i] = pos[i];
    }
    if (nuevo) {have_prev_ = true;}
    return true;
  }
  bool write(const Vec2 & u, double dt) override
  {
    return send(u[0], u[1], dt > 0.0 ? dt : 0.05);
  }
  std::string describe() const override
  {
    std::ostringstream s;
    s << "serial: puerto " << port_->name() << ", " << cpr_ << " cuentas/rev, timeout "
      << timeout_ms_ << " ms";
    return s.str();
  }

private:
  bool send(double a, double b, double dt)
  {
    CommandFrame c;
    c.seq = ++seq_;
    c.dt_us = static_cast<uint32_t>(std::lround(dt * 1e6));
    c.tau[0] = a;
    c.tau[1] = b;
    return port_->write(encode(c));
  }
  // vacia el puerto; true si llego al menos una trama de estado valida
  bool poll()
  {
    bool nuevo = false;
    char buf[256];
    for (size_t n; (n = port_->read(buf, sizeof(buf))) > 0;) {
      for (const auto & fr : parser_.feed(buf, n)) {
        if (const auto * s = std::get_if<StateFrame>(&fr)) {
          st_ = *s;       // se queda la ultima
          nuevo = have_ = true;
        }
      }
    }
    return nuevo;
  }

  std::unique_ptr<SerialPort> port_;
  FrameParser parser_;
  StateFrame st_;
  bool have_ = false, have_prev_ = false;
  double cpr_ = 65536.0, timeout_ms_ = 250.0, tau_vel_ = 0.05;
  uint32_t seq_ = 0;
  Vec2 pos_prev_{0, 0}, vel_f_{0, 0};
  double silent_s_ = 0.0;
};

}  // namespace

std::unique_ptr<MountBackend> make_serial_backend(std::unique_ptr<SerialPort> port)
{
  return std::make_unique<SerialBackend>(std::move(port));
}

std::unique_ptr<MountBackend> make_backend(const std::string & name)
{
  if (name == "sim") {
    return std::make_unique<SimBackend>();
  }
  if (name == "serial") {
    return std::make_unique<SerialBackend>();
  }
  throw std::invalid_argument("backend desconocido: " + name + " (sim | serial)");
}

}  // namespace atp_control
