// Fisica de la montura (C++ puro, sin ROS): J theta'' + b theta' = u + perturbaciones.
//
// Port de src/atp_ros/atp_ros/planta.py (clase PlantaMontura), operacion por
// operacion, para que el backend `sim` del plugin AtpMountSystem reproduzca la
// planta del nodo Python mount_sim. Dos ejes (az, el). Capas opcionales, todas
// APAGADAS por defecto (con los valores por defecto el paso es el semi-implicito
// de siempre: alpha = (u + par_ext - b w)/J):
//   * friccion de Coulomb + Stribeck con adherencia tipo Karnopp
//   * backlash del reductor: dos cuerpos (motor y carga) con holgura entre ellos
//   * desbalance gravitatorio en elevacion: par -m g d cos(el)
//   * cuantizacion del encoder
// Fuentes y condiciones de validez: docs/CONTROL.md, seccion 1.
#pragma once

#include <array>
#include <cmath>
#include <limits>

namespace atp_control
{

using Vec2 = std::array<double, 2>;

struct PlantParams
{
  double J = 2.0;                 // kg m^2, inercia reflejada al eje de carga
  double b = 0.5;                 // N m s/rad
  double w_max = std::numeric_limits<double>::infinity();  // rad/s
  double h = 0.002;               // subpaso de integracion, s
  double coulomb_nm = 0.0;
  double stribeck_nm = 0.0;       // exceso estatico Fs - Fc
  double v_stribeck = 0.01;       // rad/s
  double v_eps = 1e-3;            // banda de adherencia, rad/s
  double backlash_rad = 0.0;      // holgura total
  double frac_motor = 0.5;        // fraccion de J (y b) en el motor
  double restitution = 0.0;
  double mgd_nm = 0.0;            // m g d de la elevacion
};

class MountPlant
{
public:
  explicit MountPlant(const PlantParams & p);

  /// Pone la carga en `theta0` (rad) y en reposo (como el enganche de mount_sim).
  void engage(const Vec2 & theta0);

  /// Avanza `h` s con el mando `u` (N m, en el motor) y el par externo `ext`
  /// (en la carga).
  void step(const Vec2 & u, const Vec2 & ext, double h);

  /// Integra `dt` s en subpasos de p.h (el ultimo, el resto), como on_clock.
  void advance(const Vec2 & u, const Vec2 & ext, double dt);

  /// Angulo que mide el encoder: en la carga o en el motor (detras del reductor).
  const Vec2 & encoder_angle(bool motor_side = false) const;

  bool engaged() const {return engaged_;}
  const Vec2 & theta() const {return theta_;}     // carga, rad
  const Vec2 & omega() const {return omega_;}     // carga, rad/s
  const Vec2 & theta_m() const {return theta_m_;}
  const Vec2 & omega_m() const {return omega_m_;}
  const PlantParams & params() const {return p_;}

private:
  void step_backlash(const Vec2 & u, const Vec2 & ext, double h);

  PlantParams p_;
  bool advanced_;
  bool engaged_ = false;
  Vec2 theta_{0, 0}, omega_{0, 0}, theta_m_{0, 0}, omega_m_{0, 0}, lado_{0, 0};
};

/// Cuantiza un angulo (rad) a un encoder de `bits` bits sobre 360 grados
/// (mid-tread, redondeo al par: igual que np.round). bits <= 0: sin cuantizar.
double quantize_angle(double theta_rad, int bits);

/// Curva de Stribeck F(w) = [Fc + Fe exp(-(w/vs)^2)] sgn.
double stribeck_friction(double w, double coulomb, double exceso, double vs, double signo);

}  // namespace atp_control
