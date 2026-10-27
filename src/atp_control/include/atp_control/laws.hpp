// Leyes de control de la casa en C++ (sin ROS): PID, LQR con accion integral y
// filtro de Kalman, portadas de src/atp_ros/atp_ros/{pid_controller,control}.py.
//
// Mismas formulas, mismo orden de operaciones y mismos valores por defecto que
// el codigo Python (docs/CONTROL.md): la ganancia K del LQR se calcula aqui
// mismo (Riccati discreta por duplicacion estructurada, SDA) y la prueba
// test_laws.cpp la contrasta con la que da control.lqr_integral.
//
// Un `TrackingLaw` se llama UNA vez por periodo del lazo (20 Hz) con la medida
// y la referencia del MISMO instante y devuelve el mando de los dos ejes
// (retencion de orden cero hasta la siguiente llamada).
#pragma once

#include <Eigen/Dense>

#include <array>
#include <memory>
#include <string>

#include "atp_control/mount_plant.hpp"

namespace atp_control
{

using Mat = Eigen::MatrixXd;

// ---------------------------------------------------------------- algebra

/// Exponencial de matriz por escalamiento y cuadrado con Taylor (control.py::expm).
Mat expm(const Mat & M, int terminos = 24);

/// Discretizacion exacta con retencion de orden cero: Ad = e^{A Ts}, Bd = int e^{As} ds B.
void discretize_zoh(const Mat & A, const Mat & B, double Ts, Mat & Ad, Mat & Bd);

/// Covarianza discreta de proceso (Van Loan 1978) para dx = A x dt + dw, densidad Qc.
void discretize_noise(const Mat & A, const Mat & Qc, double Ts, Mat & Ad, Mat & Qd);

/// Riccati discreta X = A'X(I + G X)^-1 A + Q por duplicacion estructurada (SDA).
Mat dare(const Mat & A, const Mat & B, const Mat & Q, const Mat & R, double tol = 1e-13,
  int max_iter = 100);

struct LqrDesign
{
  Eigen::RowVector3d K;           // [k_theta, k_omega, k_int]
  Eigen::Matrix3d Acl;            // lazo cerrado del sistema aumentado
  double max_pole_abs = 0.0;      // radio espectral (debe ser < 1)
  double Ts = 0.05;
};

/// LQR discreto con accion integral sobre [theta, omega, integral e] (control.py::lqr_integral).
LqrDesign lqr_integral(
  double J, double b, double Ts, double q_theta = 1.0e4, double q_omega = 1.0,
  double q_int = 1.0e3, double r = 1.0);

struct KalmanDesign
{
  Eigen::Matrix3d Ad;
  Eigen::Vector3d Bd;
  Eigen::Matrix3d Qd;
  double R = 0.0;
};

/// Modelo del Kalman [theta, omega, d] (control.py::diseno_kalman).
KalmanDesign kalman_design(
  double J, double b, double Ts, double sigma_med_rad, int bits, double q_acc = 1.0e-4,
  double q_par = 1.0e-2);

// ---------------------------------------------------------------- bloques

/// Estado estimado por eje: angulo, velocidad y par de sesgo.
struct Estimate
{
  Vec2 th{0, 0}, w{0, 0}, d{0, 0};
};

/// Velocidad y aceleracion de la referencia por diferencias hacia atras de 2.o orden.
class RefDifferentiator
{
public:
  explicit RefDifferentiator(double Ts) : Ts_(Ts) {}
  void step(const Vec2 & r, Vec2 & v, Vec2 & a);

private:
  double Ts_;
  Vec2 r1_{0, 0}, r2_{0, 0};
  int n_ = 0;
};

/// Estimador: medida + derivada filtrada (sin Kalman) o Kalman [theta, omega, d].
class Estimator
{
public:
  virtual ~Estimator() = default;
  /// Estimacion con la medida `y` y el mando previo (el realmente aplicado).
  virtual Estimate step(const Vec2 & y, const Vec2 & u_prev) = 0;
  virtual Estimate initial() const = 0;
};

std::unique_ptr<Estimator> make_estimator(
  bool kalman, double J, double b, double Ts, const Vec2 & y0, double sigma_med_rad, int bits,
  double q_acc, double q_par);

// ---------------------------------------------------------------- leyes

struct LawParams
{
  std::string law = "pid";          // pid | lqr
  std::string estimator = "none";   // none | kalman
  double rate_hz = 20.0;
  double u_max = 8.0;
  double J = 2.0, b = 0.5;
  // PID (pid_controller.py)
  double kp = 25.0, ki = 2.0, kd = 8.0, tau_derivada = 0.08;
  bool antiwindup = true;
  // LQR (control.LQR_DEFECTO)
  double q_theta = 1.0e4, q_omega = 1.0, q_int = 1.0e3, r_u = 1.0;
  bool feedforward = true, use_dhat = true;
  // lo que el filtro sabe del sensor
  double enc_sigma_deg = 0.0;
  int enc_bits = 0;
  double kf_q_acc = 1.0e-4, kf_q_par = 1.0e-2;
};

class TrackingLaw
{
public:
  virtual ~TrackingLaw() = default;
  /// Un periodo del lazo: medida y referencia (rad) del mismo instante -> mando (N m).
  virtual Vec2 compute(const Vec2 & meas, const Vec2 & ref) = 0;
  virtual std::string describe() const = 0;
};

/// Crea la ley (`law` = pid | lqr). Lanza std::invalid_argument si no existe.
std::unique_ptr<TrackingLaw> make_law(const LawParams & p);

}  // namespace atp_control
