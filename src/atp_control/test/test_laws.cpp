// Las leyes de control en C++ contra las de Python (control.py, pid_controller.py).
//  * ganancia K del LQR integral y diseno del Kalman: contra gains_ref.txt
//  * lazo cerrado planta + ley en 400 pasos: contra laws_ref.csv
// Referencias generadas por test/gen_ref.py.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "atp_control/laws.hpp"
#include "atp_control/mount_plant.hpp"

using namespace atp_control;   // NOLINT

namespace
{

constexpr double kDeg = M_PI / 180.0;
constexpr double kDt = 0.05;

std::map<std::string, std::vector<double>> gains()
{
  std::ifstream f(std::string(TEST_DATA_DIR) + "/gains_ref.txt");
  EXPECT_TRUE(f.good());
  std::map<std::string, std::vector<double>> m;
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string k;
    ss >> k;
    double v;
    while (ss >> v) {m[k].push_back(v);}
  }
  return m;
}

Vec2 referencia(int k)
{
  const double t = k * kDt;
  return {(40.0 + 9.0 * t + 6.0 * std::sin(0.4 * t)) * kDeg,
    (30.0 + 25.0 * std::sin(0.12 * t)) * kDeg};
}

struct Row {int k; double u[2], th[2];};

std::map<std::string, std::vector<Row>> load_laws()
{
  std::ifstream f(std::string(TEST_DATA_DIR) + "/laws_ref.csv");
  EXPECT_TRUE(f.good());
  std::map<std::string, std::vector<Row>> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string name, tok;
    std::getline(ss, name, ',');
    double v[5];
    for (double & x : v) {
      std::getline(ss, tok, ',');
      x = std::stod(tok);
    }
    out[name].push_back({static_cast<int>(v[0]), {v[1], v[2]}, {v[3], v[4]}});
  }
  return out;
}

}  // namespace

TEST(Leyes, GananciaLqrIgualQuePython)
{
  const auto g = gains();
  const auto d = lqr_integral(2.0, 0.5, kDt);
  std::printf("[lqr] K C++ = [%.9f, %.9f, %.9f]  K Python = [%.9f, %.9f, %.9f]  |z|max %.6f\n",
    d.K(0), d.K(1), d.K(2), g.at("K")[0], g.at("K")[1], g.at("K")[2], d.max_pole_abs);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(d.K(i), g.at("K")[i], 1e-9) << "K[" << i << "]";
  }
  EXPECT_NEAR(d.max_pole_abs, g.at("polo_max")[0], 1e-9);
  EXPECT_LT(d.max_pole_abs, 1.0);
  // cifras de CONTROL.md: K = [84.0, 17.8, 24.6], |z| max 0.984
  EXPECT_NEAR(d.K(0), 84.0, 0.05);
  EXPECT_NEAR(d.K(1), 17.8, 0.05);
  EXPECT_NEAR(d.K(2), 24.6, 0.05);
  EXPECT_NEAR(d.max_pole_abs, 0.984, 5e-4);
}

TEST(Leyes, RiccatiContraFormaCerradaEscalar)
{
  // x+ = a x + u, Q = q, R = r: X = q + a^2 X - a^2 X^2/(r + X)  ->  se contrasta el residuo
  Mat A(1, 1), B(1, 1), Q(1, 1), R(1, 1);
  A << 0.9; B << 1.0; Q << 2.0; R << 0.5;
  const Mat X = dare(A, B, Q, R);
  const double x = X(0, 0);
  const double resid = x - (2.0 + 0.81 * x - 0.81 * x * x / (0.5 + x));
  EXPECT_NEAR(resid, 0.0, 1e-10);
}

TEST(Leyes, DisenoKalmanIgualQuePython)
{
  const auto g = gains();
  const auto k = kalman_design(2.0, 0.5, kDt, 0.0, 14);
  EXPECT_NEAR(k.R, g.at("kalman_R")[0], 1e-20);
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(k.Ad(i / 3, i % 3), g.at("kalman_Ad")[i], 1e-12);
    EXPECT_NEAR(k.Qd(i / 3, i % 3), g.at("kalman_Qd")[i], 1e-14);
  }
  // ganancia estacionaria del filtro tras 400 pasos con medida cero
  auto est = make_estimator(true, 2.0, 0.5, kDt, {0.0, 0.0}, 0.0, 14, 1e-4, 1e-2);
  for (int i = 0; i < 400; ++i) {est->step({0.0, 0.0}, {0.0, 0.0});}
  // (la ganancia no se expone: se verifica a traves del seguimiento en el lazo cerrado)
  SUCCEED();
}

class LazoCerrado : public ::testing::TestWithParam<const char *> {};

TEST_P(LazoCerrado, IgualQuePython)
{
  const std::string nombre = GetParam();
  const auto ref = load_laws().at(nombre);
  LawParams lp;
  lp.law = nombre.rfind("lqr", 0) == 0 ? "lqr" : "pid";
  const bool kal = nombre.size() > 3 && nombre.substr(nombre.size() - 3) == "_kf";
  lp.estimator = kal ? "kalman" : "none";
  lp.enc_bits = kal ? 14 : 0;
  auto ley = make_law(lp);

  PlantParams pp;
  if (kal) {
    pp.coulomb_nm = 0.05; pp.stribeck_nm = 0.02;
    pp.v_stribeck = 0.5 * kDeg; pp.v_eps = 0.0573 * kDeg;
    pp.backlash_rad = 0.05 * kDeg;
  }
  MountPlant p(pp);
  p.engage(referencia(1));
  Vec2 u{0, 0};
  double max_u = 0.0, max_th = 0.0;
  for (const auto & r : ref) {
    if (r.k > 1) {p.advance(u, {0.0, 0.0}, kDt);}
    Vec2 meas = p.encoder_angle();
    for (double & m : meas) {m = quantize_angle(m, lp.enc_bits);}
    u = ley->compute(meas, referencia(r.k));
    for (int i = 0; i < 2; ++i) {
      max_u = std::max(max_u, std::fabs(u[i] - r.u[i]));
      max_th = std::max(max_th, std::fabs(p.theta()[i] - r.th[i]));
    }
  }
  std::printf("[ley %-7s] pasos=%zu  max|du|=%.3e N m  max|dtheta|=%.3e rad   (%s)\n",
    nombre.c_str(), ref.size(), max_u, max_th, ley->describe().c_str());
  EXPECT_LT(max_u, 1e-7) << nombre;
  EXPECT_LT(max_th, 1e-9) << nombre;
}

INSTANTIATE_TEST_SUITE_P(Leyes, LazoCerrado, ::testing::Values("pid", "pid_kf", "lqr", "lqr_kf"));

TEST(Leyes, LeyDesconocidaLanza)
{
  LawParams lp;
  lp.law = "mpc";
  EXPECT_THROW(make_law(lp), std::invalid_argument);
  lp.law = "pid";
  lp.estimator = "ukf";
  EXPECT_THROW(make_law(lp), std::invalid_argument);
}

TEST(Leyes, SaturaEnUMax)
{
  LawParams lp;
  lp.law = "pid";
  auto ley = make_law(lp);
  const Vec2 u = ley->compute({0.0, 0.0}, {1.0, -1.0});   // 57 grados de error
  EXPECT_DOUBLE_EQ(u[0], 8.0);
  EXPECT_DOUBLE_EQ(u[1], -8.0);
}
