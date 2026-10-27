// El backend `sim` (MountPlant) debe reproducir la planta de planta.py.
// Referencia: test/data/planta_ref.csv, generada por test/gen_ref.py con
// atp_ros.planta.PlantaMontura y el mismo mando (retencion de orden cero a 20 Hz,
// subpaso de 2 ms, rafaga de 1 N m en los pasos 100-119).
//
// Tolerancia: 1e-9 rad / rad/s. Sin capas opcionales el codigo C++ repite las
// MISMAS operaciones en el mismo orden y el resultado es identico bit a bit; con
// friccion (exp de libm contra exp de numpy) y backlash (conmutaciones) la
// diferencia esperada es del orden del redondeo (1e-16 a 1e-13) y solo una
// conmutacion distinta (un subpaso) la haria crecer: la tolerancia detecta eso.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "atp_control/mount_plant.hpp"

using atp_control::MountPlant;
using atp_control::PlantParams;
using atp_control::Vec2;

namespace
{

constexpr double kDeg = M_PI / 180.0;

PlantParams caso(const std::string & n)
{
  PlantParams p;
  if (n == "tope_vel") {
    p.w_max = 6.0 * kDeg;
  } else if (n == "friccion" || n == "realista") {
    p.coulomb_nm = 0.05;
    p.stribeck_nm = 0.02;
    p.v_stribeck = 0.5 * kDeg;
    p.v_eps = 0.0573 * kDeg;
  }
  if (n == "backlash") {
    p.backlash_rad = 0.05 * kDeg;
  } else if (n == "backlash_rest") {
    p.backlash_rad = 0.2 * kDeg;
    p.restitution = 0.4;
  } else if (n == "realista") {
    p.backlash_rad = 0.05 * kDeg;
    p.mgd_nm = 0.05;
  }
  return p;
}

struct Row {int k; double u[2], th[2], w[2], thm[2];};

std::map<std::string, std::vector<Row>> load()
{
  std::ifstream f(std::string(TEST_DATA_DIR) + "/planta_ref.csv");
  EXPECT_TRUE(f.good()) << "falta planta_ref.csv";
  std::map<std::string, std::vector<Row>> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string name, tok;
    std::getline(ss, name, ',');
    Row r{};
    double v[9];
    for (int i = 0; i < 9; ++i) {
      std::getline(ss, tok, ',');
      v[i] = std::stod(tok);
    }
    r.k = static_cast<int>(v[0]);
    r.u[0] = v[1]; r.u[1] = v[2];
    r.th[0] = v[3]; r.th[1] = v[4];
    r.w[0] = v[5]; r.w[1] = v[6];
    r.thm[0] = v[7]; r.thm[1] = v[8];
    out[name].push_back(r);
  }
  return out;
}

}  // namespace

TEST(Planta, ReproducePlantaPython)
{
  const auto ref = load();
  ASSERT_EQ(ref.size(), 6u);
  for (const auto & [name, rows] : ref) {
    MountPlant p(caso(name));
    p.engage({20.0 * kDeg, 35.0 * kDeg});
    double max_th = 0.0, max_w = 0.0, max_thm = 0.0;
    for (const auto & r : rows) {
      const double ext = (r.k >= 100 && r.k < 120) ? 1.0 : 0.0;
      p.advance({r.u[0], r.u[1]}, {ext, ext}, 0.05);
      for (int i = 0; i < 2; ++i) {
        max_th = std::max(max_th, std::fabs(p.theta()[i] - r.th[i]));
        max_w = std::max(max_w, std::fabs(p.omega()[i] - r.w[i]));
        const double thm = caso(name).backlash_rad > 0 ? p.theta_m()[i] : p.theta()[i];
        max_thm = std::max(max_thm, std::fabs(thm - r.thm[i]));
      }
    }
    std::printf("[planta] %-14s pasos=%zu  max|dtheta|=%.3e rad  max|domega|=%.3e rad/s  "
      "max|dtheta_m|=%.3e\n", name.c_str(), rows.size(), max_th, max_w, max_thm);
    EXPECT_LT(max_th, 1e-9) << name;
    EXPECT_LT(max_w, 1e-9) << name;
    EXPECT_LT(max_thm, 1e-9) << name;
  }
}

TEST(Planta, IdealIdenticaBitABit)
{
  // Sin capas opcionales las operaciones son las mismas que en Python: igualdad exacta.
  const auto ref = load();
  MountPlant p(caso("ideal"));
  p.engage({20.0 * kDeg, 35.0 * kDeg});
  for (const auto & r : ref.at("ideal")) {
    const double ext = (r.k >= 100 && r.k < 120) ? 1.0 : 0.0;
    p.advance({r.u[0], r.u[1]}, {ext, ext}, 0.05);
    ASSERT_EQ(p.theta()[0], r.th[0]) << "k=" << r.k;
    ASSERT_EQ(p.theta()[1], r.th[1]) << "k=" << r.k;
    ASSERT_EQ(p.omega()[0], r.w[0]) << "k=" << r.k;
  }
}

TEST(Planta, FisicaBasica)
{
  // Mando constante: la velocidad tiende a u/b (el amortiguamiento domina).
  MountPlant p(PlantParams{});
  p.engage({0.0, 0.0});
  for (int k = 0; k < 1200; ++k) {p.advance({1.0, -0.5}, {0.0, 0.0}, 0.05);}
  EXPECT_NEAR(p.omega()[0], 1.0 / 0.5, 1e-6);
  EXPECT_NEAR(p.omega()[1], -0.5 / 0.5, 1e-6);
  // constante de tiempo J/b = 4 s: a t = 4 s, 63.2 % de la velocidad final
  MountPlant q(PlantParams{});
  q.engage({0.0, 0.0});
  for (int k = 0; k < 80; ++k) {q.advance({1.0, 0.0}, {0.0, 0.0}, 0.05);}
  EXPECT_NEAR(q.omega()[0] / 2.0, 1.0 - std::exp(-1.0), 5e-3);
}

TEST(Planta, FriccionAdhiereBajoLaEstatica)
{
  PlantParams c = caso("friccion");
  MountPlant p(c);
  p.engage({0.0, 0.0});
  for (int k = 0; k < 200; ++k) {p.advance({0.05, 0.05}, {0.0, 0.0}, 0.05);}   // < Fs = 0.07
  EXPECT_EQ(p.omega()[0], 0.0);
  EXPECT_EQ(p.theta()[0], 0.0);
  for (int k = 0; k < 200; ++k) {p.advance({0.2, 0.2}, {0.0, 0.0}, 0.05);}     // > Fs: arranca
  EXPECT_GT(p.omega()[0], 0.01);
}

TEST(Planta, CuantizacionDelEncoder)
{
  using atp_control::quantize_angle;
  const double q = 2.0 * M_PI / 16384.0;          // 14 bits: 0.02197 grados
  EXPECT_NEAR(q / kDeg, 0.02197, 1e-5);
  EXPECT_DOUBLE_EQ(quantize_angle(0.3 * q, 14), 0.0);
  EXPECT_DOUBLE_EQ(quantize_angle(0.7 * q, 14), q);
  EXPECT_DOUBLE_EQ(quantize_angle(-0.7 * q, 14), -q);
  EXPECT_DOUBLE_EQ(quantize_angle(0.5 * q, 14), 0.0);    // al par, como np.round
  EXPECT_DOUBLE_EQ(quantize_angle(1.234, 0), 1.234);
}
