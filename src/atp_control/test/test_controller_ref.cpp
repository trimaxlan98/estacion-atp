// Historial y extrapolacion de la referencia del AtpTrackingController.
#include <gtest/gtest.h>

#include <cmath>

#include "atp_control/atp_tracking_controller.hpp"

using namespace atp_control;   // NOLINT

namespace
{
constexpr int64_t kTs = 50000000;   // 50 ms
Vec2 f(double t) {return {0.3 + 0.2 * t + 0.1 * t * t, 1.0 - 0.05 * t * t};}
}  // namespace

TEST(RefHistory, OrdenaIgnoraRepetidasYAtrasadas)
{
  RefHistory h;
  h.push(100, {1.0, 1.0});
  h.push(200, {2.0, 2.0});
  h.push(200, {9.0, 9.0});      // repetida
  h.push(150, {8.0, 8.0});      // atrasada
  h.push(300, {3.0, 3.0});
  EXPECT_EQ(h.n, 3);
  EXPECT_EQ(h.stamp_ns[0], 300);
  EXPECT_EQ(h.stamp_ns[1], 200);
  EXPECT_EQ(h.stamp_ns[2], 100);
  EXPECT_EQ(h.pos[1][0], 2.0);
}

TEST(RefHistory, SinExtrapolarSiElSelloCoincide)
{
  RefHistory h;
  for (int k = 1; k <= 3; ++k) {h.push(k * kTs, f(k * 0.05));}
  const Vec2 r = reference_at(h, 3 * kTs, true);     // lockstep: mismo sello
  EXPECT_EQ(r[0], f(3 * 0.05)[0]);
  EXPECT_EQ(r[1], f(3 * 0.05)[1]);
}

TEST(RefHistory, ExtrapolaExactoUnPolinomioDeSegundoOrden)
{
  RefHistory h;
  for (int k = 1; k <= 3; ++k) {h.push(k * kTs, f(k * 0.05));}
  for (double dt : {0.01, 0.02, 0.05}) {
    const Vec2 r = reference_at(h, 3 * kTs + static_cast<int64_t>(dt * 1e9), true);
    EXPECT_NEAR(r[0], f(0.15 + dt)[0], 1e-12) << dt;
    EXPECT_NEAR(r[1], f(0.15 + dt)[1], 1e-12) << dt;
  }
  // sin extrapolacion queda la ultima muestra (el error v*dt del CONTRATO 8)
  const Vec2 s = reference_at(h, 3 * kTs + 20000000, false);
  EXPECT_EQ(s[0], f(3 * 0.05)[0]);
}

TEST(RefHistory, ExtrapolaExactoConSellosIrregulares)
{
  // con el reloj de pared los sellos de la referencia tiemblan: Lagrange con los sellos
  // reales sigue siendo exacto para un polinomio de 2.o orden (el paso uniforme no lo seria)
  RefHistory h;
  const int64_t sellos[3] = {1 * kTs + 700000, 2 * kTs - 1300000, 3 * kTs + 900000};
  for (int k = 0; k < 3; ++k) {h.push(sellos[k], f(sellos[k] * 1e-9));}
  const int64_t ahora = sellos[2] + 31000000;
  const Vec2 r = reference_at(h, ahora, true);
  EXPECT_NEAR(r[0], f(ahora * 1e-9)[0], 1e-12);
  EXPECT_NEAR(r[1], f(ahora * 1e-9)[1], 1e-12);
}

TEST(RefHistory, LinealConDosMuestrasYSeguroConHuecosGrandes)
{
  RefHistory h;
  h.push(1 * kTs, {1.0, 0.0});
  h.push(2 * kTs, {1.1, 0.0});
  const Vec2 r = reference_at(h, 2 * kTs + 25000000, true);
  EXPECT_NEAR(r[0], 1.15, 1e-12);
  // una espera absurda (> 250 ms) no extrapola: se queda con la ultima
  const Vec2 g = reference_at(h, 2 * kTs + 2000000000, true);
  EXPECT_EQ(g[0], 1.1);
  // sin muestras: cero
  EXPECT_EQ(reference_at(RefHistory{}, 100, true)[0], 0.0);
}
