// Backend `serial`: protocolo (codec, XOR, resincronizacion), MCU falso y la
// equivalencia con el backend `sim` a traves del mismo camino de codigo que
// usaria un microcontrolador real (bytes por un puerto).
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>

#include "atp_control/mount_backend.hpp"
#include "atp_control/serial_port.hpp"
#include "atp_control/serial_protocol.hpp"

using namespace atp_control;   // NOLINT

TEST(Protocolo, ChecksumXor)
{
  EXPECT_EQ(checksum("Z"), 'Z');
  EXPECT_EQ(checksum("AB"), 'A' ^ 'B');
  // trama conocida a mano: $Z*5A
  EXPECT_EQ(encode(ZeroFrame{}), "$Z*5A\n");
}

TEST(Protocolo, IdaYVueltaDeTramas)
{
  CommandFrame c;
  c.seq = 42; c.dt_us = 50000; c.tau[0] = 1.234567; c.tau[1] = -0.5;
  const auto s = encode(c);
  EXPECT_EQ(s.substr(0, 3), "$C,");
  const auto d = decode(s.substr(0, s.size() - 1));
  ASSERT_TRUE(d.has_value());
  const auto & c2 = std::get<CommandFrame>(*d);
  EXPECT_EQ(c2.seq, 42u);
  EXPECT_EQ(c2.dt_us, 50000u);
  EXPECT_NEAR(c2.tau[0], 1.234567, 1e-9);
  EXPECT_NEAR(c2.tau[1], -0.5, 1e-9);

  StateFrame st;
  st.seq = 7; st.t_us = 123456789012ULL; st.cnt[0] = -1234; st.cnt[1] = 65535;
  st.rate[0] = 100; st.rate[1] = -200; st.flags = 6;
  const auto e = encode(st);
  const auto d2 = decode(e.substr(0, e.size() - 1));
  ASSERT_TRUE(d2.has_value());
  const auto & s2 = std::get<StateFrame>(*d2);
  EXPECT_EQ(s2.t_us, 123456789012ULL);
  EXPECT_EQ(s2.cnt[0], -1234);
  EXPECT_EQ(s2.cnt[1], 65535);
  EXPECT_EQ(s2.rate[1], -200);
  EXPECT_EQ(s2.flags, 6u);
}

TEST(Protocolo, RechazaCorruptas)
{
  CommandFrame c;
  auto s = encode(c);
  s.pop_back();
  ASSERT_TRUE(decode(s).has_value());
  auto mala = s;
  mala[5] = '9';                                // un byte cambiado: XOR no coincide
  EXPECT_FALSE(decode(mala).has_value());
  EXPECT_FALSE(decode("$C,1,2*00").has_value());       // campos de menos
  EXPECT_FALSE(decode("C,1,2,3,4*00").has_value());    // sin '$'
  EXPECT_FALSE(decode("$X*58").has_value());           // tipo desconocido
}

TEST(Protocolo, ParserResincronizaConBasuraYTramasPartidas)
{
  FrameParser p;
  StateFrame a, b;
  a.seq = 1; b.seq = 2;
  const std::string sa = encode(a), sb = encode(b);
  // basura, la trama a partida en dos lecturas, una corrupta, y la trama b
  std::string x = "\x01\xff garbage " + sa.substr(0, 9);
  auto f1 = p.feed(x.data(), x.size());
  EXPECT_TRUE(f1.empty());
  std::string y = sa.substr(9) + "$S,1,2,3*00\n" + sb;
  auto f2 = p.feed(y.data(), y.size());
  ASSERT_EQ(f2.size(), 2u);
  EXPECT_EQ(std::get<StateFrame>(f2[0]).seq, 1u);
  EXPECT_EQ(std::get<StateFrame>(f2[1]).seq, 2u);
  EXPECT_GE(p.bad_frames(), 1u);
}

namespace
{

std::map<std::string, std::string> hw(const std::string & backend)
{
  return {{"backend", backend}, {"J", "2.0"}, {"b", "0.5"}, {"serial_port", "fake"},
    {"serial_cpr", "65536"}, {"serial_timeout_ms", "100"}};
}

}  // namespace

TEST(Serial, MismaTrayectoriaQueSimSalvoLaCuantizacion)
{
  // El mismo mando por el backend sim (estado exacto) y por el serial con MCU falso
  // (cuentas de 65536/rev = 0.0055 grados): deben coincidir a media cuenta.
  const HwParams p_sim(hw("sim")), p_ser(hw("serial"));
  auto sim = make_backend("sim");
  auto ser = make_backend("serial");
  const Vec2 ini{0.35, 0.60};
  ASSERT_TRUE(sim->configure(p_sim, ini, 8.0));
  ASSERT_TRUE(ser->configure(p_ser, ini, 8.0));
  ASSERT_TRUE(sim->activate());
  ASSERT_TRUE(ser->activate());
  const double cuenta = 2.0 * M_PI / 65536.0;
  double max_dpos = 0.0, max_dvel = 0.0;
  Vec2 ps{}, vs{}, pr{}, vr{};
  std::string why;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(sim->read(0.05, ps, vs, why)) << why;
    ASSERT_TRUE(ser->read(0.05, pr, vr, why)) << why;
    max_dpos = std::max({max_dpos, std::fabs(ps[0] - pr[0]), std::fabs(ps[1] - pr[1])});
    max_dvel = std::max({max_dvel, std::fabs(vs[0] - vr[0]), std::fabs(vs[1] - vr[1])});
    const Vec2 u{4.0 * std::sin(0.3 * k * 0.05), -3.0 * std::cos(0.2 * k * 0.05)};
    ASSERT_TRUE(sim->write(u, 0.05));
    ASSERT_TRUE(ser->write(u, 0.05));
  }
  std::printf("[serial] max|dpos|=%.3e rad (media cuenta = %.3e)  max|dvel|=%.3e rad/s\n",
    max_dpos, 0.5 * cuenta, max_dvel);
  EXPECT_LE(max_dpos, 0.5 * cuenta + 1e-12);
  EXPECT_LE(max_dvel, cuenta * 20.0 + 1e-9);    // velocidad cuantizada a 1 cuenta/s
}

TEST(Serial, VelocidadPorDerivadaSiElMcuNoLaMide)
{
  auto h = hw("serial");
  h["serial_fake_rate"] = "0";
  h["serial_vel_tau_s"] = "0.02";
  auto ser = make_backend("serial");
  ASSERT_TRUE(ser->configure(HwParams(h), {0.0, 0.0}, 8.0));
  ASSERT_TRUE(ser->activate());
  Vec2 p{}, v{};
  std::string why;
  for (int k = 0; k < 300; ++k) {
    ASSERT_TRUE(ser->read(0.05, p, v, why)) << why;
    ASSERT_TRUE(ser->write({2.0, 0.0}, 0.05));
  }
  // con 2 N m y b = 0.5 la velocidad final es 4 rad/s: la derivada filtrada la sigue
  EXPECT_NEAR(v[0], 4.0, 0.15);
}

TEST(Serial, TimeoutSiElMcuCallaYFallaSiReportaFalla)
{
  // se construye el puerto a mano para poder inyectar fallas
  FakeMcuConfig cfg;
  auto port = std::make_unique<FakeMcuPort>(cfg);
  FakeMcuPort * raw = port.get();
  ASSERT_TRUE(raw->open());
  // $C -> $S sin pasar por el backend
  CommandFrame c; c.seq = 1;
  raw->set_silent(true);
  raw->write(encode(c));
  char buf[64];
  EXPECT_EQ(raw->read(buf, sizeof(buf)), 0u);        // callado: ninguna respuesta
  raw->set_silent(false);
  raw->set_fault(true);
  raw->write(encode(c));
  const size_t n = raw->read(buf, sizeof(buf));
  ASSERT_GT(n, 0u);
  FrameParser p;
  const auto fr = p.feed(buf, n);
  ASSERT_EQ(fr.size(), 1u);
  EXPECT_TRUE(std::get<StateFrame>(fr[0]).flags & StateFrame::kFault);
}

TEST(Serial, VigilanciaEnTiempoDelLazoSilencioYFalla)
{
  auto port = std::make_unique<FakeMcuPort>(FakeMcuConfig{});
  FakeMcuPort * mcu = port.get();
  auto h = hw("serial");
  h["serial_timeout_ms"] = "200";
  auto ser = make_serial_backend(std::move(port));
  ASSERT_TRUE(ser->configure(HwParams(h), {0.0, 0.0}, 8.0));
  ASSERT_TRUE(ser->activate());
  Vec2 p{}, v{};
  std::string why;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(ser->read(0.05, p, v, why)) << why;
    ASSERT_TRUE(ser->write({0.1, 0.1}, 0.05));
  }
  mcu->set_silent(true);                      // el MCU deja de responder
  int ciclos = 0;
  bool fallo = false;
  for (; ciclos < 20 && !fallo; ++ciclos) {
    fallo = !ser->read(0.05, p, v, why);
    ser->write({0.1, 0.1}, 0.05);
  }
  EXPECT_TRUE(fallo);
  EXPECT_NE(why.find("sin tramas"), std::string::npos);
  // la 1.a lectura recoge la trama pendiente; luego 200 ms de silencio con ciclos de 50 ms:
  // falla en el 5.o o 6.o ciclo (segun el redondeo de 4 x 0.05 s contra 0.2 s)
  EXPECT_GE(ciclos, 5);
  EXPECT_LE(ciclos, 6);
}

TEST(Serial, FallaDelMcuLlevaAError)
{
  auto port = std::make_unique<FakeMcuPort>(FakeMcuConfig{});
  FakeMcuPort * mcu = port.get();
  auto ser = make_serial_backend(std::move(port));
  ASSERT_TRUE(ser->configure(HwParams(hw("serial")), {0.0, 0.0}, 8.0));
  ASSERT_TRUE(ser->activate());
  Vec2 p{}, v{};
  std::string why;
  ASSERT_TRUE(ser->read(0.05, p, v, why));
  mcu->set_fault(true);
  ser->write({0.0, 0.0}, 0.05);
  EXPECT_FALSE(ser->read(0.05, p, v, why));
  EXPECT_NE(why.find("falla"), std::string::npos);
}

TEST(Serial, SobreviveABasuraYTramasCorruptasEnElCable)
{
  auto port = std::make_unique<FakeMcuPort>(FakeMcuConfig{});
  FakeMcuPort * mcu = port.get();
  auto ser = make_serial_backend(std::move(port));
  ASSERT_TRUE(ser->configure(HwParams(hw("serial")), {0.3, 0.4}, 8.0));
  ASSERT_TRUE(ser->activate());
  Vec2 p{}, v{};
  std::string why;
  for (int k = 0; k < 40; ++k) {
    mcu->inject("\x01\xfe ruido de linea $S,1,2,3*77\r\n");
    ASSERT_TRUE(ser->read(0.05, p, v, why)) << why;
    ASSERT_TRUE(ser->write({0.5, 0.5}, 0.05));
  }
  EXPECT_GT(p[0], 0.3);        // la montura se movio con el par
}

TEST(Serial, PuertoRealInexistenteFallaAlConfigurar)
{
  auto h = hw("serial");
  h["serial_port"] = "/dev/atp_no_existe";
  auto ser = make_backend("serial");
  EXPECT_FALSE(ser->configure(HwParams(h), {0.0, 0.0}, 8.0));
}

TEST(Serial, BackendDesconocidoLanza)
{
  EXPECT_THROW(make_backend("canbus"), std::invalid_argument);
}
