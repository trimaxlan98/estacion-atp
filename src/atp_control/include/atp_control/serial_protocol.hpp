// Protocolo serie entre el hardware_interface (host) y el microcontrolador de la montura.
//
// ESTE PROTOCOLO ES UN BORRADOR DE INTERFAZ: lo define el lado ros2_control para
// que el firmware (ESP32, el trabajo del equipo J con micro-ROS o un firmware
// serie plano) lo implemente, o se adapte aqui `SerialProtocol` en un solo
// archivo. Texto ASCII, una trama por linea, comprobacion XOR al estilo NMEA:
//
//   $<tipo>,<campo>,<campo>...*<HH>\n         HH = XOR de los bytes entre '$' y '*', 2 hex
//
//   host -> MCU
//   $C,<seq>,<dt_us>,<tau_az>,<tau_el>*HH     mando de par (N m) de los dos ejes
//   $Z*HH                                     fijar el cero de los encoders en la posicion actual
//   MCU -> host
//   $S,<seq>,<t_us>,<az_cnt>,<el_cnt>,<az_rate>,<el_rate>,<flags>*HH     estado
//
//   seq     contador del mando; el MCU devuelve en $S el del ultimo $C que aplico
//   dt_us   periodo del lazo del host (us). Informativo para el MCU real (que usa
//           su propio temporizador); el MCU simulado (FakeMcuPort) lo integra
//   t_us    reloj propio del MCU (us), para estimar la edad de la muestra
//   *_cnt   cuentas del encoder (entero con signo) respecto del cero; CPR por revolucion
//   *_rate  velocidad en cuentas/s (0 si el MCU no la mide; bit 2 de flags dice si es valida)
//   flags   bit0 = falla/paro de emergencia (el host pasa a ERROR), bit1 = con cero fijado
//           (homed), bit2 = *_rate valida
//
// Seguridad: el MCU DEBE cortar el par si no recibe $C en `watchdog` ms (el
// host, por su lado, pasa a ERROR si no recibe $S en `serial_timeout_ms`).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace atp_control
{

struct CommandFrame
{
  uint32_t seq = 0;
  uint32_t dt_us = 50000;
  double tau[2] = {0.0, 0.0};
};

struct ZeroFrame {};

struct StateFrame
{
  uint32_t seq = 0;
  uint64_t t_us = 0;
  int64_t cnt[2] = {0, 0};
  int64_t rate[2] = {0, 0};
  uint32_t flags = 0;

  static constexpr uint32_t kFault = 1u << 0;
  static constexpr uint32_t kHomed = 1u << 1;
  static constexpr uint32_t kRateValid = 1u << 2;
};

using Frame = std::variant<CommandFrame, ZeroFrame, StateFrame>;

/// XOR de los bytes de `payload` (lo que va entre '$' y '*').
uint8_t checksum(const std::string & payload);

/// Trama completa con '$', '*HH' y '\n'.
std::string encode(const Frame & f);

/// Decodifica una linea (sin '\n'); nullopt si esta mal formada o falla el XOR.
std::optional<Frame> decode(const std::string & line);

/// Acumula bytes y entrega las tramas completas. Descarta basura entre tramas
/// (resincroniza en el siguiente '$') y limita el buffer; cuenta las malas.
class FrameParser
{
public:
  std::vector<Frame> feed(const char * data, size_t n);
  size_t bad_frames() const {return bad_;}

private:
  std::string buf_;
  size_t bad_ = 0;
};

}  // namespace atp_control
