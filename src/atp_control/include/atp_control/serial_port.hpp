// Puertos serie del backend `serial`: el real (termios) y un microcontrolador
// FALSO en proceso (FakeMcuPort) que habla el mismo protocolo, para probar el
// hardware_interface sin hardware.
#pragma once

#include <memory>
#include <string>

#include "atp_control/mount_plant.hpp"
#include "atp_control/serial_protocol.hpp"

namespace atp_control
{

/// Puerto de bytes sin bloqueo.
class SerialPort
{
public:
  virtual ~SerialPort() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  /// Escribe todo `data`; false si fallo.
  virtual bool write(const std::string & data) = 0;
  /// Lee hasta n bytes disponibles YA (sin esperar); devuelve cuantos.
  virtual size_t read(char * buf, size_t n) = 0;
  virtual std::string name() const = 0;
};

/// Puerto POSIX (/dev/ttyUSB0, /dev/ttyACM0...): 8N1 crudo, sin control de flujo, O_NONBLOCK.
class PosixSerialPort : public SerialPort
{
public:
  PosixSerialPort(std::string path, int baud) : path_(std::move(path)), baud_(baud) {}
  ~PosixSerialPort() override {close();}
  bool open() override;
  void close() override;
  bool is_open() const override {return fd_ >= 0;}
  bool write(const std::string & data) override;
  size_t read(char * buf, size_t n) override;
  std::string name() const override {return path_;}

private:
  std::string path_;
  int baud_;
  int fd_ = -1;
};

struct FakeMcuConfig
{
  PlantParams plant;               // la "montura fisica" detras del MCU
  Vec2 initial{0.0, 0.0};          // posicion inicial de la carga, rad
  double cpr = 65536.0;            // cuentas por revolucion
  double u_max = 8.0;              // limitador de par del MCU, N m
  bool report_rate = true;         // si el MCU mide velocidad (flag RateValid)
};

/// Un MCU simulado: aplica cada $C integrando la planta `dt_us`, y responde un $S.
/// Sincrono (la respuesta ya esta disponible en el siguiente read()), que es lo
/// que hace determinista una prueba; un MCU real responde con su propia latencia.
class FakeMcuPort : public SerialPort
{
public:
  explicit FakeMcuPort(const FakeMcuConfig & cfg);
  bool open() override {open_ = true; return true;}
  void close() override {open_ = false;}
  bool is_open() const override {return open_;}
  bool write(const std::string & data) override;
  size_t read(char * buf, size_t n) override;
  std::string name() const override {return "fake";}

  // inyeccion de fallas para las pruebas
  void set_silent(bool s) {silent_ = s;}                // deja de responder (timeout del host)
  void set_fault(bool f) {fault_ = f;}                  // levanta el flag de falla
  void inject(const std::string & bytes) {rx_ += bytes;}  // basura / tramas corruptas

  const MountPlant & plant() const {return plant_;}
  uint32_t commands_received() const {return n_cmd_;}

private:
  void respond();

  FakeMcuConfig cfg_;
  MountPlant plant_;
  FrameParser parser_;
  bool open_ = false, silent_ = false, fault_ = false;
  std::string rx_;                    // lo que el host leera
  int64_t zero_[2] = {0, 0};
  bool homed_ = false;
  uint32_t seq_ = 0, n_cmd_ = 0;
  uint64_t t_us_ = 0;
};

/// "fake" -> FakeMcuPort con `cfg`; si no, PosixSerialPort(path, baud).
std::unique_ptr<SerialPort> make_port(
  const std::string & port, int baud, const FakeMcuConfig & cfg);

}  // namespace atp_control
