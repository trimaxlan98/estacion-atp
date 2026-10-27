// Backends de AtpMountSystem: quien pone el estado de los dos ejes.
//   sim     integra la dinamica de planta.py dentro de read() (MountPlant)
//   serial  habla con un microcontrolador por el protocolo de serial_protocol.hpp
// El plugin es el mismo; el backend se elige con el parametro `backend` de la
// etiqueta <hardware> del URDF.
#pragma once

#include <map>
#include <memory>
#include <string>

#include "atp_control/mount_plant.hpp"
#include "atp_control/serial_port.hpp"

namespace atp_control
{

/// Parametros de la etiqueta <hardware> (cadenas del URDF) con valores por defecto.
class HwParams
{
public:
  template<class Map>
  explicit HwParams(const Map & m) : m_(m.begin(), m.end()) {}
  double num(const std::string & k, double def) const;
  int integer(const std::string & k, int def) const;
  std::string str(const std::string & k, const std::string & def) const;

private:
  std::map<std::string, std::string> m_;
};

PlantParams plant_params_from(const HwParams & p);

class MountBackend
{
public:
  virtual ~MountBackend() = default;
  /// Prepara el backend con la posicion inicial (rad) de los dos ejes. false = error.
  virtual bool configure(const HwParams & p, const Vec2 & initial, double u_max) = 0;
  /// Al activar el hardware (el serial hace el saludo con el MCU).
  virtual bool activate() {return true;}
  virtual void deactivate() {}
  /// Estado en este instante: avanza `dt` s con el ultimo mando escrito. false = error.
  virtual bool read(double dt, Vec2 & pos, Vec2 & vel, std::string & why) = 0;
  /// Mando de par (N m) para el siguiente periodo (`dt` = periodo del lazo).
  virtual bool write(const Vec2 & u, double dt) = 0;
  virtual std::string describe() const = 0;
};

std::unique_ptr<MountBackend> make_backend(const std::string & name);

/// Backend serial sobre un puerto dado (pruebas: un FakeMcuPort con fallas inyectadas).
std::unique_ptr<MountBackend> make_serial_backend(std::unique_ptr<SerialPort> port);

}  // namespace atp_control
