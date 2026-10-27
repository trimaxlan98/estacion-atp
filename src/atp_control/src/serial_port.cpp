#include "atp_control/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

namespace atp_control
{

namespace
{

speed_t baud_const(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return B115200;
  }
}

}  // namespace

bool PosixSerialPort::open()
{
  fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {return false;}
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {close(); return false;}
  cfmakeraw(&tio);                       // 8N1 crudo
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;               // sin control de flujo
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, baud_const(baud_));
  cfsetospeed(&tio, baud_const(baud_));
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {close(); return false;}
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void PosixSerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool PosixSerialPort::write(const std::string & data)
{
  if (fd_ < 0) {return false;}
  size_t off = 0;
  int reintentos = 0;
  while (off < data.size()) {
    const ssize_t w = ::write(fd_, data.data() + off, data.size() - off);
    if (w > 0) {
      off += static_cast<size_t>(w);
    } else if (w < 0 && errno == EAGAIN && reintentos++ < 100) {
      usleep(100);                       // buffer de TX lleno: espera un poco
    } else {
      return false;
    }
  }
  return true;
}

size_t PosixSerialPort::read(char * buf, size_t n)
{
  if (fd_ < 0) {return 0;}
  const ssize_t r = ::read(fd_, buf, n);
  return r > 0 ? static_cast<size_t>(r) : 0;
}

// ------------------------------------------------------------------ MCU falso

FakeMcuPort::FakeMcuPort(const FakeMcuConfig & cfg)
: cfg_(cfg), plant_(cfg.plant)
{
  plant_.engage(cfg.initial);
}

bool FakeMcuPort::write(const std::string & data)
{
  if (!open_) {return false;}
  for (const auto & fr : parser_.feed(data.data(), data.size())) {
    if (const auto * c = std::get_if<CommandFrame>(&fr)) {
      ++n_cmd_;
      seq_ = c->seq;
      const double dt = c->dt_us * 1e-6;
      const Vec2 u{std::min(std::max(c->tau[0], -cfg_.u_max), cfg_.u_max),
        std::min(std::max(c->tau[1], -cfg_.u_max), cfg_.u_max)};
      plant_.advance(u, {0.0, 0.0}, dt);
      t_us_ += c->dt_us;
      if (!silent_) {respond();}
    } else if (std::holds_alternative<ZeroFrame>(fr)) {
      const Vec2 th = plant_.encoder_angle();
      for (int i = 0; i < 2; ++i) {
        zero_[i] = static_cast<int64_t>(std::nearbyint(th[i] / (2.0 * M_PI) * cfg_.cpr));
      }
      homed_ = true;
    }
  }
  return true;
}

void FakeMcuPort::respond()
{
  const Vec2 th = plant_.encoder_angle();
  StateFrame s;
  s.seq = seq_;
  s.t_us = t_us_;
  for (int i = 0; i < 2; ++i) {
    s.cnt[i] = static_cast<int64_t>(std::nearbyint(th[i] / (2.0 * M_PI) * cfg_.cpr)) - zero_[i];
    s.rate[i] = cfg_.report_rate ?
      static_cast<int64_t>(std::nearbyint(plant_.omega()[i] / (2.0 * M_PI) * cfg_.cpr)) : 0;
  }
  s.flags = (fault_ ? StateFrame::kFault : 0u) | (homed_ ? StateFrame::kHomed : 0u) |
    (cfg_.report_rate ? StateFrame::kRateValid : 0u);
  rx_ += encode(s);
}

size_t FakeMcuPort::read(char * buf, size_t n)
{
  const size_t k = std::min(n, rx_.size());
  std::memcpy(buf, rx_.data(), k);
  rx_.erase(0, k);
  return k;
}

std::unique_ptr<SerialPort> make_port(
  const std::string & port, int baud, const FakeMcuConfig & cfg)
{
  if (port == "fake") {
    return std::make_unique<FakeMcuPort>(cfg);
  }
  return std::make_unique<PosixSerialPort>(port, baud);
}

}  // namespace atp_control
