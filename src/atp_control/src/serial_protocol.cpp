#include "atp_control/serial_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace atp_control
{

uint8_t checksum(const std::string & payload)
{
  uint8_t x = 0;
  for (unsigned char c : payload) {
    x ^= c;
  }
  return x;
}

namespace
{

std::string with_checksum(const std::string & payload)
{
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\n", checksum(payload));
  return "$" + payload + tail;
}

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  std::istringstream ss(s);
  while (std::getline(ss, cur, ',')) {
    out.push_back(cur);
  }
  if (!s.empty() && s.back() == ',') {
    out.emplace_back();
  }
  return out;
}

bool to_int(const std::string & s, int64_t & v)
{
  if (s.empty()) {return false;}
  char * end = nullptr;
  v = std::strtoll(s.c_str(), &end, 10);
  return *end == '\0';
}

bool to_double(const std::string & s, double & v)
{
  if (s.empty()) {return false;}
  char * end = nullptr;
  v = std::strtod(s.c_str(), &end);
  return *end == '\0';
}

}  // namespace

std::string encode(const Frame & f)
{
  char buf[200];
  if (const auto * c = std::get_if<CommandFrame>(&f)) {
    std::snprintf(buf, sizeof(buf), "C,%u,%u,%.9f,%.9f", c->seq, c->dt_us, c->tau[0], c->tau[1]);
    return with_checksum(buf);
  }
  if (std::holds_alternative<ZeroFrame>(f)) {
    return with_checksum("Z");
  }
  const auto & s = std::get<StateFrame>(f);
  std::snprintf(
    buf, sizeof(buf), "S,%u,%llu,%lld,%lld,%lld,%lld,%u", s.seq,
    static_cast<unsigned long long>(s.t_us), static_cast<long long>(s.cnt[0]),
    static_cast<long long>(s.cnt[1]), static_cast<long long>(s.rate[0]),
    static_cast<long long>(s.rate[1]), s.flags);
  return with_checksum(buf);
}

std::optional<Frame> decode(const std::string & line)
{
  if (line.size() < 5 || line[0] != '$') {return std::nullopt;}
  const auto star = line.rfind('*');
  if (star == std::string::npos || line.size() != star + 3) {return std::nullopt;}
  const std::string payload = line.substr(1, star - 1);
  char * end = nullptr;
  const unsigned long hh = std::strtoul(line.substr(star + 1, 2).c_str(), &end, 16);
  if (*end != '\0' || hh != checksum(payload)) {return std::nullopt;}
  const auto f = split(payload);
  if (f.empty()) {return std::nullopt;}
  int64_t a[8] = {};
  if (f[0] == "Z" && f.size() == 1) {
    return Frame{ZeroFrame{}};
  }
  if (f[0] == "C" && f.size() == 5) {
    CommandFrame c;
    if (!to_int(f[1], a[0]) || !to_int(f[2], a[1]) || !to_double(f[3], c.tau[0]) ||
      !to_double(f[4], c.tau[1]))
    {
      return std::nullopt;
    }
    c.seq = static_cast<uint32_t>(a[0]);
    c.dt_us = static_cast<uint32_t>(a[1]);
    return Frame{c};
  }
  if (f[0] == "S" && f.size() == 8) {
    StateFrame s;
    for (int i = 0; i < 7; ++i) {
      if (!to_int(f[i + 1], a[i])) {return std::nullopt;}
    }
    s.seq = static_cast<uint32_t>(a[0]);
    s.t_us = static_cast<uint64_t>(a[1]);
    s.cnt[0] = a[2];
    s.cnt[1] = a[3];
    s.rate[0] = a[4];
    s.rate[1] = a[5];
    s.flags = static_cast<uint32_t>(a[6]);
    return Frame{s};
  }
  return std::nullopt;
}

std::vector<Frame> FrameParser::feed(const char * data, size_t n)
{
  std::vector<Frame> out;
  buf_.append(data, n);
  for (;;) {
    const auto nl = buf_.find('\n');
    if (nl == std::string::npos) {break;}
    std::string line = buf_.substr(0, nl);
    buf_.erase(0, nl + 1);
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    // resincroniza: lo que haya antes del ultimo '$' de la linea es basura
    const auto d = line.rfind('$');
    if (d == std::string::npos) {
      if (!line.empty()) {++bad_;}
      continue;
    }
    if (auto fr = decode(line.substr(d))) {
      out.push_back(*fr);
    } else {
      ++bad_;
    }
  }
  if (buf_.size() > 4096) {   // sin '\n' en mucho rato: ruido, se tira
    buf_.clear();
    ++bad_;
  }
  return out;
}

}  // namespace atp_control
