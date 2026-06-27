#include "jetank_motor_control/feetech_bus.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <numeric>

namespace jetank_motor_control
{

namespace
{
/// Map an integer baud rate to a termios speed_t constant. Returns B0 if the
/// rate is unsupported (caller treats B0 as failure).
speed_t to_speed(int baud)
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
    default: return B0;
  }
}
}  // namespace

FeetechBus::~FeetechBus()
{
  close();
}

bool FeetechBus::open(const std::string & port, int baud_rate)
{
  close();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  const speed_t speed = to_speed(baud_rate);
  if (speed == B0) {
    close();
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem ctrl lines, enable receiver
  tty.c_cflag &= ~CSTOPB;            // 1 stop bit
  tty.c_cflag &= ~PARENB;            // no parity
  tty.c_cflag &= ~CRTSCTS;           // no hardware flow control
  tty.c_cc[VMIN] = 0;                // non-blocking read; we poll with timeout
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void FeetechBus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int FeetechBus::read_bytes(uint8_t * buf, int n, int timeout_ms)
{
  int got = 0;
  // Simple poll loop; the bus is fast (1 Mbps) so total wait is short.
  for (int waited = 0; got < n && waited < timeout_ms; ) {
    const ssize_t r = ::read(fd_, buf + got, n - got);
    if (r > 0) {
      got += static_cast<int>(r);
    } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      break;
    } else {
      usleep(200);          // 0.2 ms
      waited += 1;          // approximate; coarse but adequate for framing
    }
  }
  return got;
}

bool FeetechBus::send_packet(
  uint8_t id, uint8_t instruction, const std::vector<uint8_t> & params)
{
  if (fd_ < 0) {
    return false;
  }
  const uint8_t length = static_cast<uint8_t>(params.size() + 2);

  std::vector<uint8_t> packet;
  packet.reserve(params.size() + 6);
  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(id);
  packet.push_back(length);
  packet.push_back(instruction);
  packet.insert(packet.end(), params.begin(), params.end());

  uint32_t sum = id + length + instruction;
  sum = std::accumulate(params.begin(), params.end(), sum);
  packet.push_back(static_cast<uint8_t>(~sum & 0xFF));

  tcflush(fd_, TCIFLUSH);  // drop any stale bytes before a fresh transaction
  const ssize_t written = ::write(fd_, packet.data(), packet.size());
  return written == static_cast<ssize_t>(packet.size());
}

bool FeetechBus::recv_packet(
  uint8_t expected_id, std::vector<uint8_t> & params, uint8_t expected_param_len)
{
  // Status packet: 0xFF 0xFF ID LEN ERR PARAMS... CHECKSUM
  // LEN = param_len + 2 (ERR + CHECKSUM).
  const int total = expected_param_len + 6;
  std::vector<uint8_t> buf(total);
  const int got = read_bytes(buf.data(), total, 50 /*ms*/);
  if (got < total) {
    return false;
  }
  if (buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != expected_id) {
    return false;
  }
  const uint8_t length = buf[3];
  if (length != expected_param_len + 2) {
    return false;
  }

  uint32_t sum = buf[2] + buf[3] + buf[4];  // ID + LEN + ERR
  for (uint8_t i = 0; i < expected_param_len; ++i) {
    sum += buf[5 + i];
  }
  const uint8_t checksum = buf[5 + expected_param_len];
  if (checksum != static_cast<uint8_t>(~sum & 0xFF)) {
    return false;
  }

  params.assign(buf.begin() + 5, buf.begin() + 5 + expected_param_len);
  return true;
}

bool FeetechBus::write_register(
  uint8_t id, uint8_t reg, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params;
  params.reserve(data.size() + 1);
  params.push_back(reg);
  params.insert(params.end(), data.begin(), data.end());
  if (!send_packet(id, INST_WRITE, params)) {
    return false;
  }
  // A WRITE elicits a status packet with no params (unless the servo is in
  // no-reply mode). Consume it if present, but do not fail the write if the
  // servo is configured to stay silent on writes.
  std::vector<uint8_t> resp;
  recv_packet(id, resp, 0);
  return true;
}

bool FeetechBus::read_register(
  uint8_t id, uint8_t reg, uint8_t length, std::vector<uint8_t> & out)
{
  const std::vector<uint8_t> params{reg, length};
  if (!send_packet(id, INST_READ, params)) {
    return false;
  }
  return recv_packet(id, out, length);
}

bool FeetechBus::ping(uint8_t id)
{
  if (!send_packet(id, INST_PING, {})) {
    return false;
  }
  std::vector<uint8_t> resp;
  return recv_packet(id, resp, 0);
}

bool FeetechBus::set_torque(uint8_t id, bool on)
{
  return write_register(id, REG_TORQUE_ENABLE, {static_cast<uint8_t>(on ? 1 : 0)});
}

bool FeetechBus::write_goal_position(uint8_t id, uint16_t ticks)
{
  const std::vector<uint8_t> data{
    static_cast<uint8_t>(ticks & 0xFF),
    static_cast<uint8_t>((ticks >> 8) & 0xFF)};
  return write_register(id, REG_GOAL_POSITION, data);
}

bool FeetechBus::read_present_position(uint8_t id, uint16_t & ticks)
{
  std::vector<uint8_t> out;
  if (!read_register(id, REG_PRESENT_POSITION, 2, out) || out.size() != 2) {
    return false;
  }
  ticks = static_cast<uint16_t>(out[0] | (out[1] << 8));
  return true;
}

}  // namespace jetank_motor_control
