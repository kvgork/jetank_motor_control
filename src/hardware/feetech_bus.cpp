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
  last_tx_ = packet;       // remembered so recv_packet can strip the TX echo
  const ssize_t written = ::write(fd_, packet.data(), packet.size());
  return written == static_cast<ssize_t>(packet.size());
}

bool FeetechBus::recv_packet(
  uint8_t expected_id, std::vector<uint8_t> & params, uint8_t expected_param_len)
{
  if (fd_ < 0) {
    return false;
  }
  // Status packet: 0xFF 0xFF ID LEN ERR PARAMS... CHECKSUM ; LEN = param_len + 2.
  const size_t status_len = static_cast<size_t>(expected_param_len) + 6;

  // This half-duplex UART reflects our TX, so the reply may be preceded by an
  // echo of last_tx_. Accumulate bytes with a deadline and parse incrementally,
  // returning as soon as a valid status frame is found (so a present reply does
  // not pay the full timeout). The echo is a valid instruction packet with the
  // same id, so we skip a leading copy of last_tx_ before scanning.
  std::vector<uint8_t> buf;
  buf.reserve(status_len + last_tx_.size() + 8);

  for (int waited = 0; waited < 60; ) {        // ~12 ms ceiling (60 * 0.2 ms)
    uint8_t tmp[64];
    const ssize_t r = ::read(fd_, tmp, sizeof(tmp));
    if (r > 0) {
      buf.insert(buf.end(), tmp, tmp + r);

      // Skip a leading TX echo if present.
      size_t start = 0;
      if (buf.size() >= last_tx_.size() &&
        std::equal(last_tx_.begin(), last_tx_.end(), buf.begin()))
      {
        start = last_tx_.size();
      }

      // Frame-sync: scan for FF FF <id> <len> with a matching checksum.
      for (size_t i = start; i + status_len <= buf.size(); ++i) {
        if (buf[i] != 0xFF || buf[i + 1] != 0xFF || buf[i + 2] != expected_id) {
          continue;
        }
        if (buf[i + 3] != expected_param_len + 2) {
          continue;
        }
        uint32_t sum = buf[i + 2] + buf[i + 3] + buf[i + 4];  // ID + LEN + ERR
        for (uint8_t k = 0; k < expected_param_len; ++k) {
          sum += buf[i + 5 + k];
        }
        if (static_cast<uint8_t>(~sum & 0xFF) != buf[i + 5 + expected_param_len]) {
          continue;
        }
        params.assign(buf.begin() + i + 5, buf.begin() + i + 5 + expected_param_len);
        return true;
      }
    } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      break;
    } else {
      usleep(200);
      waited += 1;
    }
  }
  return false;
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
  // SCS is BIG-endian: high byte first.
  const std::vector<uint8_t> data{
    static_cast<uint8_t>((ticks >> 8) & 0xFF),
    static_cast<uint8_t>(ticks & 0xFF)};
  return write_register(id, REG_GOAL_POSITION, data);
}

bool FeetechBus::read_present_position(uint8_t id, uint16_t & ticks)
{
  std::vector<uint8_t> out;
  if (!read_register(id, REG_PRESENT_POSITION, 2, out) || out.size() != 2) {
    return false;
  }
  ticks = static_cast<uint16_t>((out[0] << 8) | out[1]);  // SCS big-endian
  return true;
}

}  // namespace jetank_motor_control
