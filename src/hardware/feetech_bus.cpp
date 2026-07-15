#include "jetank_motor_control/feetech_bus.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <numeric>

namespace jetank_motor_control
{

namespace
{
/// Reply-wait ceiling per transaction. Matches the previous implementation's
/// 60 * 0.2 ms usleep-spin budget so timeout semantics are unchanged.
constexpr int kReplyTimeoutMs = 12;

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

bool FeetechBus::send_packet(
  uint8_t id, uint8_t instruction, const uint8_t * params, size_t param_len)
{
  if (fd_ < 0) {
    return false;
  }
  const uint8_t length = static_cast<uint8_t>(param_len + 2);

  // Assemble the frame directly in last_tx_: it doubles as the echo-strip
  // reference for recv_packet, and reusing its capacity means no per-call
  // heap allocation in the 50 Hz read/write path.
  last_tx_.clear();
  last_tx_.reserve(param_len + 6);  // no-op once capacity is warm
  last_tx_.push_back(0xFF);
  last_tx_.push_back(0xFF);
  last_tx_.push_back(id);
  last_tx_.push_back(length);
  last_tx_.push_back(instruction);
  last_tx_.insert(last_tx_.end(), params, params + param_len);

  uint32_t sum = id + length + instruction;
  sum = std::accumulate(params, params + param_len, sum);
  last_tx_.push_back(static_cast<uint8_t>(~sum & 0xFF));

  tcflush(fd_, TCIFLUSH);  // drop any stale bytes (incl. unread echoes) first
  const ssize_t written = ::write(fd_, last_tx_.data(), last_tx_.size());
  return written == static_cast<ssize_t>(last_tx_.size());
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
  // rx_buf_ is a reusable member: clear() keeps its capacity, so the 50 Hz
  // path performs no per-transaction heap allocation once warm.
  std::vector<uint8_t> & buf = rx_buf_;
  buf.clear();
  buf.reserve(status_len + last_tx_.size() + 8);

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(kReplyTimeoutMs);
  while (true) {
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
      // No data pending: block on poll(2) until bytes arrive or the deadline
      // passes, instead of busy-spinning with usleep. A present reply still
      // returns as soon as its frame parses; an absent reply pays at most
      // kReplyTimeoutMs, exactly as before.
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()) + 1;
      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN;
      const int pr = ::poll(&pfd, 1, remaining_ms);
      if (pr < 0 && errno != EINTR) {
        break;
      }
    }
  }
  return false;
}

bool FeetechBus::write_register(
  uint8_t id, uint8_t reg, const uint8_t * data, size_t data_len)
{
  // Build [reg, data...] in the reusable scratch buffer (capacity kept across
  // calls — no per-call allocation once warm).
  scratch_.clear();
  scratch_.reserve(data_len + 1);
  scratch_.push_back(reg);
  scratch_.insert(scratch_.end(), data, data + data_len);
  if (!send_packet(id, INST_WRITE, scratch_.data(), scratch_.size())) {
    return false;
  }
  if (expect_write_replies_) {
    // Default SRM (status return level) replies to every instruction: consume
    // the zero-param status packet if present (poll deadline as before), but
    // do not fail the write if the servo stays silent.
    recv_packet(id, scratch_, 0);
  }
  // else: servos configured for no-reply mode on writes — return immediately.
  // Any unexpected reply/echo bytes left on the line are flushed by the
  // tcflush(TCIFLUSH) at the start of the next send_packet.
  return true;
}

bool FeetechBus::read_register(
  uint8_t id, uint8_t reg, uint8_t length, std::vector<uint8_t> & out)
{
  const uint8_t params[2] = {reg, length};
  if (!send_packet(id, INST_READ, params, sizeof(params))) {
    return false;
  }
  return recv_packet(id, out, length);
}

bool FeetechBus::ping(uint8_t id)
{
  if (!send_packet(id, INST_PING, nullptr, 0)) {
    return false;
  }
  return recv_packet(id, scratch_, 0);
}

bool FeetechBus::set_torque(uint8_t id, bool on)
{
  const uint8_t value = on ? 1 : 0;
  return write_register(id, REG_TORQUE_ENABLE, &value, 1);
}

bool FeetechBus::write_goal_position(uint8_t id, uint16_t ticks)
{
  // SCS is BIG-endian: high byte first.
  const uint8_t data[2] = {
    static_cast<uint8_t>((ticks >> 8) & 0xFF),
    static_cast<uint8_t>(ticks & 0xFF)};
  return write_register(id, REG_GOAL_POSITION, data, sizeof(data));
}

bool FeetechBus::read_present_position(uint8_t id, uint16_t & ticks)
{
  if (!read_register(id, REG_PRESENT_POSITION, 2, scratch_) || scratch_.size() != 2) {
    return false;
  }
  ticks = static_cast<uint16_t>((scratch_[0] << 8) | scratch_[1]);  // SCS big-endian
  return true;
}

}  // namespace jetank_motor_control
