#ifndef JETANK_MOTOR_CONTROL__FEETECH_BUS_HPP_
#define JETANK_MOTOR_CONTROL__FEETECH_BUS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace jetank_motor_control
{

/// Minimal Feetech serial bus servo protocol (SMS/STS family) over a POSIX
/// termios serial port. No external SDK dependency.
///
/// Protocol summary (half-duplex UART, default 1 Mbps):
///   Instruction packet: 0xFF 0xFF ID LEN INST PARAMS... CHECKSUM
///   Status packet:      0xFF 0xFF ID LEN ERR PARAMS... CHECKSUM
///   LEN      = number of params + 2
///   CHECKSUM = ~(ID + LEN + INST + sum(params)) & 0xFF
///
/// Register map / encoding assumes the SMS_STS family (e.g. STS3215):
///   - positions/speeds are 16-bit little-endian
///   - position range 0..4095 over 0..360 deg
///
/// IMPORTANT: confirm the register map + endianness against the actual servos
/// (via the hardware-test MCP feetech tools) before enabling torque on real
/// hardware. The constants are isolated here so they are easy to adjust.
class FeetechBus
{
public:
  FeetechBus() = default;
  ~FeetechBus();

  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

  /// Open the serial port at the given baud rate. Returns false on failure.
  bool open(const std::string & port, int baud_rate);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// Probe a servo. Returns true if it responds to a PING.
  bool ping(uint8_t id);

  /// Enable/disable torque on a servo.
  bool set_torque(uint8_t id, bool on);

  /// Command a goal position (raw ticks, 0..4095).
  bool write_goal_position(uint8_t id, uint16_t ticks);

  /// Read the present position (raw ticks). Returns false on timeout/error.
  bool read_present_position(uint8_t id, uint16_t & ticks);

  /// SMS/STS control-table register addresses (memory map).
  enum Register : uint8_t
  {
    REG_TORQUE_ENABLE = 40,    // 0x28
    REG_GOAL_POSITION = 42,    // 0x2A (2 bytes, LE)
    REG_PRESENT_POSITION = 56  // 0x38 (2 bytes, LE)
  };

  static constexpr uint16_t TICKS_PER_REV = 4096;  // 0..4095 over 360 deg
  static constexpr uint16_t CENTER_TICKS = 2048;   // tick at 0 rad (default)

private:
  enum Instruction : uint8_t
  {
    INST_PING = 0x01,
    INST_READ = 0x02,
    INST_WRITE = 0x03
  };

  bool write_register(uint8_t id, uint8_t reg, const std::vector<uint8_t> & data);
  bool read_register(uint8_t id, uint8_t reg, uint8_t length, std::vector<uint8_t> & out);

  /// Send a raw instruction packet (header/len/checksum added here).
  bool send_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t> & params);
  /// Read a status packet body (params only, checksum verified). Returns false
  /// on timeout/framing/checksum error.
  bool recv_packet(uint8_t expected_id, std::vector<uint8_t> & params, uint8_t expected_param_len);

  /// Blocking read of n bytes with a per-call timeout. Returns bytes read.
  int read_bytes(uint8_t * buf, int n, int timeout_ms);

  int fd_ = -1;
};

}  // namespace jetank_motor_control

#endif  // JETANK_MOTOR_CONTROL__FEETECH_BUS_HPP_
