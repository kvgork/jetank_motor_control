#ifndef JETANK_MOTOR_CONTROL__FEETECH_BUS_HPP_
#define JETANK_MOTOR_CONTROL__FEETECH_BUS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace jetank_motor_control
{

/// Minimal Feetech serial bus servo protocol (SCS / SCSCL family) over a POSIX
/// termios serial port. No external SDK dependency.
///
/// Protocol summary (half-duplex UART, default 1 Mbps):
///   Instruction packet: 0xFF 0xFF ID LEN INST PARAMS... CHECKSUM
///   Status packet:      0xFF 0xFF ID LEN ERR PARAMS... CHECKSUM
///   LEN      = number of params + 2
///   CHECKSUM = ~(ID + LEN + INST + sum(params)) & 0xFF
///
/// Register map / encoding for the SCS family (JeTank arm = 4x SCS15-AP + 1x
/// SCS15-S), VERIFIED on hardware 2026-06-27:
///   - positions are 16-bit BIG-endian (high byte first) — NOT little-endian
///     like the SMS/STS family. Decoding LE gives nonsense >1023 values.
///   - position range 0..1023 (10-bit) over ~300 deg, center 512.
///
/// This UART (Jetson /dev/ttyTHS1) reflects the TX bytes on the shared
/// half-duplex line, so recv_packet strips the echo and frame-syncs on a valid
/// status packet rather than trusting a fixed-offset read.
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

  /// Whether WRITE instructions are expected to elicit a status reply.
  /// The SCS status-return level (SRM) is factory-default "reply to
  /// everything"; this driver does not (yet) reconfigure it, so the safe
  /// default is true: after a WRITE we wait (poll deadline) to consume the
  /// status packet. Set to false only when the servos are configured for
  /// no-reply mode on writes — then write_register returns immediately after
  /// TX, saving up to the reply timeout per servo per write() cycle. READ and
  /// PING always wait for their reply regardless of this flag.
  void set_expect_write_replies(bool expect) {expect_write_replies_ = expect;}
  bool expect_write_replies() const {return expect_write_replies_;}

  /// Enable/disable torque on a servo.
  bool set_torque(uint8_t id, bool on);

  /// Command a goal position (raw ticks, 0..4095).
  bool write_goal_position(uint8_t id, uint16_t ticks);

  /// Read the present position (raw ticks). Returns false on timeout/error.
  bool read_present_position(uint8_t id, uint16_t & ticks);

  /// SCS / SCSCL control-table register addresses (memory map).
  enum Register : uint8_t
  {
    REG_TORQUE_ENABLE = 40,    // 0x28
    REG_GOAL_POSITION = 42,    // 0x2A (2 bytes, BIG-endian on SCS)
    REG_PRESENT_POSITION = 56  // 0x38 (2 bytes, BIG-endian on SCS)
  };

  // SCS15: 10-bit position, 0..1023 over ~300 deg, center 512.
  static constexpr uint16_t POSITION_FULLSCALE = 1024;  // 0..1023 usable
  static constexpr uint16_t POSITION_MAX = 1023;
  static constexpr uint16_t CENTER_TICKS = 512;         // tick at 0 rad (default)
  static constexpr double SPAN_DEGREES = 300.0;         // SCS15 angular span

private:
  enum Instruction : uint8_t
  {
    INST_PING = 0x01,
    INST_READ = 0x02,
    INST_WRITE = 0x03
  };

  bool write_register(uint8_t id, uint8_t reg, const uint8_t * data, size_t data_len);
  bool read_register(uint8_t id, uint8_t reg, uint8_t length, std::vector<uint8_t> & out);

  /// Send a raw instruction packet (header/len/checksum added here). The
  /// packet is assembled in the reusable last_tx_ member buffer — no per-call
  /// heap allocation in the steady state (50 Hz read/write path).
  /// `params` must NOT alias last_tx_: the frame is built into last_tx_
  /// (clear + push_back) while reading through `params`, so a pointer into
  /// last_tx_ would be read after invalidation (asserted in debug builds).
  bool send_packet(uint8_t id, uint8_t instruction, const uint8_t * params, size_t param_len);
  /// Read a status packet body (params only, checksum verified). Returns false
  /// on timeout/framing/checksum error.
  bool recv_packet(uint8_t expected_id, std::vector<uint8_t> & params, uint8_t expected_param_len);

  int fd_ = -1;
  /// Skip the reply-wait after WRITE instructions when false (see setter).
  bool expect_write_replies_ = true;
  /// Last instruction packet sent — send_packet builds the frame directly in
  /// this buffer (capacity reused across calls) and recv_packet uses it to
  /// strip the TX echo that this UART reflects on the shared half-duplex line.
  std::vector<uint8_t> last_tx_;
  /// RX accumulation buffer for recv_packet — reused across calls so the
  /// 50 Hz path performs no per-transaction heap allocation.
  std::vector<uint8_t> rx_buf_;
  /// Scratch buffer for instruction params / status params. Never live across
  /// a nested call that also uses it: write_register finishes with it before
  /// reusing it for the status reply, and read paths pass it only to one
  /// stage at a time. Not thread-safe — neither is the bus itself.
  std::vector<uint8_t> scratch_;
};

}  // namespace jetank_motor_control

#endif  // JETANK_MOTOR_CONTROL__FEETECH_BUS_HPP_
