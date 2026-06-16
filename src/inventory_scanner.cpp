#include "agv_inventory_system/inventory_scanner.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace agv_inventory_system
{
namespace
{

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

std::string bytes_to_hex_string(
  const unsigned char * data,
  std::size_t length)
{
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < length; ++i) {
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  return oss.str();
}

bool serial_device_is_reserved_for_relay(const std::string & device_path)
{
  return device_path == "/dev/ttyCH341USB0" ||
         device_path.find("ttyCH341USB") != std::string::npos;
}

bool baud_to_termios_speed(int baud, speed_t & speed)
{
  switch (baud) {
    case 1200:
      speed = B1200;
      return true;
    case 2400:
      speed = B2400;
      return true;
    case 4800:
      speed = B4800;
      return true;
    case 9600:
      speed = B9600;
      return true;
    case 19200:
      speed = B19200;
      return true;
    case 38400:
      speed = B38400;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
    default:
      return false;
  }
}

}  // namespace

InventoryScanner::~InventoryScanner()
{
  reset();
}

void InventoryScanner::configure(const InventoryScannerConfig & config)
{
  config_ = config;
  config_.scan_duration_sec = std::max(0.0, config_.scan_duration_sec);
  config_.rfid_scan_duration_sec = std::max(0.1, config_.rfid_scan_duration_sec);
  config_.scan_timeout_sec =
    std::max({config_.scan_duration_sec, config_.rfid_scan_duration_sec, config_.scan_timeout_sec});
  config_.scan_retry_count = std::max(0, config_.scan_retry_count);
  config_.scan_result_timeout_sec = std::max(0.0, config_.scan_result_timeout_sec);
  config_.rfid_reader_mode = normalize_reader_mode(config_.rfid_reader_mode);
  config_.rfid_frame_min_length = std::max(8, config_.rfid_frame_min_length);
  config_.rfid_frame_max_length =
    std::max(config_.rfid_frame_min_length, config_.rfid_frame_max_length);
  // UHFReader188 defaults
  config_.uhf_reader_baudrate = std::max(1200, config_.uhf_reader_baudrate);
  config_.uhf_reader_address = std::max(0, std::min(255, config_.uhf_reader_address));
  config_.uhf_reader_q_value = std::max(0, std::min(8, config_.uhf_reader_q_value));
  config_.uhf_reader_session = std::max(0, std::min(3, config_.uhf_reader_session));
  config_.uhf_reader_scan_rounds_per_cell = std::max(1, config_.uhf_reader_scan_rounds_per_cell);
  config_.uhf_reader_scan_interval_sec = std::max(0.0, config_.uhf_reader_scan_interval_sec);
  config_.uhf_reader_frame_timeout_sec = std::max(0.1, config_.uhf_reader_frame_timeout_sec);
  config_.uhf_reader_max_follow_frames = std::max(0, config_.uhf_reader_max_follow_frames);
  if (config_.uhf_reader_power < 0 || config_.uhf_reader_power > 33) {
    std::cerr << "[scanner][RFID][uhf_reader188] WARNING uhf_reader_power="
              << config_.uhf_reader_power << " out of range [0,33], clamping" << std::endl;
    config_.uhf_reader_power = std::max(0, std::min(33, config_.uhf_reader_power));
  }
}

const InventoryScannerConfig & InventoryScanner::config() const
{
  return config_;
}

bool InventoryScanner::start_grid_scan(int cabinet_id, int row, int level, int column)
{
  // Reset both readers
  reset_active_report_serial_reader();
  reset_uhf_reader188();
  clear_scan_output();
  active_report_serial_active_ = false;
  uhf_active_ = false;

  if (!config_.enabled) {
    finished_ = true;
    success_ = true;
    last_result_ = "scanner_disabled";
    last_output_.source = InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_EMPTY;
    last_output_.fallback_to_placeholder = false;
    last_output_.rfids.clear();
    last_output_.message = last_result_;
    std::cout << "[scanner] disabled cabinet=" << cabinet_id
              << " level=" << level
              << " column=" << column << std::endl;
    return true;
  }

  active_ = true;
  finished_ = false;
  success_ = false;
  cabinet_id_ = cabinet_id;
  row_ = row;
  level_ = level;
  column_ = column;
  last_result_.clear();
  start_time_ = Clock::now();

  std::cout << "[scanner] scan started cabinet=" << cabinet_id_
            << " row=" << row_
            << " level=" << level_
            << " column=" << column_
            << " reader_mode=" << config_.rfid_reader_mode << std::endl;

  // Route to the correct reader mode
  if (is_uhf_reader188_mode(config_.rfid_reader_mode)) {
    // UHFReader188 answer mode: open port and start synchronous cell scan in a thread
    if (!config_.rfid_reader_enabled) {
      finish_uhf_reader188_scan(false, "rfid_reader_enabled=false");
    } else {
      uhf_active_ = true;
    }
  } else {
    // Active report serial mode (old reader)
    std::string error_message;
    if (!config_.rfid_reader_enabled) {
      finish_active_report_serial_scan(false, "rfid_reader_enabled=false");
    } else if (!start_active_report_serial_reader(error_message)) {
      finish_active_report_serial_scan(false, error_message);
    } else {
      active_report_serial_active_ = true;
    }
  }
  return true;
}

void InventoryScanner::update()
{
  if (!active_ || finished_) {
    return;
  }

  const auto now = Clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - start_time_).count();

  if (uhf_active_) {
    // UHFReader188 mode: run the full cell scan synchronously on first update
    uhf_active_ = false;
    uhf_run_cell_scan();
    return;
  }

  if (active_report_serial_active_) {
    std::string error_message;
    if (!poll_active_report_serial_once(error_message)) {
      finish_active_report_serial_scan(false, error_message);
      return;
    }

    if (elapsed >= config_.rfid_scan_duration_sec) {
      finish_active_report_serial_scan(true, "scan_window_complete");
      return;
    }
    return;
  }

  if (elapsed > config_.scan_timeout_sec) {
    finish_active_report_serial_scan(false, "scan_timeout");
    return;
  }
}

bool InventoryScanner::is_scan_finished() const
{
  return finished_;
}

bool InventoryScanner::scan_success() const
{
  return success_;
}

const std::string & InventoryScanner::last_scan_result() const
{
  return last_result_;
}

const InventoryScanOutput & InventoryScanner::last_scan_output() const
{
  return last_output_;
}

void InventoryScanner::reset()
{
  reset_active_report_serial_reader();
  reset_uhf_reader188();
  active_ = false;
  finished_ = false;
  success_ = false;
  active_report_serial_active_ = false;
  uhf_active_ = false;
  cabinet_id_ = 0;
  row_ = 0;
  level_ = 0;
  column_ = 0;
  last_result_.clear();
  start_time_ = Clock::time_point{};
  clear_scan_output();
}

std::string InventoryScanner::normalize_reader_mode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode == "uhf_reader188_answer_serial" || mode == "active_report_serial") {
    return mode;
  }
  // Default to uhf_reader188_answer_serial
  return "uhf_reader188_answer_serial";
}

bool InventoryScanner::is_uhf_reader188_mode(const std::string & mode)
{
  return mode == "uhf_reader188_answer_serial";
}

// =========================================================================
// UHFReader188 answer mode implementation
// =========================================================================

unsigned short InventoryScanner::uhf_crc16(const unsigned char * data, std::size_t length)
{
  unsigned short crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<unsigned short>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0x8408;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

std::vector<unsigned char> InventoryScanner::uhf_build_cmd(
  int addr, int cmd, const std::vector<unsigned char> & data)
{
  int length = 4 + static_cast<int>(data.size());
  std::vector<unsigned char> frame;
  frame.reserve(length + 2);
  frame.push_back(static_cast<unsigned char>(length & 0xFF));
  frame.push_back(static_cast<unsigned char>(addr & 0xFF));
  frame.push_back(static_cast<unsigned char>(cmd & 0xFF));
  frame.insert(frame.end(), data.begin(), data.end());
  unsigned short crc = uhf_crc16(frame.data(), frame.size());
  frame.push_back(static_cast<unsigned char>(crc & 0xFF));        // CRC low byte
  frame.push_back(static_cast<unsigned char>((crc >> 8) & 0xFF)); // CRC high byte
  return frame;
}

std::vector<unsigned char> InventoryScanner::uhf_build_inventory_short(
  int addr, int q_value, int session)
{
  std::vector<unsigned char> data = {
    static_cast<unsigned char>(q_value & 0x07),
    static_cast<unsigned char>(session & 0x03)
  };
  return uhf_build_cmd(addr, 0x01, data);
}

std::vector<unsigned char> InventoryScanner::uhf_build_inventory_single(int addr)
{
  return uhf_build_cmd(addr, 0x0F, {});
}

std::vector<unsigned char> InventoryScanner::uhf_build_reader_info_cmd(int addr)
{
  return uhf_build_cmd(addr, 0x21, {});
}

std::vector<unsigned char> InventoryScanner::uhf_build_set_power_cmd(int addr, int power)
{
  std::vector<unsigned char> data = {static_cast<unsigned char>(power & 0xFF)};
  return uhf_build_cmd(addr, 0x2F, data);
}

bool InventoryScanner::uhf_apply_power_setting()
{
  if (!config_.uhf_reader_set_power_on_open) {
    return true;
  }

  int power = config_.uhf_reader_power;
  auto cmd = uhf_build_set_power_cmd(config_.uhf_reader_address, power);

  if (!uhf_reader188_send_cmd(cmd)) {
    std::cerr << "[scanner][RFID][uhf_reader188] set power failed: send_cmd_failed" << std::endl;
    return false;
  }

  std::vector<unsigned char> frame;
  if (!uhf_reader188_read_frame(frame)) {
    std::cerr << "[scanner][RFID][uhf_reader188] set power failed: read_frame_failed" << std::endl;
    return false;
  }

  if (frame.size() < 5) {
    std::cerr << "[scanner][RFID][uhf_reader188] set power failed: short_frame" << std::endl;
    return false;
  }

  // frame[2]=reCmd, frame[3]=Status
  unsigned char re_cmd = frame[2];
  unsigned char status = frame[3];

  if (re_cmd != 0x2F) {
    std::cerr << "[scanner][RFID][uhf_reader188] set power failed: unexpected reCmd=0x"
              << std::hex << static_cast<int>(re_cmd) << std::dec << std::endl;
    return false;
  }

  if (status != 0x00) {
    std::cerr << "[scanner][RFID][uhf_reader188] set power failed status=0x"
              << std::hex << static_cast<int>(status) << std::dec << std::endl;
    return false;
  }

  std::cout << "[scanner][RFID][uhf_reader188] set power ok power=" << power << std::endl;
  return true;
}

bool InventoryScanner::start_uhf_reader188(std::string & error_message)
{
  close_uhf_reader188_device();
  uhf_rfids_.clear();
  uhf_seen_rfids_.clear();
  uhf_error_log_.clear();

  if (config_.uhf_reader_serial_port.empty()) {
    error_message = "uhf_reader_serial_port is empty";
    return false;
  }
  if (serial_device_is_reserved_for_relay(config_.uhf_reader_serial_port)) {
    error_message =
      "refuse to open relay serial device as UHFReader188 path=" +
      config_.uhf_reader_serial_port;
    return false;
  }

  uhf_fd_ = ::open(
    config_.uhf_reader_serial_port.c_str(),
    O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (uhf_fd_ < 0) {
    error_message = errno_message(
      "open UHFReader188 serial failed path=" + config_.uhf_reader_serial_port);
    if (errno == EACCES || errno == EPERM) {
      error_message += " (permission denied; check dialout group or udev permissions)";
    }
    return false;
  }

  if (!configure_uhf_reader188_port(error_message)) {
    close_uhf_reader188_device();
    return false;
  }

  if (::tcflush(uhf_fd_, TCIOFLUSH) != 0) {
    error_message = errno_message(
      "tcflush UHFReader188 serial failed path=" + config_.uhf_reader_serial_port);
    close_uhf_reader188_device();
    return false;
  }

  error_message.clear();
  std::cout << "[scanner][RFID][uhf_reader188] opened path="
            << config_.uhf_reader_serial_port
            << " baud=" << config_.uhf_reader_baudrate
            << " addr=0x" << std::hex << config_.uhf_reader_address << std::dec
            << " q=" << config_.uhf_reader_q_value
            << " session=S" << config_.uhf_reader_session
            << " rounds=" << config_.uhf_reader_scan_rounds_per_cell
            << " interval=" << config_.uhf_reader_scan_interval_sec
            << " set_power_on_open=" << (config_.uhf_reader_set_power_on_open ? "true" : "false")
            << " power=" << config_.uhf_reader_power
            << std::endl;
  return true;
}

bool InventoryScanner::configure_uhf_reader188_port(std::string & error_message)
{
  if (uhf_fd_ < 0) {
    error_message = "UHFReader188 serial device is not open";
    return false;
  }

  speed_t speed;
  if (!baud_to_termios_speed(config_.uhf_reader_baudrate, speed)) {
    error_message = "unsupported uhf_reader_baudrate=" +
      std::to_string(config_.uhf_reader_baudrate);
    return false;
  }

  termios tty;
  if (::tcgetattr(uhf_fd_, &tty) != 0) {
    error_message = errno_message(
      "tcgetattr UHFReader188 failed path=" + config_.uhf_reader_serial_port);
    return false;
  }

  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    error_message = errno_message(
      "set UHFReader188 baud failed path=" + config_.uhf_reader_serial_port);
    return false;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD;
  tty.c_cflag |= CLOCAL;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  // Non-blocking read: VMIN=0, VTIME=0 for poll-based reading
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(uhf_fd_, TCSANOW, &tty) != 0) {
    error_message = errno_message(
      "tcsetattr UHFReader188 failed path=" + config_.uhf_reader_serial_port);
    return false;
  }

  return true;
}

bool InventoryScanner::uhf_reader188_send_cmd(const std::vector<unsigned char> & cmd)
{
  if (uhf_fd_ < 0) {return false;}
  ssize_t written = ::write(uhf_fd_, cmd.data(), cmd.size());
  if (written < 0 || static_cast<std::size_t>(written) != cmd.size()) {
    return false;
  }
  if (config_.uhf_reader_debug_hex_log) {
    std::cout << "[scanner][RFID][uhf_reader188] TX: "
              << bytes_to_hex_string(cmd.data(), cmd.size()) << std::endl;
  }
  return true;
}

bool InventoryScanner::uhf_reader188_read_frame(std::vector<unsigned char> & frame)
{
  return uhf_reader188_read_frame_with_timeout(frame, config_.uhf_reader_frame_timeout_sec);
}

bool InventoryScanner::uhf_reader188_read_frame_with_timeout(
  std::vector<unsigned char> & frame, double timeout_sec)
{
  frame.clear();
  if (uhf_fd_ < 0) {return false;}

  // Read Len byte with timeout using poll
  auto deadline = Clock::now() + std::chrono::milliseconds(
    static_cast<int>(timeout_sec * 1000.0));

  // Poll for Len byte
  while (true) {
    auto now = Clock::now();
    if (now >= deadline) {
      return false;  // timeout
    }
    double remaining_ms = std::chrono::duration<double>(deadline - now).count() * 1000.0;
    pollfd pfd;
    pfd.fd = uhf_fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int poll_result = ::poll(&pfd, 1, static_cast<int>(remaining_ms));
    if (poll_result < 0) {
      if (errno == EINTR) {continue;}
      return false;
    }
    if (poll_result == 0) {continue;}  // timeout, keep polling
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      return false;
    }
    break;
  }

  // Read Len byte
  unsigned char len_byte;
  while (true) {
    ssize_t n = ::read(uhf_fd_, &len_byte, 1);
    if (n == 1) {break;}
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // Check timeout
      if (Clock::now() >= deadline) {return false;}
      // Small sleep and retry
      struct timespec ts = {0, 1000000};  // 1ms
      ::nanosleep(&ts, nullptr);
      continue;
    }
    return false;
  }

  frame.push_back(len_byte);
  int remaining = len_byte;
  if (remaining < 3) {return false;}  // minimum: addr + cmd + crc_lo + crc_h but len includes 4 bytes

  // Read remaining bytes
  int total_to_read = remaining;
  std::vector<unsigned char> buf(total_to_read);
  int total_read = 0;
  while (total_read < total_to_read) {
    if (Clock::now() >= deadline) {
      return false;
    }
    ssize_t n = ::read(uhf_fd_, buf.data() + total_read, total_to_read - total_read);
    if (n > 0) {
      total_read += static_cast<int>(n);
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      struct timespec ts = {0, 1000000};  // 1ms
      ::nanosleep(&ts, nullptr);
      continue;
    } else {
      return false;
    }
  }

  frame.insert(frame.end(), buf.begin(), buf.begin() + total_to_read);

  if (config_.uhf_reader_debug_hex_log) {
    std::cout << "[scanner][RFID][uhf_reader188] RX: "
              << bytes_to_hex_string(frame.data(), frame.size()) << std::endl;
  }

  // Validate frame length
  if (frame.size() < 5) {return false;}

  // Validate CRC
  unsigned short calc_crc = uhf_crc16(frame.data(), frame.size() - 2);
  unsigned short recv_crc = static_cast<unsigned short>(frame[frame.size() - 2]) |
    (static_cast<unsigned short>(frame[frame.size() - 1]) << 8);
  if (calc_crc != recv_crc) {
    if (config_.uhf_reader_debug_hex_log) {
      std::cerr << "[scanner][RFID][uhf_reader188] CRC mismatch: calc=0x"
                << std::hex << calc_crc << " recv=0x" << recv_crc << std::dec << std::endl;
    }
    return false;
  }

  return true;
}

bool InventoryScanner::uhf_parse_tags(
  const unsigned char * data, std::size_t length,
  std::set<std::string> & seen, std::vector<std::string> & rfids)
{
  if (length < 1) {return false;}

  std::size_t offset = 0;
  int num_tags = data[offset];
  offset += 1;

  for (int t = 0; t < num_tags && offset < length; ++t) {
    // Auto-detect format: standard (EPC_Len + EPC + RSSI) or variant (Ant + EPC_Len + EPC + RSSI)
    unsigned char candidate = data[offset];
    std::size_t after_epc_len = offset + 1 + candidate + 1;  // EPC_Len + EPC + RSSI
    int antenna = 0;

    if (candidate >= 4 && candidate <= 32 && candidate % 2 == 0 && after_epc_len <= length) {
      // Standard format
      offset += 1;  // skip EPC_Len
    } else if (offset + 1 < length) {
      // Variant: Ant + EPC_Len + EPC + RSSI
      antenna = candidate;
      offset += 1;
      if (offset >= length) {break;}
      candidate = data[offset];
      if (candidate < 4 || candidate > 32 || candidate % 2 != 0) {break;}
      after_epc_len = offset + 1 + candidate + 1;
      if (after_epc_len > length) {break;}
      offset += 1;  // skip EPC_Len
    } else {
      break;
    }

    int epc_len = candidate;
    if (offset + epc_len > length) {break;}

    std::string epc = bytes_to_hex_string(data + offset, epc_len);
    offset += epc_len;

    int rssi = 0;
    if (offset < length) {
      rssi = data[offset];
      offset += 1;
    }

    (void)antenna;
    (void)rssi;

    if (seen.find(epc) == seen.end()) {
      seen.insert(epc);
      rfids.push_back(epc);
    }
  }

  return true;
}

void InventoryScanner::uhf_collect_inventory_round()
{
  // Build and send 0x01 short format command
  auto cmd = uhf_build_inventory_short(
    config_.uhf_reader_address,
    config_.uhf_reader_q_value,
    config_.uhf_reader_session);

  if (!uhf_reader188_send_cmd(cmd)) {
    uhf_error_log_.push_back("send_cmd_failed");
    return;
  }

  // Read first response frame
  std::vector<unsigned char> frame;
  if (!uhf_reader188_read_frame(frame)) {
    uhf_error_log_.push_back("read_frame_failed");
    return;
  }

  if (frame.size() < 5) {
    uhf_error_log_.push_back("short_frame");
    return;
  }

  // Parse: frame[0]=Len, frame[1]=Addr, frame[2]=reCmd, frame[3]=Status, frame[4..]=Data+CRC
  unsigned char status = frame[3];
  const unsigned char * data = frame.data() + 4;
  // Frame: Len Addr reCmd Status Data CRC_L CRC_H
  // Len includes Addr+reCmd+Status+Data+CRC = 3+len(Data)+2 = 5+len(Data)
  // payload_len = frame.size()-1(Len)-4(Addr,Cmd,Status,CRC)= frame.size()-6
  std::size_t payload_len = frame.size() > 6 ? frame.size() - 6 : 0;

  if (status == 0x03 || status == 0x01 || status == 0x02 || status == 0x00 || status == 0x04) {
    uhf_parse_tags(data, payload_len, uhf_seen_rfids_, uhf_rfids_);
  }

  // Collect continuation frames if status=0x03
  int follow_count = 0;
  while (status == 0x03 && config_.uhf_reader_collect_follow_frames &&
    follow_count < config_.uhf_reader_max_follow_frames)
  {
    std::vector<unsigned char> follow_frame;
    if (!uhf_reader188_read_frame(follow_frame)) {
      uhf_error_log_.push_back("follow_frame_read_failed");
      break;
    }
    if (follow_frame.size() < 5) {break;}
    status = follow_frame[3];
    const unsigned char * fdata = follow_frame.data() + 4;
    std::size_t fdata_len = 0;
    if (follow_frame.size() > 6) {
      fdata_len = follow_frame.size() - 6;
    }

    if (status == 0x03 || status == 0x01 || status == 0x02 || status == 0x00 || status == 0x04) {
      uhf_parse_tags(fdata, fdata_len, uhf_seen_rfids_, uhf_rfids_);
    }
    follow_count++;
  }
}

void InventoryScanner::uhf_run_cell_scan()
{
  std::string error_message;
  if (!start_uhf_reader188(error_message)) {
    finish_uhf_reader188_scan(false, error_message);
    return;
  }

  // Set power on open (non-fatal: warn and continue on failure)
  if (config_.uhf_reader_set_power_on_open) {
    uhf_apply_power_setting();
  }

  // Perform multiple rounds of inventory
  for (int i = 0; i < config_.uhf_reader_scan_rounds_per_cell; ++i) {
    uhf_collect_inventory_round();

    // Sleep between rounds (except after the last round)
    if (i < config_.uhf_reader_scan_rounds_per_cell - 1) {
      ::usleep(static_cast<useconds_t>(
        config_.uhf_reader_scan_interval_sec * 1000000.0));
    }
  }

  // Fallback: if no tags found and fallback_single_cmd is enabled
  if (uhf_rfids_.empty() && config_.uhf_reader_fallback_single_cmd &&
    config_.uhf_reader_single_cmd_enabled)
  {
    std::cout << "[scanner][RFID][uhf_reader188] no tags from 0x01, "
              << "trying 0x0F single-tag fallback" << std::endl;
    auto cmd = uhf_build_inventory_single(config_.uhf_reader_address);
    if (uhf_reader188_send_cmd(cmd)) {
      std::vector<unsigned char> frame;
      if (uhf_reader188_read_frame(frame) && frame.size() >= 5) {
        unsigned char status = frame[3];
        if (status == 0x01 || status == 0x02 || status == 0x03) {
          const unsigned char * data = frame.data() + 4;
          std::size_t data_len = frame.size() > 6 ? frame.size() - 6 : 0;
          uhf_parse_tags(data, data_len, uhf_seen_rfids_, uhf_rfids_);
        }
      }
    }
  }

  // Check for hardware errors
  bool reader_ok = true;
  std::string reason = "cell_scan_complete";
  for (const auto & err : uhf_error_log_) {
    if (err == "send_cmd_failed" || err == "read_frame_failed") {
      // If we got some tags, still report success
      if (uhf_rfids_.empty()) {
        reader_ok = false;
        reason = err;
        break;
      }
    }
  }

  close_uhf_reader188_device();
  finish_uhf_reader188_scan(reader_ok, reason);
}

void InventoryScanner::finish_uhf_reader188_scan(
  bool reader_ok, const std::string & reason)
{
  close_uhf_reader188_device();
  uhf_active_ = false;
  active_ = false;
  finished_ = true;
  success_ = true;

  std::vector<std::string> rfids;
  if (reader_ok) {
    rfids = uhf_rfids_;
  }

  std::ostringstream result;
  result << "uhf_reader188_"
         << (reader_ok ? (rfids.empty() ? "empty" : "ok") : "failed")
         << ":cabinet=" << cabinet_id_
         << ",row=" << row_
         << ",level=" << level_
         << ",column=" << column_
         << ",rfid_count=" << rfids.size()
         << ",reason=" << reason;
  last_result_ = result.str();
  last_output_.source = reader_ok ?
    (rfids.empty() ?
      InventoryScanOutputSource::UHF_READER188_EMPTY :
      InventoryScanOutputSource::UHF_READER188_SUCCESS) :
    InventoryScanOutputSource::UHF_READER188_FAILED;
  last_output_.fallback_to_placeholder = false;
  last_output_.rfids = rfids;
  last_output_.message = reason;

  const bool has_epc = !rfids.empty();
  std::ostream & stream = (reader_ok && has_epc) ? std::cout : std::cerr;
  stream << "[scanner][RFID][uhf_reader188] "
         << ((reader_ok && has_epc) ? "scan finished" : "WARNING scan no_epc_or_failed")
         << " cabinet=" << cabinet_id_
         << " level=" << level_
         << " column=" << column_
         << " rfid_count=" << rfids.size()
         << " reason=\"" << reason << "\""
         << " fallback_to_placeholder=false" << std::endl;

  if (!uhf_error_log_.empty()) {
    std::cout << "[scanner][RFID][uhf_reader188] error_log=";
    for (std::size_t i = 0; i < uhf_error_log_.size(); ++i) {
      if (i > 0) {std::cout << ",";}
      std::cout << uhf_error_log_[i];
    }
    std::cout << std::endl;
  }
}

void InventoryScanner::close_uhf_reader188_device()
{
  if (uhf_fd_ >= 0) {
    ::close(uhf_fd_);
    uhf_fd_ = -1;
  }
}

void InventoryScanner::reset_uhf_reader188()
{
  close_uhf_reader188_device();
  uhf_active_ = false;
  uhf_rfids_.clear();
  uhf_seen_rfids_.clear();
  uhf_error_log_.clear();
}

// =========================================================================
// Active report serial mode implementation (unchanged from original)
// =========================================================================

bool InventoryScanner::start_active_report_serial_reader(std::string & error_message)
{
  close_active_report_serial_device();
  active_report_serial_buffer_.clear();
  active_report_serial_rfids_.clear();
  active_report_serial_seen_rfids_.clear();
  if (config_.rfid_serial_device.empty()) {
    error_message = "rfid_serial_device is empty";
    return false;
  }
  if (serial_device_is_reserved_for_relay(config_.rfid_serial_device)) {
    error_message =
      "refuse to open relay serial device as RFID reader path=" + config_.rfid_serial_device;
    return false;
  }

  active_report_serial_fd_ =
    ::open(config_.rfid_serial_device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (active_report_serial_fd_ < 0) {
    error_message = errno_message("open RFID serial device failed path=" + config_.rfid_serial_device);
    if (errno == EACCES || errno == EPERM) {
      error_message += " (permission denied; check dialout group or udev permissions)";
    }
    return false;
  }

  if (!configure_active_report_serial_port(error_message)) {
    close_active_report_serial_device();
    return false;
  }

  if (::tcflush(active_report_serial_fd_, TCIFLUSH) != 0) {
    error_message = errno_message("tcflush RFID serial input failed path=" + config_.rfid_serial_device);
    close_active_report_serial_device();
    return false;
  }

  error_message.clear();
  std::cout << "[scanner][RFID][active_report_serial] opened path="
            << config_.rfid_serial_device
            << " baud=" << config_.rfid_serial_baud
            << " frame_id=00EE00"
            << " frame_min_length=" << config_.rfid_frame_min_length
            << " frame_max_length=" << config_.rfid_frame_max_length
            << " duration_sec=" << config_.rfid_scan_duration_sec << std::endl;
  return true;
}

bool InventoryScanner::configure_active_report_serial_port(std::string & error_message)
{
  if (active_report_serial_fd_ < 0) {
    error_message = "RFID serial device is not open";
    return false;
  }

  speed_t speed;
  if (!baud_to_termios_speed(config_.rfid_serial_baud, speed)) {
    error_message = "unsupported rfid_serial_baud=" + std::to_string(config_.rfid_serial_baud);
    return false;
  }

  termios tty;
  if (::tcgetattr(active_report_serial_fd_, &tty) != 0) {
    error_message =
      errno_message("tcgetattr RFID serial failed path=" + config_.rfid_serial_device);
    return false;
  }

  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    error_message =
      errno_message("set RFID serial baud failed path=" + config_.rfid_serial_device);
    return false;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD;
  tty.c_cflag |= CLOCAL;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(active_report_serial_fd_, TCSANOW, &tty) != 0) {
    error_message =
      errno_message("tcsetattr RFID serial failed path=" + config_.rfid_serial_device);
    return false;
  }

  return true;
}

bool InventoryScanner::poll_active_report_serial_once(std::string & error_message)
{
  error_message.clear();
  if (active_report_serial_fd_ < 0) {
    error_message = "RFID serial device is not open";
    return false;
  }

  pollfd pfd;
  pfd.fd = active_report_serial_fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;
  const int poll_result = ::poll(&pfd, 1, 0);
  if (poll_result < 0) {
    if (errno == EINTR) {
      return true;
    }
    error_message = errno_message("poll RFID serial device failed");
    return false;
  }
  if (poll_result == 0) {
    return true;
  }
  if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    error_message = "RFID serial device poll error revents=" + std::to_string(pfd.revents);
    return false;
  }

  unsigned char buffer[128];
  while (true) {
    const ssize_t bytes_read = ::read(active_report_serial_fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
      active_report_serial_buffer_.insert(
        active_report_serial_buffer_.end(),
        buffer,
        buffer + bytes_read);
      parse_active_report_serial_buffer();
      continue;
    }
    if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    }
    if (bytes_read < 0 && errno == EINTR) {
      continue;
    }
    if (bytes_read == 0) {
      break;
    }

    error_message = errno_message("read RFID serial device failed");
    return false;
  }

  return true;
}

void InventoryScanner::parse_active_report_serial_buffer()
{
  constexpr unsigned char kFrameId0 = 0x00;
  constexpr unsigned char kFrameId1 = 0xEE;
  constexpr unsigned char kFrameId2 = 0x00;
  const std::size_t min_frame_length = static_cast<std::size_t>(config_.rfid_frame_min_length);
  const std::size_t max_frame_length = static_cast<std::size_t>(config_.rfid_frame_max_length);

  while (active_report_serial_buffer_.size() >= 4U) {
    std::size_t candidate_pos = active_report_serial_buffer_.size();
    for (std::size_t i = 0; i + 3U < active_report_serial_buffer_.size(); ++i) {
      if (active_report_serial_buffer_[i + 1U] == kFrameId0 &&
        active_report_serial_buffer_[i + 2U] == kFrameId1 &&
        active_report_serial_buffer_[i + 3U] == kFrameId2)
      {
        candidate_pos = i;
        break;
      }
    }

    if (candidate_pos == active_report_serial_buffer_.size()) {
      const std::size_t keep = std::min<std::size_t>(3U, active_report_serial_buffer_.size());
      if (active_report_serial_buffer_.size() > keep) {
        active_report_serial_buffer_.erase(
          active_report_serial_buffer_.begin(),
          active_report_serial_buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      }
      return;
    }

    if (candidate_pos > 0U) {
      active_report_serial_buffer_.erase(
        active_report_serial_buffer_.begin(),
        active_report_serial_buffer_.begin() + static_cast<std::ptrdiff_t>(candidate_pos));
    }

    if (active_report_serial_buffer_.size() < 4U) {
      return;
    }

    const std::size_t frame_length =
      static_cast<std::size_t>(active_report_serial_buffer_[0]) + 1U;
    if (frame_length < min_frame_length || frame_length > max_frame_length) {
      std::cout << "[scanner][RFID][active_report_serial] WARNING invalid frame_length="
                << frame_length
                << " min=" << min_frame_length
                << " max=" << max_frame_length
                << " drop_len_byte="
                << bytes_to_hex_string(active_report_serial_buffer_.data(), 1U) << std::endl;
      active_report_serial_buffer_.erase(active_report_serial_buffer_.begin());
      continue;
    }

    if (active_report_serial_buffer_.size() < frame_length) {
      return;
    }

    const std::size_t rfid_length = frame_length - 6U;
    if (rfid_length == 0U) {
      std::cout << "[scanner][RFID][active_report_serial] WARNING empty RFID payload"
                << " frame_length=" << frame_length << std::endl;
      active_report_serial_buffer_.erase(
        active_report_serial_buffer_.begin(),
        active_report_serial_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
      continue;
    }

    const unsigned char * frame = active_report_serial_buffer_.data();
    const std::string rfid = bytes_to_hex_string(frame + 4U, rfid_length);
    const std::string tail = bytes_to_hex_string(frame + frame_length - 2U, 2U);
    const bool inserted = active_report_serial_seen_rfids_.insert(rfid).second;
    if (inserted) {
      active_report_serial_rfids_.push_back(rfid);
    }

    std::cout << "[scanner][RFID][active_report_serial] frame cabinet=" << cabinet_id_
              << " level=" << level_
              << " column=" << column_
              << " frame_len=" << frame_length
              << " rfid=" << rfid
              << " tail_check_not_rfid=" << tail
              << " duplicate=" << (inserted ? "false" : "true") << std::endl;

    active_report_serial_buffer_.erase(
      active_report_serial_buffer_.begin(),
      active_report_serial_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
  }
}

void InventoryScanner::finish_active_report_serial_scan(
  bool reader_ok,
  const std::string & reason)
{
  close_active_report_serial_device();
  active_report_serial_active_ = false;
  active_ = false;
  finished_ = true;
  success_ = true;

  std::vector<std::string> rfids;
  if (reader_ok) {
    rfids = active_report_serial_rfids_;
  }

  std::ostringstream result;
  result << "active_report_serial_"
         << (reader_ok ? (rfids.empty() ? "empty" : "ok") : "failed")
         << ":cabinet=" << cabinet_id_
         << ",row=" << row_
         << ",level=" << level_
         << ",column=" << column_
         << ",rfid_count=" << rfids.size()
         << ",reason=" << reason;
  last_result_ = result.str();
  last_output_.source = reader_ok ?
    (rfids.empty() ?
      InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_EMPTY :
      InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_SUCCESS) :
    InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_FAILED;
  last_output_.fallback_to_placeholder = false;
  last_output_.rfids = rfids;
  last_output_.message = reason;

  const bool has_epc = !rfids.empty();
  std::ostream & stream = (reader_ok && has_epc) ? std::cout : std::cerr;
  stream << "[scanner][RFID][active_report_serial] "
         << ((reader_ok && has_epc) ? "scan finished" : "WARNING scan no_epc_or_failed")
         << " cabinet=" << cabinet_id_
         << " level=" << level_
         << " column=" << column_
         << " rfid_count=" << rfids.size()
         << " reason=\"" << reason << "\""
         << " fallback_to_placeholder=false" << std::endl;
}

void InventoryScanner::close_active_report_serial_device()
{
  if (active_report_serial_fd_ >= 0) {
    ::close(active_report_serial_fd_);
    active_report_serial_fd_ = -1;
  }
}

void InventoryScanner::reset_active_report_serial_reader()
{
  close_active_report_serial_device();
  active_report_serial_active_ = false;
  active_report_serial_buffer_.clear();
  active_report_serial_rfids_.clear();
  active_report_serial_seen_rfids_.clear();
}

void InventoryScanner::clear_scan_output()
{
  last_output_ = InventoryScanOutput{};
}

}  // namespace agv_inventory_system