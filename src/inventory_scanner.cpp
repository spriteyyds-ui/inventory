#include "agv_inventory_system/inventory_scanner.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

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

bool hex_string_to_bytes(const std::string & text, std::vector<unsigned char> & bytes)
{
  std::string compact;
  compact.reserve(text.size());
  for (const unsigned char ch : text) {
    if (std::isxdigit(ch) != 0) {
      compact.push_back(static_cast<char>(std::toupper(ch)));
    } else if (std::isspace(ch) != 0 || ch == ':' || ch == '-') {
      continue;
    } else {
      return false;
    }
  }

  if (compact.empty() || compact.size() % 2U != 0U) {
    return false;
  }

  bytes.clear();
  bytes.reserve(compact.size() / 2U);
  for (std::size_t i = 0; i < compact.size(); i += 2U) {
    const std::string byte_text = compact.substr(i, 2U);
    char * end = nullptr;
    const long value = std::strtol(byte_text.c_str(), &end, 16);
    if (end == nullptr || *end != '\0' || value < 0 || value > 0xff) {
      return false;
    }
    bytes.push_back(static_cast<unsigned char>(value));
  }
  return true;
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
  config_.rfid_frame_length = std::max(1, config_.rfid_frame_length);
  config_.rfid_epc_offset = std::max(0, config_.rfid_epc_offset);
  config_.rfid_epc_length = std::max(0, config_.rfid_epc_length);
}

const InventoryScannerConfig & InventoryScanner::config() const
{
  return config_;
}

bool InventoryScanner::start_grid_scan(int cabinet_id, int row, int level, int column)
{
  reset_active_report_serial_reader();
  clear_scan_output();
  active_report_serial_active_ = false;

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
  std::string error_message;
  if (!config_.rfid_reader_enabled) {
    finish_active_report_serial_scan(false, "rfid_reader_enabled=false");
  } else if (!start_active_report_serial_reader(error_message)) {
    finish_active_report_serial_scan(false, error_message);
  } else {
    active_report_serial_active_ = true;
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
  active_ = false;
  finished_ = false;
  success_ = false;
  active_report_serial_active_ = false;
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
  if (mode == "active_report_serial") {
    return mode;
  }
  return "active_report_serial";
}

bool InventoryScanner::start_active_report_serial_reader(std::string & error_message)
{
  close_active_report_serial_device();
  active_report_serial_buffer_.clear();
  active_report_serial_epcs_.clear();
  active_report_serial_seen_epcs_.clear();

  std::vector<unsigned char> header;
  if (!hex_string_to_bytes(config_.rfid_frame_header, header)) {
    error_message = "invalid rfid_frame_header=" + config_.rfid_frame_header;
    return false;
  }
  if (config_.rfid_frame_length < static_cast<int>(header.size())) {
    error_message = "rfid_frame_length shorter than rfid_frame_header";
    return false;
  }
  if (config_.rfid_epc_length <= 0 ||
    config_.rfid_epc_offset < 0 ||
    config_.rfid_epc_offset + config_.rfid_epc_length > config_.rfid_frame_length)
  {
    error_message = "invalid EPC slice offset=" + std::to_string(config_.rfid_epc_offset) +
      " length=" + std::to_string(config_.rfid_epc_length) +
      " frame_length=" + std::to_string(config_.rfid_frame_length);
    return false;
  }
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
            << " frame_length=" << config_.rfid_frame_length
            << " epc_offset=" << config_.rfid_epc_offset
            << " epc_length=" << config_.rfid_epc_length
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
  std::vector<unsigned char> header;
  if (!hex_string_to_bytes(config_.rfid_frame_header, header) || header.empty()) {
    active_report_serial_buffer_.clear();
    return;
  }

  const std::size_t frame_length = static_cast<std::size_t>(config_.rfid_frame_length);
  const std::size_t epc_offset = static_cast<std::size_t>(config_.rfid_epc_offset);
  const std::size_t epc_length = static_cast<std::size_t>(config_.rfid_epc_length);

  while (active_report_serial_buffer_.size() >= header.size()) {
    auto header_pos = std::search(
      active_report_serial_buffer_.begin(),
      active_report_serial_buffer_.end(),
      header.begin(),
      header.end());

    if (header_pos == active_report_serial_buffer_.end()) {
      const std::size_t keep = std::min(header.size() - 1U, active_report_serial_buffer_.size());
      if (keep > 0U) {
        active_report_serial_buffer_.erase(
          active_report_serial_buffer_.begin(),
          active_report_serial_buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      } else {
        active_report_serial_buffer_.clear();
      }
      return;
    }

    if (header_pos != active_report_serial_buffer_.begin()) {
      active_report_serial_buffer_.erase(active_report_serial_buffer_.begin(), header_pos);
    }

    if (active_report_serial_buffer_.size() < frame_length) {
      return;
    }

    if (epc_offset + epc_length > frame_length) {
      active_report_serial_buffer_.erase(
        active_report_serial_buffer_.begin(),
        active_report_serial_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
      continue;
    }

    const unsigned char * frame = active_report_serial_buffer_.data();
    const std::string epc = bytes_to_hex_string(frame + epc_offset, epc_length);
    const std::size_t tail_offset = frame_length >= 2U ? frame_length - 2U : frame_length;
    const std::size_t tail_length = frame_length - tail_offset;
    const std::string tail =
      tail_length > 0U ? bytes_to_hex_string(frame + tail_offset, tail_length) : "";
    const bool inserted = active_report_serial_seen_epcs_.insert(epc).second;
    if (inserted) {
      active_report_serial_epcs_.push_back(epc);
    }

    std::cout << "[scanner][RFID][active_report_serial] frame cabinet=" << cabinet_id_
              << " level=" << level_
              << " column=" << column_
              << " epc=" << epc
              << " tail_check_not_epc=" << tail
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
    rfids = active_report_serial_epcs_;
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
  active_report_serial_epcs_.clear();
  active_report_serial_seen_epcs_.clear();
}

void InventoryScanner::clear_scan_output()
{
  last_output_ = InventoryScanOutput{};
}

}  // namespace agv_inventory_system
