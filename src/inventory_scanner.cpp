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
