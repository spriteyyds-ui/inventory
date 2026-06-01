#include "agv_inventory_system/hid_scanner_reader.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace agv_inventory_system
{
namespace
{

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

bool is_shift_key(unsigned short code)
{
  return code == KEY_LEFTSHIFT || code == KEY_RIGHTSHIFT;
}

std::string trim(const std::string & value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  if (first == value.end()) {
    return "";
  }
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  return std::string(first, last);
}

std::string unquote_input_value(std::string value)
{
  value = trim(value);
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    return value.substr(1U, value.size() - 2U);
  }
  return value;
}

bool contains_any_case_insensitive(const std::string & value, const std::vector<std::string> & needles)
{
  std::string lower_value = value;
  std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  for (auto needle : needles) {
    std::transform(needle.begin(), needle.end(), needle.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    if (lower_value.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string find_auto_hid_device_path()
{
  std::ifstream input("/proc/bus/input/devices");
  if (!input) {
    return "";
  }

  std::string line;
  std::string name;
  std::string handlers;
  std::string vendor;
  std::string product;
  auto flush_block = [&]() -> std::string {
    const bool looks_like_scanner =
      contains_any_case_insensitive(name, {"STM32", "Custm HID", "scanner"}) ||
      (vendor == "ffff" && product == "0035");
    if (!looks_like_scanner) {
      return "";
    }
    std::istringstream stream(handlers);
    std::string handler;
    while (stream >> handler) {
      if (handler.rfind("event", 0) == 0) {
        return "/dev/input/" + handler;
      }
    }
    return "";
  };

  while (std::getline(input, line)) {
    if (line.empty()) {
      const std::string path = flush_block();
      if (!path.empty()) {
        return path;
      }
      name.clear();
      handlers.clear();
      vendor.clear();
      product.clear();
      continue;
    }
    if (line.rfind("N: Name=", 0) == 0) {
      name = unquote_input_value(line.substr(std::string("N: Name=").size()));
    } else if (line.rfind("H: Handlers=", 0) == 0) {
      handlers = line.substr(std::string("H: Handlers=").size());
    } else if (line.rfind("I:", 0) == 0) {
      std::istringstream stream(line);
      std::string token;
      while (stream >> token) {
        if (token.rfind("Vendor=", 0) == 0) {
          vendor = token.substr(std::string("Vendor=").size());
        } else if (token.rfind("Product=", 0) == 0) {
          product = token.substr(std::string("Product=").size());
        }
      }
    }
  }
  return flush_block();
}

}  // namespace

HidScannerReader::~HidScannerReader()
{
  reset();
}

bool HidScannerReader::start(const HidScannerReaderConfig & config, std::string & error_message)
{
  reset();
  config_ = config;
  config_.inter_char_timeout_ms = std::max(1, config_.inter_char_timeout_ms);
  config_.max_tags_per_location = std::max(0, config_.max_tags_per_location);

  std::string device_path = config_.device_path;
  if (device_path.empty() || device_path == "auto") {
    device_path = find_auto_hid_device_path();
    if (device_path.empty()) {
      error_message =
        "rfid_hid_device_path is empty/auto and no STM32 Custm HID scanner was found";
      return false;
    }
    config_.device_path = device_path;
  }

  fd_ = ::open(device_path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error_message = errno_message("open HID input device failed path=" + device_path);
    if (errno == EACCES || errno == EPERM) {
      error_message += " (permission denied; add user to input group or configure udev rule)";
    }
    return false;
  }

  if (config_.grab_device) {
    if (::ioctl(fd_, EVIOCGRAB, 1) == 0) {
      grabbed_ = true;
    } else {
      error_message = errno_message("EVIOCGRAB failed path=" + device_path);
      close_device();
      return false;
    }
  }

  drain_input_events();
  error_message.clear();
  return true;
}

bool HidScannerReader::poll_once(int timeout_ms, std::string & error_message)
{
  error_message.clear();
  if (fd_ < 0) {
    error_message = "HID input device is not open";
    return false;
  }

  maybe_finish_pending_tag_by_timeout();
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const int bounded_timeout = std::clamp(timeout_ms, 0, 20);
  const int poll_result = ::poll(&pfd, 1, bounded_timeout);
  if (poll_result < 0) {
    if (errno == EINTR) {
      return true;
    }
    error_message = errno_message("poll HID input device failed");
    return false;
  }

  if (poll_result == 0) {
    maybe_finish_pending_tag_by_timeout();
    return true;
  }

  if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    error_message = "HID input device poll error";
    return false;
  }

  while (true) {
    input_event event;
    const ssize_t bytes_read = ::read(fd_, &event, sizeof(event));
    if (bytes_read == static_cast<ssize_t>(sizeof(event))) {
      if (event.type == EV_KEY && event.value == 1) {
        (void)process_key_event(static_cast<unsigned short>(event.code));
      }
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

    error_message = errno_message("read HID input event failed");
    return false;
  }

  maybe_finish_pending_tag_by_timeout();
  return true;
}

void HidScannerReader::finish_pending_tag()
{
  append_current_tag_if_valid();
}

bool HidScannerReader::ready_after_quiet() const
{
  if (tags_.empty() && current_tag_.empty()) {
    return false;
  }
  const auto reference_time =
    current_tag_.empty() ? last_activity_time_ : last_key_time_;
  if (reference_time == Clock::time_point{}) {
    return false;
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now() - reference_time);
  return elapsed.count() >= config_.inter_char_timeout_ms;
}

const std::vector<std::string> & HidScannerReader::tags() const
{
  return tags_;
}

void HidScannerReader::reset()
{
  close_device();
  shift_for_next_key_ = false;
  current_tag_.clear();
  tags_.clear();
  seen_tags_.clear();
  last_key_time_ = Clock::time_point{};
  last_activity_time_ = Clock::time_point{};
}

bool HidScannerReader::process_key_event(unsigned short code)
{
  if (is_shift_key(code)) {
    shift_for_next_key_ = true;
    return true;
  }

  if (code == KEY_ENTER || code == KEY_KPENTER) {
    append_current_tag_if_valid();
    shift_for_next_key_ = false;
    last_activity_time_ = Clock::now();
    return true;
  }

  if (code == KEY_BACKSPACE) {
    if (!current_tag_.empty()) {
      current_tag_.pop_back();
      last_key_time_ = Clock::now();
      last_activity_time_ = last_key_time_;
    }
    shift_for_next_key_ = false;
    return true;
  }

  char ch = '\0';
  if (!key_code_to_char(code, ch)) {
    shift_for_next_key_ = false;
    return false;
  }

  current_tag_.push_back(ch);
  last_key_time_ = Clock::now();
  last_activity_time_ = last_key_time_;
  shift_for_next_key_ = false;
  return true;
}

bool HidScannerReader::key_code_to_char(unsigned short code, char & ch) const
{
  if (code >= KEY_1 && code <= KEY_9) {
    ch = static_cast<char>('1' + (code - KEY_1));
    return true;
  }
  if (code == KEY_0) {
    ch = '0';
    return true;
  }
  char letter = '\0';
  switch (code) {
    case KEY_KP0:
      ch = '0';
      return true;
    case KEY_KP1:
      ch = '1';
      return true;
    case KEY_KP2:
      ch = '2';
      return true;
    case KEY_KP3:
      ch = '3';
      return true;
    case KEY_KP4:
      ch = '4';
      return true;
    case KEY_KP5:
      ch = '5';
      return true;
    case KEY_KP6:
      ch = '6';
      return true;
    case KEY_KP7:
      ch = '7';
      return true;
    case KEY_KP8:
      ch = '8';
      return true;
    case KEY_KP9:
      ch = '9';
      return true;
    case KEY_A:
      letter = 'a';
      break;
    case KEY_B:
      letter = 'b';
      break;
    case KEY_C:
      letter = 'c';
      break;
    case KEY_D:
      letter = 'd';
      break;
    case KEY_E:
      letter = 'e';
      break;
    case KEY_F:
      letter = 'f';
      break;
    case KEY_G:
      letter = 'g';
      break;
    case KEY_H:
      letter = 'h';
      break;
    case KEY_I:
      letter = 'i';
      break;
    case KEY_J:
      letter = 'j';
      break;
    case KEY_K:
      letter = 'k';
      break;
    case KEY_L:
      letter = 'l';
      break;
    case KEY_M:
      letter = 'm';
      break;
    case KEY_N:
      letter = 'n';
      break;
    case KEY_O:
      letter = 'o';
      break;
    case KEY_P:
      letter = 'p';
      break;
    case KEY_Q:
      letter = 'q';
      break;
    case KEY_R:
      letter = 'r';
      break;
    case KEY_S:
      letter = 's';
      break;
    case KEY_T:
      letter = 't';
      break;
    case KEY_U:
      letter = 'u';
      break;
    case KEY_V:
      letter = 'v';
      break;
    case KEY_W:
      letter = 'w';
      break;
    case KEY_X:
      letter = 'x';
      break;
    case KEY_Y:
      letter = 'y';
      break;
    case KEY_Z:
      letter = 'z';
      break;
    default:
      break;
  }

  if (letter != '\0') {
    ch = shift_for_next_key_ ? static_cast<char>(std::toupper(letter)) : letter;
    return true;
  }
  return false;
}

void HidScannerReader::append_current_tag_if_valid()
{
  if (current_tag_.empty()) {
    last_key_time_ = Clock::time_point{};
    return;
  }
  if (config_.max_tags_per_location <= 0 ||
    tags_.size() < static_cast<std::size_t>(config_.max_tags_per_location))
  {
    if (seen_tags_.insert(current_tag_).second) {
      tags_.push_back(current_tag_);
    }
  }
  current_tag_.clear();
  last_key_time_ = Clock::time_point{};
  last_activity_time_ = Clock::now();
}

void HidScannerReader::maybe_finish_pending_tag_by_timeout()
{
  if (ready_after_quiet()) {
    append_current_tag_if_valid();
  }
}

void HidScannerReader::close_device()
{
  if (fd_ < 0) {
    grabbed_ = false;
    return;
  }
  if (grabbed_) {
    (void)::ioctl(fd_, EVIOCGRAB, 0);
    grabbed_ = false;
  }
  (void)::close(fd_);
  fd_ = -1;
}

void HidScannerReader::drain_input_events()
{
  if (fd_ < 0) {
    return;
  }
  while (true) {
    input_event event;
    const ssize_t bytes_read = ::read(fd_, &event, sizeof(event));
    if (bytes_read == static_cast<ssize_t>(sizeof(event))) {
      continue;
    }
    if (bytes_read < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
}

}  // namespace agv_inventory_system
