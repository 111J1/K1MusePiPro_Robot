#include "k1muse_control_manager/serial_transport.hpp"

#include <stdexcept>
#include <system_error>
#include <utility>

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace k1muse_control_manager
{

namespace
{

#ifndef _WIN32
speed_t baud_to_termios(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud_rate));
  }
}
#endif

}  // namespace

SerialTransport::SerialTransport(std::string port, int baud_rate)
: port_(std::move(port)), baud_rate_(baud_rate)
{
}

SerialTransport::~SerialTransport()
{
  close();
}

void SerialTransport::open()
{
#ifdef _WIN32
  throw std::runtime_error("SerialTransport is implemented for POSIX/K1 Linux targets");
#else
  if (fd_ >= 0) {
    return;
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "open " + port_);
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const auto err = errno;
    close();
    throw std::system_error(err, std::generic_category(), "tcgetattr " + port_);
  }

  cfmakeraw(&tty);
  const speed_t speed = baud_to_termios(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const auto err = errno;
    close();
    throw std::system_error(err, std::generic_category(), "tcsetattr " + port_);
  }
  tcflush(fd_, TCIOFLUSH);
#endif
}

void SerialTransport::close()
{
#ifndef _WIN32
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#else
  fd_ = -1;
#endif
}

bool SerialTransport::is_open() const
{
  return fd_ >= 0;
}

void SerialTransport::write_bytes(const uint8_t * data, std::size_t size)
{
#ifdef _WIN32
  (void)data;
  (void)size;
  throw std::runtime_error("SerialTransport is implemented for POSIX/K1 Linux targets");
#else
  if (fd_ < 0) {
    throw std::runtime_error("serial port is not open");
  }
  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd_, data + written, size - written);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      throw std::system_error(errno, std::generic_category(), "write " + port_);
    }
    written += static_cast<std::size_t>(n);
  }
  tcdrain(fd_);
#endif
}

std::vector<uint8_t> SerialTransport::read_available(std::chrono::milliseconds timeout)
{
  std::vector<uint8_t> data;
#ifdef _WIN32
  (void)timeout;
  throw std::runtime_error("SerialTransport is implemented for POSIX/K1 Linux targets");
#else
  if (fd_ < 0) {
    return data;
  }

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd_, &read_set);

  timeval tv{};
  tv.tv_sec = static_cast<long>(timeout.count() / 1000);
  tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

  const int ret = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
  if (ret < 0) {
    if (errno == EINTR) {
      return data;
    }
    throw std::system_error(errno, std::generic_category(), "select " + port_);
  }
  if (ret == 0 || !FD_ISSET(fd_, &read_set)) {
    return data;
  }

  uint8_t buffer[256];
  const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
  if (n < 0) {
    if (errno == EINTR || errno == EAGAIN) {
      return data;
    }
    throw std::system_error(errno, std::generic_category(), "read " + port_);
  }
  data.assign(buffer, buffer + n);
  return data;
#endif
}

}  // namespace k1muse_control_manager
