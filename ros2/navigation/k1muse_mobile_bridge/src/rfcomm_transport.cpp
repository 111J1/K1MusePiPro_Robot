#include "k1muse_mobile_bridge/rfcomm_transport.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace k1muse_mobile_bridge
{
namespace
{

bool wait_writable(int fd, int timeout_ms)
{
  fd_set write_fds;
  FD_ZERO(&write_fds);
  FD_SET(fd, &write_fds);

  timeval tv {};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  const int rc = ::select(fd + 1, nullptr, &write_fds, nullptr, &tv);
  return rc > 0 && FD_ISSET(fd, &write_fds);
}

}  // namespace

RfcommTransport::~RfcommTransport()
{
  close_device();
}

bool RfcommTransport::open_device(const std::string & path)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (fd_ >= 0) {
    (void)::close(fd_);
    fd_ = -1;
    generation_++;
  }

  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    return false;
  }
  generation_++;

  int fd_flags = ::fcntl(fd_, F_GETFD, 0);
  if (fd_flags >= 0) {
    (void)::fcntl(fd_, F_SETFD, fd_flags | FD_CLOEXEC);
  }

  termios tio {};
  if (::tcgetattr(fd_, &tio) == 0) {
    cfmakeraw(&tio);
    tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    (void)::tcsetattr(fd_, TCSANOW, &tio);
  }

  // Keep O_NONBLOCK so that TX never blocks the kernel Bluetooth stack.
  // A blocked write on RFCOMM can stall incoming data in the kernel,
  // causing seconds of teleop latency.
  return true;
}

bool RfcommTransport::is_open() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return fd_ >= 0;
}

void RfcommTransport::close_device()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (fd_ >= 0) {
    (void)::close(fd_);
    fd_ = -1;
    generation_++;
  }
}

ssize_t RfcommTransport::read_some(uint8_t * buffer, size_t capacity, int timeout_ms)
{
  std::lock_guard<std::mutex> read_lock(read_mutex_);
  const FdSnapshot snapshot = current_fd();
  if (snapshot.fd < 0) {
    return -1;
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(snapshot.fd, &read_fds);

  timeval tv {};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rc = ::select(snapshot.fd + 1, &read_fds, nullptr, nullptr, &tv);
  if (rc < 0) {
    if (errno == EINTR) {
      return 0;
    }
    close_fd_if_current(snapshot);
    return -1;
  }
  if (rc == 0 || !FD_ISSET(snapshot.fd, &read_fds)) {
    return 0;
  }

  ssize_t n = ::read(snapshot.fd, buffer, capacity);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // non-blocking fd, no data available right now
    }
    close_fd_if_current(snapshot);
    return -1;
  }
  if (n == 0) {
    // EOF on RFCOMM — peer disconnected
    close_fd_if_current(snapshot);
    return -1;
  }
  return n;
}

bool RfcommTransport::write_all(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  const FdSnapshot snapshot = current_fd();
  if (snapshot.fd < 0) {
    return false;
  }

  size_t written = 0;
  while (written < data.size()) {
    ssize_t rc = ::write(snapshot.fd, data.data() + written, data.size() - written);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (written > 0U && wait_writable(snapshot.fd, 25)) {
          continue;
        }
        if (written > 0U) {
          (void)::tcflush(snapshot.fd, TCOFLUSH);
        }
        static int64_t eagain_count = 0;
        if (++eagain_count <= 20 || eagain_count % 100 == 0) {
          std::fprintf(stderr, "[rfcomm] write EAGAIN #%ld (buffer full, frame dropped)\n",
                       (long)eagain_count);
        }
        return false;
      }
      close_fd_if_current(snapshot);
      return false;
    }
    if (rc == 0) {
      close_fd_if_current(snapshot);
      return false;
    }
    written += static_cast<size_t>(rc);
  }
  return true;
}

void RfcommTransport::flush_output()
{
  std::lock_guard<std::mutex> write_lock(write_mutex_);
  const FdSnapshot snapshot = current_fd();
  if (snapshot.fd >= 0) {
    (void)::tcflush(snapshot.fd, TCOFLUSH);
  }
}

RfcommTransport::FdSnapshot RfcommTransport::current_fd() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return FdSnapshot{fd_, generation_};
}

void RfcommTransport::close_fd_if_current(FdSnapshot snapshot)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (fd_ == snapshot.fd && generation_ == snapshot.generation) {
    (void)::close(fd_);
    fd_ = -1;
    generation_++;
  }
}

}  // namespace k1muse_mobile_bridge
