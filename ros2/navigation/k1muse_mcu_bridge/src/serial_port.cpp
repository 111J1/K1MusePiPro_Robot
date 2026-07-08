#include "k1muse_mcu_bridge/serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <cstring>
#include <cerrno>

namespace k1muse_mcu_bridge {

SerialPort::SerialPort(const std::string& device, int baudrate)
    : device_(device), baudrate_(baudrate) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open() {
    if (fd_ >= 0) close();

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= (CS8 | CREAD | CLOCAL);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::is_open() const {
    return fd_ >= 0;
}

ssize_t SerialPort::read(uint8_t* buf, size_t len, int timeout_ms) {
    if (fd_ < 0) return -1;

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) return -1;
    if (ret == 0) return 0;

    return ::read(fd_, buf, len);
}

ssize_t SerialPort::write(const uint8_t* data, size_t len) {
    if (fd_ < 0) return -1;
    std::lock_guard<std::mutex> lock(write_mutex_);
    return ::write(fd_, data, len);
}

}  // namespace k1muse_mcu_bridge
