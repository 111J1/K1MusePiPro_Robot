#pragma once
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <string>

namespace k1muse_mcu_bridge {

class SerialPort {
public:
    SerialPort(const std::string& device, int baudrate);
    ~SerialPort();

    bool open();
    void close();
    bool is_open() const;
    int fd() const { return fd_; }

    // non-blocking read with timeout (ms), returns bytes read or -1 on error
    ssize_t read(uint8_t* buf, size_t len, int timeout_ms);

    // blocking write, thread-safe
    ssize_t write(const uint8_t* data, size_t len);

private:
    std::string device_;
    int baudrate_;
    int fd_ = -1;
    std::mutex write_mutex_;
};

}  // namespace k1muse_mcu_bridge
