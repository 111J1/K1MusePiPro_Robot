#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_control_manager
{

class ByteTransport
{
public:
  virtual ~ByteTransport() = default;
  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual void write_bytes(const uint8_t * data, std::size_t size) = 0;
  virtual std::vector<uint8_t> read_available(std::chrono::milliseconds timeout) = 0;
};

class SerialTransport : public ByteTransport
{
public:
  SerialTransport(std::string port, int baud_rate);
  ~SerialTransport() override;

  void open() override;
  void close() override;
  bool is_open() const override;
  void write_bytes(const uint8_t * data, std::size_t size) override;
  std::vector<uint8_t> read_available(std::chrono::milliseconds timeout) override;

private:
  std::string port_;
  int baud_rate_;
  int fd_{-1};
};

}  // namespace k1muse_control_manager
