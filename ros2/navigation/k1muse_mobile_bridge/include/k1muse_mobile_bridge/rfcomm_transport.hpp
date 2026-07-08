#ifndef K1MUSE_MOBILE_BRIDGE_RFCOMM_TRANSPORT_HPP_
#define K1MUSE_MOBILE_BRIDGE_RFCOMM_TRANSPORT_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <vector>

namespace k1muse_mobile_bridge
{

class RfcommTransport
{
public:
  RfcommTransport() = default;
  ~RfcommTransport();

  RfcommTransport(const RfcommTransport &) = delete;
  RfcommTransport & operator=(const RfcommTransport &) = delete;

  bool open_device(const std::string & path);
  bool is_open() const;
  void close_device();
  ssize_t read_some(uint8_t * buffer, size_t capacity, int timeout_ms);
  bool write_all(const std::vector<uint8_t> & data);
  void flush_output();

private:
  struct FdSnapshot
  {
    int fd;
    uint64_t generation;
  };

  FdSnapshot current_fd() const;
  void close_fd_if_current(FdSnapshot snapshot);

  mutable std::mutex state_mutex_;
  std::mutex read_mutex_;
  std::mutex write_mutex_;
  int fd_ = -1;
  uint64_t generation_ = 0;
};

}  // namespace k1muse_mobile_bridge

#endif  // K1MUSE_MOBILE_BRIDGE_RFCOMM_TRANSPORT_HPP_
