#pragma once
#include <atomic>
#include <thread>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "k1muse_mcu_bridge/msg/chassis_status.hpp"
#include "k1muse_mcu_bridge/msg/chassis_mov.hpp"
#include "k1muse_mcu_bridge/srv/chassis_stop.hpp"
#include "k1muse_mcu_bridge/srv/chassis_odom.hpp"

#include "k1muse_mcu_bridge/protocol.hpp"
#include "k1muse_mcu_bridge/serial_port.hpp"
#include "k1muse_mcu_bridge/odom_publisher.hpp"
#include "k1muse_mcu_bridge/cmd_vel_router.hpp"

namespace k1muse_mcu_bridge {

class McuBridgeNode : public rclcpp::Node {
public:
    explicit McuBridgeNode(const std::string& device = "/dev/mymcu",
                           int baudrate = 115200);
    ~McuBridgeNode();

private:
    // ---- serial ---------------------------------------------------------
    SerialPort serial_;

    // ---- publishers (MCU -> ROS) ----------------------------------------
    rclcpp::Publisher<msg::ChassisStatus>::SharedPtr chassis_status_pub_;

    // ---- subscribers (ROS -> MCU) ---------------------------------------
    rclcpp::Subscription<msg::ChassisMov>::SharedPtr chassis_mov_sub_;

    // ---- service servers ------------------------------------------------
    rclcpp::Service<srv::ChassisStop>::SharedPtr chassis_stop_srv_;
    rclcpp::Service<srv::ChassisOdom>::SharedPtr chassis_odom_srv_;

    // ---- chassis adapter, same lifetime as MCU bridge -------------------
    std::unique_ptr<OdomPublisher> odom_publisher_;
    std::unique_ptr<CmdVelRouter> cmd_vel_router_;

    // ---- tx -------------------------------------------------------------
    std::mutex tx_mutex_;
    uint8_t seq_ = 0;
    bool log_tx_hex_ = false;
    int tx_log_every_n_ = 100;
    int rx_log_every_n_ = 500;

    void send_frame(Target target, uint8_t cmd,
                    const uint8_t* payload, uint8_t len);

    // ---- read thread ----------------------------------------------------
    std::thread read_thread_;
    std::atomic<bool> running_{true};
    void read_loop();

    // ---- rx frame handler -----------------------------------------------
    void handle_frame(const uint8_t* header, const uint8_t* payload,
                      uint8_t payload_len);
};

}  // namespace k1muse_mcu_bridge
