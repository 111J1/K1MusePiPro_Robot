#include "k1muse_mcu_bridge/mcu_bridge_node.hpp"
#include "k1muse_mcu_bridge/protocol.hpp"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace k1muse_mcu_bridge {
namespace {

constexpr const char * kDebugLogDir =
    "/home/bianbu/k1muse_communicate_ros/src/k1muse_mobile_bridge/debug_logs";

int64_t wall_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void append_debug_line(const std::string& event, const std::string& fields) {
    static std::mutex log_mutex;
    std::lock_guard<std::mutex> lock(log_mutex);
    std::filesystem::create_directories(kDebugLogDir);
    std::ofstream out(std::filesystem::path(kDebugLogDir) / "mcu_bridge_debug.tsv",
                      std::ios::app);
    out << wall_ms() << '\t' << event << '\t' << fields << '\n';
}

}  // namespace

using namespace std::chrono_literals;
using msg::ChassisStatus;
using msg::ChassisMov;
using srv::ChassisStop;
using srv::ChassisOdom;

McuBridgeNode::McuBridgeNode(const std::string& device, int baudrate)
    : Node("mcu_bridge_node"), serial_(device, baudrate) {
    log_tx_hex_ = declare_parameter<bool>("log_tx_hex", false);
    tx_log_every_n_ = declare_parameter<int>("tx_log_every_n", 100);
    rx_log_every_n_ = declare_parameter<int>("rx_log_every_n", 500);
    if (tx_log_every_n_ < 0) tx_log_every_n_ = 0;
    if (rx_log_every_n_ < 0) rx_log_every_n_ = 0;

    // ---- publishers ----------------------------------------------------
    chassis_status_pub_ = create_publisher<ChassisStatus>(
        "/mcu/chassis/status", 10);

    // ---- subscribers ---------------------------------------------------
    chassis_mov_sub_ = create_subscription<ChassisMov>(
        "/mcu/chassis/mov", 10,
        [this](const ChassisMov::SharedPtr msg) {
            append_debug_line(
                "mov_sub_rx",
                "move_cs=" + std::to_string(msg->move_cs) +
                    "\tdirection=" + std::to_string(msg->direction) +
                    "\tv=" + std::to_string(msg->v) +
                    "\tomega=" + std::to_string(msg->omega));
            uint8_t payload[CHASSIS_MOV_PAYLOAD_SIZE];
            payload[0] = msg->move_cs;
            write_le(payload + 1, msg->direction);
            write_le(payload + 5, msg->v);
            write_le(payload + 9, msg->omega);
            send_frame(Target::CHASSIS,
                       static_cast<uint8_t>(ChassisCmd::MOV),
                       payload, CHASSIS_MOV_PAYLOAD_SIZE);
        });

    // ---- service servers ------------------------------------------------
    chassis_stop_srv_ = create_service<ChassisStop>(
        "/mcu/chassis/stop",
        [this](const ChassisStop::Request::SharedPtr,
               ChassisStop::Response::SharedPtr) {
            append_debug_line("stop_srv_rx", "");
            send_frame(Target::CHASSIS,
                       static_cast<uint8_t>(ChassisCmd::STOP),
                       nullptr, 0);
        });

    chassis_odom_srv_ = create_service<ChassisOdom>(
        "/mcu/chassis/odom",
        [this](const ChassisOdom::Request::SharedPtr req,
               ChassisOdom::Response::SharedPtr) {
            uint8_t payload[CHASSIS_ODOM_PAYLOAD_SIZE];
            write_le(payload + 0, req->direction);
            write_le(payload + 4, req->x);
            write_le(payload + 8, req->y);
            send_frame(Target::CHASSIS,
                       static_cast<uint8_t>(ChassisCmd::ODOM),
                       payload, CHASSIS_ODOM_PAYLOAD_SIZE);
        });

    // ---- chassis adapter -------------------------------------------------
    // Keep odom/TF and optional /cmd_vel routing in the same lifecycle as
    // the serial bridge. No separate adapter node should be launched.
    odom_publisher_ = std::make_unique<OdomPublisher>(this);
    cmd_vel_router_ = std::make_unique<CmdVelRouter>(this);

    // ---- open serial & start read thread --------------------------------
    if (serial_.open()) {
        RCLCPP_INFO(get_logger(), "Serial port %s opened", device.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "Serial port %s not available, will retry",
                    device.c_str());
    }

    read_thread_ = std::thread(&McuBridgeNode::read_loop, this);
}

McuBridgeNode::~McuBridgeNode() {
    running_ = false;
    if (read_thread_.joinable())
        read_thread_.join();
}

void McuBridgeNode::send_frame(Target target, uint8_t cmd,
                               const uint8_t* payload, uint8_t len) {
    auto frame = build_frame(target, cmd, seq_++, payload, len);

    static int tx_count = 0;
    const int current_tx = ++tx_count;
    if (tx_log_every_n_ > 0 && current_tx % tx_log_every_n_ == 1) {
        if (log_tx_hex_) {
            std::string hex;
            for (size_t i = 0; i < frame.size(); i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", frame[i]);
                hex += buf;
            }
            RCLCPP_INFO(get_logger(),
                        "TX frame #%d: target=0x%02X cmd=0x%02X len=%u fd=%d bytes=%zu hex=%s",
                        current_tx, static_cast<uint8_t>(target), cmd, len,
                        serial_.fd(), frame.size(), hex.c_str());
        } else {
            RCLCPP_INFO(get_logger(),
                        "TX frame #%d: target=0x%02X cmd=0x%02X len=%u fd=%d bytes=%zu",
                        current_tx, static_cast<uint8_t>(target), cmd, len,
                        serial_.fd(), frame.size());
        }
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);
    const auto tx_start = wall_ms();
    auto written = serial_.write(frame.data(), frame.size());
    const auto tx_end = wall_ms();
    append_debug_line(
        "serial_tx",
        "target=" + std::to_string(static_cast<int>(static_cast<uint8_t>(target))) +
            "\tcmd=" + std::to_string(static_cast<int>(cmd)) +
            "\tseq=" + std::to_string(static_cast<int>(seq_ - 1)) +
            "\tbytes=" + std::to_string(frame.size()) +
            "\twritten=" + std::to_string(written) +
            "\tduration_ms=" + std::to_string(tx_end - tx_start) +
            "\tfd=" + std::to_string(serial_.fd()));
    if (written != static_cast<ssize_t>(frame.size()))
        RCLCPP_ERROR(get_logger(), "TX short write: %zd/%zu", written, frame.size());
}

void McuBridgeNode::handle_frame(const uint8_t* header,
                                 const uint8_t* payload,
                                 uint8_t payload_len) {
    auto src    = static_cast<Src>(header[0]);
    auto target = static_cast<Target>(header[1]);
    uint8_t cmd  = header[2];

    if (src != Src::MCU || cmd != STATUS_CMD) return;

    static int rx_count = 0;
    if (++rx_count && rx_log_every_n_ > 0 && rx_count % rx_log_every_n_ == 1)
        RCLCPP_INFO(get_logger(), "RX frame #%d: target=0x%02X cmd=0x%02X len=%d",
                    rx_count, static_cast<uint8_t>(target), cmd, payload_len);

    switch (target) {
    case Target::CHASSIS:
        if (payload_len == CHASSIS_STATUS_PAYLOAD_SIZE) {
            auto msg = ChassisStatus();
            msg.tick_ms          = read_le_u32(payload + 0);
            msg.state            = payload[4];
            msg.move_cs          = payload[5];
            msg.motor_block_flags = payload[6];
            msg.reserved         = payload[7];
            msg.wcs_vx           = read_le_f32(payload + 8);
            msg.wcs_vy           = read_le_f32(payload + 12);
            msg.omega            = read_le_f32(payload + 16);
            msg.wcs_x            = read_le_f32(payload + 20);
            msg.wcs_y            = read_le_f32(payload + 24);
            msg.wcs_direction    = read_le_f32(payload + 28);
            chassis_status_pub_->publish(msg);
        }
        break;

    case Target::ARM:
    case Target::LIFT:
        // reserved for extension
        break;

    default:
        break;
    }
}

void McuBridgeNode::read_loop() {
    enum State { WAIT_SOF1, WAIT_SOF2, READ_HEADER, READ_PAYLOAD, READ_CRC };
    State state = WAIT_SOF1;
    uint8_t header[5];
    int header_idx = 0;
    uint8_t payload[MAX_PAYLOAD];
    int payload_idx = 0;
    int payload_len = 0;

    while (running_) {
        // ---- reconnect loop ---------------------------------------------
        if (!serial_.is_open()) {
            std::this_thread::sleep_for(1s);
            if (!serial_.open()) continue;
            RCLCPP_INFO(get_logger(), "Serial port reconnected");
            state = WAIT_SOF1;  // reset parser state
        }

        uint8_t byte;
        ssize_t n = serial_.read(&byte, 1, 100);
        if (n <= 0) continue;

        switch (state) {
        case WAIT_SOF1:
            if (byte == SOF1) state = WAIT_SOF2;
            break;

        case WAIT_SOF2:
            if (byte == SOF2) {
                state = READ_HEADER;
                header_idx = 0;
            } else if (byte == SOF1) {
                state = WAIT_SOF2;
            } else {
                state = WAIT_SOF1;
            }
            break;

        case READ_HEADER:
            header[header_idx++] = byte;
            if (header_idx == 5) {
                payload_len = header[4];
                if (payload_len == 0) {
                    state = READ_CRC;
                } else if (payload_len <= MAX_PAYLOAD) {
                    state = READ_PAYLOAD;
                    payload_idx = 0;
                } else {
                    state = WAIT_SOF1;
                }
            }
            break;

        case READ_PAYLOAD:
            payload[payload_idx++] = byte;
            if (payload_idx == payload_len)
                state = READ_CRC;
            break;

        case READ_CRC: {
            // CRC covers SRC..PAYLOAD
            uint8_t crc_buf[5 + MAX_PAYLOAD];
            memcpy(crc_buf, header, 5);
            memcpy(crc_buf + 5, payload, payload_len);
            uint8_t expected = crc8(crc_buf, 5 + payload_len);

            if (byte == expected)
                handle_frame(header, payload, payload_len);

            state = WAIT_SOF1;
            break;
        }
        }
    }
}

}  // namespace k1muse_mcu_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<k1muse_mcu_bridge::McuBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
