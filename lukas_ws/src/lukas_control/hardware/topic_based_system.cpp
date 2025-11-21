#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <lukas_control/topic_based_system.hpp>

namespace lukas_control
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;

CallbackReturn TopicBasedSystem::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    RCLCPP_INFO(rclcpp::get_logger("TopicBasedSystem"), "Initializing hardware...");

    size_t njoints = info.joints.size();
    joint_states_.resize(njoints);
    joint_commands_.resize(njoints);
    latest_positions_.resize(njoints, 0.0);

    for (size_t i = 0; i < njoints; i++) {
        joint_states_[i].resize(1, 0.0);
        joint_commands_[i].resize(1, 0.0);
    }

    // --- UART ---
    std::string device = "/dev/ttyUSB0";
    auto it = info_.hardware_parameters.find("uart_device");
    if (it != info_.hardware_parameters.end()) device = it->second;

  // --- UART OPEN WITH RETRY ---
for (int i = 0; i < 10; i++) {
    uart_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (uart_fd_ >= 0) break;

    RCLCPP_WARN(rclcpp::get_logger("TopicBasedSystem"),
                "UART %s not ready, retrying (%d/10)...",
                device.c_str(), i+1);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

if (uart_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TopicBasedSystem"),
                 "Failed to open UART device %s", device.c_str());
    return CallbackReturn::ERROR;
}

    struct termios tty{};
    if (tcgetattr(uart_fd_, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("TopicBasedSystem"), "Failed to get UART attributes");
        return CallbackReturn::ERROR;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("TopicBasedSystem"), "Failed to set UART attributes");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("TopicBasedSystem"), "UART connected on %s", device.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn TopicBasedSystem::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(rclcpp::get_logger("TopicBasedSystem"), "Hardware ACTIVATED");
    running_ = true;
    uart_thread_ = std::thread(&TopicBasedSystem::uart_loop, this);
    return CallbackReturn::SUCCESS;
}

CallbackReturn TopicBasedSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
    running_ = false;
    if (uart_thread_.joinable()) uart_thread_.join();
    if (uart_fd_ >= 0) ::close(uart_fd_);
    RCLCPP_INFO(rclcpp::get_logger("TopicBasedSystem"), "Hardware DEACTIVATED");
    return CallbackReturn::SUCCESS;
}

// ------------------------- UART THREAD -------------------------
void TopicBasedSystem::uart_loop()
{
    uint8_t rx[4];
    while (running_) {
        int n = ::read(uart_fd_, rx, sizeof(rx));
        if (n == 4) {
            uint8_t header = rx[0];
            uint8_t ang_h  = rx[1];
            uint8_t ang_l  = rx[2];
            uint8_t id     = header & 0x0F;

            uint16_t angle_raw = (ang_h << 8) | ang_l;
            double angle_rad = static_cast<double>(angle_raw) * (2.0 * M_PI / 4096.0);

            if (id > 0 && id <= latest_positions_.size()) {
                std::lock_guard<std::mutex> lock(uart_mutex_);
                latest_positions_[id - 1] = angle_rad;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ------------------------- STATE INTERFACES -------------------------
std::vector<hardware_interface::StateInterface> TopicBasedSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "position", &joint_states_[i][POSITION_INTERFACE_INDEX])
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TopicBasedSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, "position", &joint_commands_[i][POSITION_INTERFACE_INDEX])
        );
    }
    return command_interfaces;
}

// ------------------------- READ -------------------------
hardware_interface::return_type TopicBasedSystem::read(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    static uint8_t rx[5];
    static std::size_t idx = 0;

    uint8_t byte;
    int n = ::read(uart_fd_, &byte, 1);
    if (n <= 0) {
        // Nada nuevo
        return hardware_interface::return_type::OK;
    }

    // Máquina de estados muy simple: buscar 0xFF como primer byte
    if (idx == 0) {
        if (byte != 0xFF) {
            // ignorar hasta encontrar sync
            return hardware_interface::return_type::OK;
        }
        rx[0] = byte;
        idx = 1;
        return hardware_interface::return_type::OK;
    }

    // Estamos llenando la trama
    rx[idx++] = byte;

    if (idx < 5) {
        // Aún no tenemos trama completa
        return hardware_interface::return_type::OK;
    }

    // Aquí idx == 5 => tenemos 0xFF + 4 bytes de payload
    idx = 0;  // preparar para la siguiente

    // Reconstrucción segura de int16_t
    int16_t enc1 = static_cast<int16_t>(
                       (static_cast<uint16_t>(rx[1]) << 8) |
                        static_cast<uint16_t>(rx[2]));

    int16_t enc2 = static_cast<int16_t>(
                       (static_cast<uint16_t>(rx[3]) << 8) |
                        static_cast<uint16_t>(rx[4]));

    double angle1 = static_cast<double>(enc1);
    double angle2 = static_cast<double>(enc2);

    {
        std::lock_guard<std::mutex> lock(uart_mutex_);
        joint_states_[0][POSITION_INTERFACE_INDEX] = angle1;
        joint_states_[1][POSITION_INTERFACE_INDEX] = angle2;
    }

    return hardware_interface::return_type::OK;
}

// ------------------------- WRITE -------------------------
hardware_interface::return_type TopicBasedSystem::write(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    uint8_t packet[4];

    for (size_t i = 0; i < joint_commands_.size(); i++) {

        // Convertir comando
        int16_t value = static_cast<int16_t>(
            joint_commands_[i][POSITION_INTERFACE_INDEX] * (4096.0 / (2.0 * M_PI))
        );

        // Saturación
        if (value < 0)  value = 0;
        if (value > 4095) value = 4095;

        // ----- Formato -----
        packet[0] = 0xFF;                 // Identificador
        packet[1] = (i == 0 ? 0xFA : 0xFB);  // Motor 1:0xFA, Motor 2:0xFB
        packet[2] = value & 0xFF;         // LOW BYTE
        packet[3] = (value >> 8) & 0xFF;  // HIGH BYTE

        // Enviar paquete
        ::write(uart_fd_, packet, sizeof(packet));
    }

    return hardware_interface::return_type::OK;
}

}  // namespace lukas_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lukas_control::TopicBasedSystem, hardware_interface::SystemInterface)
