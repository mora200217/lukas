#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class UARTNode : public rclcpp::Node {
public:
    UARTNode() : Node("uart_node") {
        serial_port_ = open("/dev/ttyUSB0", O_RDWR);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART");
            return;
        }

        struct termios tty;
        tcgetattr(serial_port_, &tty);
        tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tcflush(serial_port_, TCIFLUSH);
        tcsetattr(serial_port_, TCSANOW, &tty);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&UARTNode::loop, this)
        );
    }

private:
    void loop() {
        // ---- SEND ----
        std::string msg = "Hello UART\n";
        write(serial_port_, msg.c_str(), msg.size());

        // ---- RECEIVE ----
        char buffer[256];
        int n = read(serial_port_, &buffer, sizeof(buffer));
        if (n > 0) {
            std::string received(buffer, n);
            RCLCPP_INFO(this->get_logger(), "RX: %s", received.c_str());
        }
    }

    int serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTNode>());
    rclcpp::shutdown();
    return 0;
}
