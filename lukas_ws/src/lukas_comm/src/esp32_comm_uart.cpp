#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>

class UartNode : public rclcpp::Node {
public:
    UartNode() : Node("uart_node"), run_thread_(true) {

        // ---- Publica bytes recibidos ----
        pub_rx_ = this->create_publisher<std_msgs::msg::UInt8>("esp32/uart_rx", 100);

        // ---- Recibe bytes para enviar ----
        sub_tx_ = this->create_subscription<std_msgs::msg::UInt8>(
            "esp32/uart_tx", 100,
            std::bind(&UartNode::onTx, this, std::placeholders::_1)
        );

        // ---- Abrir UART ----
        fd_ = open("/dev/tty.usbserial-1130", O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "No se pudo abrir el puerto UART");
            rclcpp::shutdown();
            return;
        }

        // ---- Configuración UART 115200 8N1 ----
        termios tty{};
        tcgetattr(fd_, &tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag = CREAD | CLOCAL | CS8;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_iflag = 0;
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;
        tcsetattr(fd_, TCSANOW, &tty);
        tcflush(fd_, TCIOFLUSH);

        // ---- Hilo lector a MAX velocidad ----
        reader_thread_ = std::thread(&UartNode::readLoop, this);

        RCLCPP_INFO(get_logger(), "🔥 UART Node listo a 115200!");
    }

    ~UartNode() {
        run_thread_ = false;
        if (reader_thread_.joinable())
            reader_thread_.join();
        close(fd_);
    }

private:
    void readLoop() {
        uint8_t buf[256];

        while (rclcpp::ok() && run_thread_) {
            int n = read(fd_, buf, sizeof(buf));
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    auto msg = std_msgs::msg::UInt8();
                    msg.data = buf[i];
                    pub_rx_->publish(msg);
                }
            }
        }
    }

    void onTx(const std_msgs::msg::UInt8::SharedPtr msg) {
        uint8_t b = msg->data;
        write(fd_, &b, 1);
    }

    int fd_;
    std::atomic<bool> run_thread_;
    std::thread reader_thread_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_rx_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_tx_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UartNode>());
    rclcpp::shutdown();
    return 0;
}
