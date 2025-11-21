#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class LQRObserver : public rclcpp::Node
{
public:
    LQRObserver() : Node("lqr_observer")
    {
        sub_pos_ = create_subscription<std_msgs::msg::Float64>(
            "position", 10,
            std::bind(&LQRObserver::pos_callback, this, std::placeholders::_1));

        pub_vel_ = create_publisher<std_msgs::msg::Float64>("velocity_est", 10);

        // --- modelo discreto (ejemplo dt=0.001) ---
        dt_ = 0.001;
        A_ << 1, dt_,
              0, 1 - dt_*b_/J_;

        B_ << 0,
               dt_/J_;

        C_ << 1, 0;

        // --- Ganancia del observador (LQR style) ---
        // Pre-calculada offline con ARE discreto
        // Ajusta estos valores según tu planta
        L_ << 50.0,
              300.0;

        // Estado inicial
        xhat_ << 0.0,
                 0.0;

        timer_ = create_wall_timer(
            std::chrono::microseconds((int)(dt_ * 1e6)),
            std::bind(&LQRObserver::update, this));
    }

private:
    void pos_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pos_meas_ = msg->data;
        meas_received_ = true;
    }

    void update()
    {
        if (!meas_received_) return;

        double y = pos_meas_;

        // --- Observador discreto ---
        Eigen::Vector2d xhat_pred = A_ * xhat_;
        double yhat = (C_ * xhat_)(0);

        xhat_ = xhat_pred + L_ * (y - yhat);

        // --- publicar velocidad ---
        std_msgs::msg::Float64 vmsg;
        vmsg.data = xhat_(1);
        pub_vel_->publish(vmsg);
    }

    // Model parameters
    double J_ = 0.002;  // ajusta
    double b_ = 0.0002; // ajusta
    double dt_;

    // Matrices
    Eigen::Matrix2d A_;
    Eigen::Vector2d B_;
    Eigen::RowVector2d C_;
    Eigen::Vector2d L_;
    Eigen::Vector2d xhat_;

    double pos_meas_ = 0.0;
    bool meas_received_ = false;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_pos_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrObserver>());
  rclcpp::shutdown();
  return 0;
}