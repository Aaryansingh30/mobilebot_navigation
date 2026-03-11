// Purpose: Compare planned timed path to odometry and log tracking error.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <cmath>

// Monitors tracking error between /timed_path (planned) and /odom (actual).
class TrackingErrorMonitor : public rclcpp::Node {
public:
    TrackingErrorMonitor() : Node("tracking_error_monitor") {
        this->declare_parameter<double>("log_period_sec", 0.5);
        log_period_sec_ = this->get_parameter("log_period_sec").as_double();

        timed_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/timed_path", 10,
            std::bind(&TrackingErrorMonitor::onTimedPath, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TrackingErrorMonitor::onOdom, this, std::placeholders::_1));
        goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&TrackingErrorMonitor::onGoalReached, this, std::placeholders::_1));

        auto period = std::chrono::milliseconds(
            static_cast<int>(std::max(0.1, log_period_sec_) * 1000.0));
        timer_ = this->create_wall_timer(period, std::bind(&TrackingErrorMonitor::logError, this));
    }

private:
    void onTimedPath(const nav_msgs::msg::Path::SharedPtr msg) {
        timed_path_ = *msg;
        has_path_ = !timed_path_.poses.empty();
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        has_odom_ = true;
    }

    void logError() {
        if (!has_path_ || !has_odom_) {
            return;
        }
        if (goal_reached_) {
            return;
        }

        const rclcpp::Time now = this->now();
        if (timed_path_.poses.empty()) {
            return;
        }

        const auto t0_msg = timed_path_.poses.front().header.stamp;
        const rclcpp::Time t0(t0_msg);
        const double t_rel = (now - t0).seconds();

        // Find nearest timed pose by timestamp (linear search, OK for small paths).
        size_t best_idx = 0;
        double best_dt = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < timed_path_.poses.size(); ++i) {
            const rclcpp::Time ti(timed_path_.poses[i].header.stamp);
            const double dt = std::fabs((ti - t0).seconds() - t_rel);
            if (dt < best_dt) {
                best_dt = dt;
                best_idx = i;
            }
        }

        const auto &ref = timed_path_.poses[best_idx].pose.position;
        const auto &act = last_odom_.pose.pose.position;
        const double dx = act.x - ref.x;
        const double dy = act.y - ref.y;
        const double err = std::sqrt(dx * dx + dy * dy);

        RCLCPP_INFO(this->get_logger(),
            "t=%.2f s | ref(%.2f,%.2f) act(%.2f,%.2f) err=%.3f",
            t_rel, ref.x, ref.y, act.x, act.y, err);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr timed_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path timed_path_;
    nav_msgs::msg::Odometry last_odom_;
    bool has_path_{false};
    bool has_odom_{false};
    bool goal_reached_{false};
    double log_period_sec_{0.5};

    void onGoalReached(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            goal_reached_ = true;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingErrorMonitor>());
    rclcpp::shutdown();
    return 0;
}
