// Purpose: Interactive action client that sends waypoint goals from stdin.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_follower_ros2/action/follow_trajectory.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

// Interactive action client: reads waypoints from stdin and sends goals.
class TrajectoryActionClient : public rclcpp::Node {
public:
    using FollowTrajectory = trajectory_follower_ros2::action::FollowTrajectory;
    using GoalHandleFollowTrajectory = rclcpp_action::ClientGoalHandle<FollowTrajectory>;

    TrajectoryActionClient() : Node("trajectory_action_client") {
        this->declare_parameter<std::string>("frame_id", "odom");
        this->declare_parameter<double>("send_delay_sec", 0.2);

        frame_id_ = this->get_parameter("frame_id").as_string();
        send_delay_sec_ = this->get_parameter("send_delay_sec").as_double();

        client_ = rclcpp_action::create_client<FollowTrajectory>(
            this, "follow_trajectory");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(
                static_cast<int>(std::max(0.1, send_delay_sec_) * 1000.0)),
            std::bind(&TrajectoryActionClient::sendGoalFromInput, this));
    }

private:
    void sendGoalFromInput() {
        if (waiting_for_input_) {
            return;
        }
        waiting_for_input_ = true;
        std::string line;
        std::cout << "Enter waypoints as: x1,y1 x2,y2 x3,y3 (or 'quit'): " << std::flush;
        if (!std::getline(std::cin, line)) {
            rclcpp::shutdown();
            return;
        }
        if (line == "quit" || line == "exit") {
            rclcpp::shutdown();
            return;
        }

        std::vector<double> coords;
        if (!parseWaypoints(line, coords)) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid input. Example: 1,1 6,7 8,8 -1,5");
            waiting_for_input_ = false;
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available.");
            waiting_for_input_ = false;
            return;
        }

        auto goal_msg = FollowTrajectory::Goal();
        goal_msg.trajectory.header.frame_id = frame_id_;
        goal_msg.trajectory.header.stamp = this->now();

        for (size_t i = 0; i + 1 < coords.size(); i += 2) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = coords[i];
            pose.position.y = coords[i + 1];
            pose.orientation.w = 1.0;
            goal_msg.trajectory.poses.push_back(pose);
        }

        rclcpp_action::Client<FollowTrajectory>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const GoalHandleFollowTrajectory::SharedPtr &handle) {
                if (!handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted.");
                }
            };
        options.result_callback =
            [this](const GoalHandleFollowTrajectory::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Trajectory execution failed.");
                }
            };

        client_->async_send_goal(goal_msg, options);
        RCLCPP_INFO(this->get_logger(), "Sent trajectory with %zu points.", goal_msg.trajectory.poses.size());
        waiting_for_input_ = false;
    }

    bool parseWaypoints(const std::string &line, std::vector<double> &coords) {
        std::istringstream ss(line);
        std::string token;
        while (ss >> token) {
            auto comma = token.find(',');
            if (comma == std::string::npos) {
                return false;
            }
            try {
                double x = std::stod(token.substr(0, comma));
                double y = std::stod(token.substr(comma + 1));
                coords.push_back(x);
                coords.push_back(y);
            } catch (...) {
                return false;
            }
        }
        return coords.size() >= 4;
    }

    rclcpp_action::Client<FollowTrajectory>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;
    double send_delay_sec_{0.5};
    bool waiting_for_input_{false};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryActionClient>());
    rclcpp::shutdown();
    return 0;
}
