// Purpose: Action server that smooths waypoints, time-parameterizes a path, and publishes planning topics.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <cmath>
#include "trajectory_follower_ros2/action/follow_trajectory.hpp"
#include "trajectory_follower_ros2/cubic_spline.hpp"
#include "trajectory_follower_ros2/trajectory_time_param.hpp"

// Action server: smooth waypoints, time-parameterize trajectory, and publish path topics.
class TrajectoryActionServer : public rclcpp::Node{
public:
    using FollowTrajectory = trajectory_follower_ros2::action::FollowTrajectory;
    using GoalHandleFollowTrajectory = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

    TrajectoryActionServer() : Node("Trajectory_Action_Server"){
        action_server_ = rclcpp_action::create_server<FollowTrajectory>(
            this,
            "follow_trajectory",
            std::bind(&TrajectoryActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectoryActionServer::handle_accepted, this, std::placeholders::_1)
        );
        trajectory_speed_ = this->declare_parameter<double>("traj_max_vel", 0.4);
        trajectory_accel_ = this->declare_parameter<double>("traj_max_acc", 0.6);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_path", 10);
        rolled_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_path_from_robot", 10);
        input_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/input_waypoints", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        timed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/timed_path", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryActionServer::OdomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Trajectory action server started");
    }
private:
    rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rolled_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr input_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr timed_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::PoseStamped last_robot_pose_;
    bool has_odom_{false};
    nav_msgs::msg::Path last_path_msg_;
    bool has_path_{false};
    geometry_msgs::msg::PoseStamped start_pose_;
    bool has_start_pose_{false};
    std::mutex path_mutex_; // protects last_path_msg_ and start pose
    double trajectory_speed_{0.4};
    double trajectory_accel_{0.6};
    CubicSplineInterpolator interpolator_;
    double interpolation_resolution_{0.1};

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowTrajectory::Goal> goal){
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received trajectory goal");
        RCLCPP_INFO(this->get_logger(), "Number of poses : %ld", goal->trajectory.poses.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle){
        std::thread(std::bind(&TrajectoryActionServer::execute, this, std::placeholders::_1), goal_handle).detach();
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        last_robot_pose_.header = msg->header;
        last_robot_pose_.pose = msg->pose.pose;
        has_odom_ = true;

        nav_msgs::msg::Path rolled;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if(!has_path_){
                return;
            }
            rolled = last_path_msg_;
            if (!has_start_pose_) {
                start_pose_ = last_robot_pose_;
                has_start_pose_ = true;
            }
        }
        if (!rolled.poses.empty()) {
            rolled.header.stamp = this->now();
            rolled.poses.insert(rolled.poses.begin(), start_pose_);
            rolled_path_pub_->publish(rolled);
        }
    }

    void execute(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Executing trajectory goal");
        const auto goal = goal_handle->get_goal();

        geometry_msgs::msg::PoseArray input_msg = goal->trajectory;
        input_msg.header.stamp = this->now();
        input_pub_->publish(input_msg);

        auto dense_path = interpolator_.interpolate(goal->trajectory, interpolation_resolution_);
        if (dense_path.header.frame_id.empty()) {
            dense_path.header.frame_id = goal->trajectory.header.frame_id;
        }
        if (dense_path.header.frame_id.empty()) {
            dense_path.header.frame_id = "odom";
        }
        RCLCPP_INFO(this->get_logger(), "Interpolated trajectory has %ld poses", dense_path.poses.size());

        nav_msgs::msg::Path path_msg;
        path_msg.header = dense_path.header;
        path_msg.header.stamp = this->now();
        path_msg.poses.reserve(dense_path.poses.size());
        for (const auto& pose : dense_path.poses){
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose = pose;
            path_msg.poses.push_back(ps);
        }
        path_pub_->publish(path_msg);

        // Time-parameterize the path with a trapezoidal velocity profile.
        nav_msgs::msg::Path timed_path = path_msg;
        std::vector<double> s;
        s.reserve(timed_path.poses.size());
        double accum_dist = 0.0;
        s.push_back(0.0);
        for (size_t i = 1; i < timed_path.poses.size(); ++i) {
            const auto &p0 = timed_path.poses[i - 1].pose.position;
            const auto &p1 = timed_path.poses[i].pose.position;
            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            accum_dist += std::sqrt(dx * dx + dy * dy);
            s.push_back(accum_dist);
        }
        const auto t = compute_time_stamps(s, trajectory_speed_, trajectory_accel_);
        const rclcpp::Time t0 = this->now();
        for (size_t i = 0; i < timed_path.poses.size() && i < t.size(); ++i) {
            timed_path.poses[i].header.stamp = t0 + rclcpp::Duration::from_seconds(t[i]);
        }
        timed_path_pub_->publish(timed_path);

        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            last_path_msg_ = path_msg;
            has_path_ = true;
            has_start_pose_ = false;
        }

        if (!path_msg.poses.empty()) {
            geometry_msgs::msg::PoseStamped goal_pose = path_msg.poses.back();
            goal_pose.header.stamp = this->now();
            goal_pub_->publish(goal_pose);
        }

        if (!timed_path.poses.empty()) {
            const auto t0 = timed_path.poses.front().header.stamp;
            const rclcpp::Time t0_time(t0);
            for (size_t i = 0; i < timed_path.poses.size(); i++) {
                const auto &p = timed_path.poses[i].pose.position;
                const rclcpp::Time ti(timed_path.poses[i].header.stamp);
                const double t = (ti - t0_time).seconds();
                RCLCPP_INFO(this->get_logger(), "Pose %zu -> x: %.2f, y: %.2f, t: %.2f",
                    i, p.x, p.y, t);
            }
        }
        auto result = std::make_shared<FollowTrajectory::Result>();

        result->success = true;

        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
    }   
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
