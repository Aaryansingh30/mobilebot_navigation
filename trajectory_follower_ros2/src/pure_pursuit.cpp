// Purpose: Reference pure pursuit controller implementation.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <vector>

// Reference pure pursuit controller (not used in current launch).

class PurePursuitController : public rclcpp::Node{
public:
    PurePursuitController() : Node("Pure_Pursuit_Controller"){
        odom_sub_ = this 
        -> create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PurePursuitController::OdomCallback, this, std::placeholders::_1));
        path_sub_ = this -> create_subscription<nav_msgs::msg::Path>("/smooth_path", 10, std::bind(&PurePursuitController::PathCallback, this, std::placeholders::_1));
        cmd_pub_ = this -> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(50), 
        std::bind(&PurePursuitController::controlLoop, this));

        linear_velocity_ = 1.0;
        kv_ = 1.0;
        L_min_ = 0.4;
        goal_tolerance_ = 0.1;
    }
private: 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_;
    bool path_received_ = false;

    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double linear_velocity_;
    double kv_;
    double L_min_;
    double goal_tolerance_;

    int current_target_index_ = 0;

    void PathCallback(const nav_msgs::msg::Path::SharedPtr msg){
        path_ = *msg;
        path_received_ = true;
        current_target_index_ = 0;
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;

        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_yaw_ = std::atan2(siny, cosy);
    }

    double ComputeLookAheadDistance(){
        return kv_ * linear_velocity_ + L_min_;
    }

    void controlLoop()
    {

        if(!path_received_) return;

        double lookahead_distance = ComputeLookAheadDistance();

        geometry_msgs::msg::PoseStamped target_point;

        bool found = false;

        for(size_t i = static_cast<size_t>(current_target_index_); i < path_.poses.size(); i++)
        {
            auto pose = path_.poses[i];

            double dx =
                pose.pose.position.x - robot_x_;

            double dy =
                pose.pose.position.y - robot_y_;

            double dist =
                std::sqrt(dx*dx + dy*dy);

            if(dist >= lookahead_distance)
            {
                target_point = pose;
                current_target_index_ = i;
                found = true;
                break;
            }
        }

        if(!found)
        {
            stopRobot();
            return;
        }

        double dx =
            target_point.pose.position.x - robot_x_;

        double dy =
            target_point.pose.position.y - robot_y_;

        double local_y =
           -std::sin(robot_yaw_)*dx +
            std::cos(robot_yaw_)*dy;

        double curvature =
            (2*local_y) /
            (lookahead_distance*lookahead_distance);

        double angular_velocity =
            linear_velocity_ * curvature;

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = linear_velocity_;
        cmd.angular.z = angular_velocity;

        cmd_pub_->publish(cmd);

        checkGoalReached();
    }

    void checkGoalReached()
    {
        auto goal = path_.poses.back();

        double dx =
            goal.pose.position.x - robot_x_;

        double dy =
            goal.pose.position.y - robot_y_;

        double dist =
            std::sqrt(dx*dx + dy*dy);

        if(dist < goal_tolerance_)
        {
            stopRobot();
        }
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        cmd_pub_->publish(cmd);
    }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    auto node =
        std::make_shared<PurePursuitController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
