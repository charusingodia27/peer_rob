#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

struct State {
    double x, y, theta;
};

class MPCController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;
    State current_state_, goal_state_;
    double control_frequency_;

public:
    MPCController(ros::NodeHandle& nh) : nh_(nh) {
        odom_sub_ = nh_.subscribe("odom", 10, &MPCController::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // Goal state (example target, can be changed dynamically)
        goal_state_ = {5.0, 5.0, 0.0};
        control_frequency_ = 10.0;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        current_state_.theta = tf::getYaw(msg->pose.pose.orientation);
    }

    void mpcLoop() {
        ros::Rate rate(control_frequency_);
        while (ros::ok()) {
            geometry_msgs::Twist cmd_vel;
            double distance_to_goal = sqrt(pow(goal_state_.x - current_state_.x, 2) + pow(goal_state_.y - current_state_.y, 2));
            
            if (distance_to_goal > 0.1) {
                cmd_vel.linear.x = 0.5 * distance_to_goal;
                double angle_to_goal = atan2(goal_state_.y - current_state_.y, goal_state_.x - current_state_.x);
                cmd_vel.angular.z = 2.0 * (angle_to_goal - current_state_.theta);
            }
            
            cmd_pub_.publish(cmd_vel);
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;
    MPCController mpc_controller(nh);
    mpc_controller.mpcLoop();
    return 0;
}

