// Copyright 2026 Lin Bochuan
// Created by chuan on 2026/11/8.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_buffer.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
                  ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

    void starting(const ros::Time&) override;

    void update(const ros::Time& time, const ros::Duration& period) override;

private:

    //bool test_mode_{true};

    hardware_interface::JointHandle front_left_joint_;
    hardware_interface::JointHandle front_right_joint_;
    hardware_interface::JointHandle back_left_joint_;
    hardware_interface::JointHandle back_right_joint_;

    control_toolbox::Pid front_left_pid_;
    control_toolbox::Pid front_right_pid_;
    control_toolbox::Pid back_left_pid_;
    control_toolbox::Pid back_right_pid_;

    // 测试用的目标转速与切换时刻（方波）
    // double target_velocity_{0.0};
    // ros::Time last_switch_;

    // 速度指令接收存储
    ros::Subscriber cmd_vel_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;

    double wheel_radius_{0.07625};
    double wheel_base_{0.4};
    double wheel_track_{0.4};

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    // 里程计
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    double x_{0.0};
    double y_{0.0};
    double th{0.0};
    int odom_publish_count_{0};
};

}  // namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
