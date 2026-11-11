// Copyright 2026 Lin Bochuan
// Created by chuan on 2026/11/8.
//

#include <hero_chassis_controller/hero_chassis_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{

    bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                     ros::NodeHandle & root_nh, ros::NodeHandle & controller_nh)
    {
        //从硬件层获得关节权柄
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        //初始化PID
        if (!front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left_wheel"))) return false;
        if (!front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right_wheel"))) return false;
        if (!back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left_wheel"))) return false;
        if (!back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right_wheel"))) return false;

        //front_left_pid_.printValues();   // 临时：确认参数读进来了

        //读参数
        controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
        controller_nh.param("wheel_base", wheel_base_, 0.4);
        controller_nh.param("wheel_track", wheel_track_, 0.4);
        //订阅
        cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

        odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 1);

        return true;
    }

    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 把 msg 里的数据存进 buffer
        cmd_vel_buffer_.writeFromNonRT(*msg);
    }


    void HeroChassisController::starting(const ros::Time&)
    {
        front_left_pid_.reset();
        front_right_pid_.reset();
        back_left_pid_.reset();
        back_right_pid_.reset();
        // last_switch_ = ros::Time(0);
        // target_velocity_ = 0.0;
        x_ = 0.0;
        y_ = 0.0;
        th = 0.0;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        //这里是演示方波
        // if ((time - last_switch_).toSec() > 3.0) {
        //     target_velocity_ = (target_velocity_ == 0.0) ? 2.0 : 0.0;
        //     last_switch_ = time;
        // }
        //
        // front_left_joint_.setCommand(
        //     front_left_pid_.computeCommand(target_velocity_ - front_left_joint_.getVelocity(), period));
        // front_right_joint_.setCommand(
        //     front_right_pid_.computeCommand(target_velocity_ - front_right_joint_.getVelocity(), period));
        // back_left_joint_.setCommand(
        //     back_left_pid_.computeCommand(target_velocity_ - back_left_joint_.getVelocity(), period));
        // back_right_joint_.setCommand(
        //     back_right_pid_.computeCommand(target_velocity_ - back_right_joint_.getVelocity(), period));
        //到这里为止

        //取出指令
        const geometry_msgs::Twist cmd = *cmd_vel_buffer_.readFromRT();

        //wheel_base_ 前后轮距； wheel_track_ 左右轮距
        const double half = (wheel_base_ + wheel_track_) / 2.0;
        const double r    = wheel_radius_;

        //逆运动学：底盘速度 变为 四个轮子的目标转速
        //四个轮子的目标转速、符号由轮子倾角决定：
        // 对角两轮同号，相邻两轮反向，与 simple_chassis_controller 的力矩表一致
        const double omega_fl = (cmd.linear.x - cmd.linear.y - half * cmd.angular.z) / r;
        const double omega_fr = (cmd.linear.x + cmd.linear.y + half * cmd.angular.z) / r;
        const double omega_bl = (cmd.linear.x + cmd.linear.y - half * cmd.angular.z) / r;
        const double omega_br = (cmd.linear.x - cmd.linear.y + half * cmd.angular.z) / r;

        //四路 PID
        front_left_joint_.setCommand(
            front_left_pid_.computeCommand(omega_fl - front_left_joint_.getVelocity(), period));
        front_right_joint_.setCommand(
            front_right_pid_.computeCommand(omega_fr - front_right_joint_.getVelocity(), period));
        back_left_joint_.setCommand(
            back_left_pid_.computeCommand(omega_bl - back_left_joint_.getVelocity(), period));
        back_right_joint_.setCommand(
            back_right_pid_.computeCommand(omega_br - back_right_joint_.getVelocity(), period));

        //正运动学：轮子转速 变成 底盘速度
        const double w_fl = front_left_joint_.getVelocity();
        const double w_fr = front_right_joint_.getVelocity();
        const double w_bl = back_left_joint_.getVelocity();
        const double w_br = back_right_joint_.getVelocity();

        const double car_vx = r * ( w_fl + w_fr + w_bl + w_br) / 4.0;
        const double car_vy = r * (-w_fl + w_fr + w_bl - w_br) / 4.0;
        const double car_wz = r * (-w_fl + w_fr - w_bl + w_br) / (4.0 * half);

        //积分成位姿
        x_   += (car_vx * cos(th) - car_vy * sin(th)) * period.toSec();
        y_   += (car_vx * sin(th) + car_vy * cos(th)) * period.toSec();
        th += car_wz * period.toSec();


        //发布（降频到 1/10）
        if (++odom_publish_count_ % 10 == 0)
        {
            nav_msgs::Odometry odom;
            odom.header.stamp    = time;
            odom.header.frame_id = "odom";
            odom.child_frame_id  = "base_link";

            odom.pose.pose.position.x  = x_;
            odom.pose.pose.position.y  = y_;
            odom.pose.pose.position.z  = 0.0;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

            odom.twist.twist.linear.x  = car_vx;
            odom.twist.twist.linear.y  = car_vy;
            odom.twist.twist.angular.z = car_wz;

            odom_pub_.publish(odom);

            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = time;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id  = "base_link";
            
            tf_msg.transform.translation.x = x_;
            tf_msg.transform.translation.y = y_;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = tf::createQuaternionMsgFromYaw(th);
            tf_broadcaster_.sendTransform(tf_msg);
        }

    }

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
