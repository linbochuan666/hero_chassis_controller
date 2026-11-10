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

        front_left_pid_.printValues();   // 临时：确认参数真的读进来了

        return true;
    }


    void HeroChassisController::starting(const ros::Time&)
    {
        front_left_pid_.reset();
        front_right_pid_.reset();
        back_left_pid_.reset();
        back_right_pid_.reset();
        last_switch_ = ros::Time(0);
        target_velocity_ = 0.0;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        //double error = target_velocity_[i] - joint_.getVelocity();          //目标 − 实际
        //double commanded_effort = pid_controller_.computeCommand(error, period);
        //joint_.setCommand(commanded_effort);                     

        if ((time - last_switch_).toSec() > 3.0) {
            target_velocity_ = (target_velocity_ == 0.0) ? 2.0 : 0.0;
            last_switch_ = time;
        }

        front_left_joint_.setCommand(
            front_left_pid_.computeCommand(target_velocity_ - front_left_joint_.getVelocity(), period));
        front_right_joint_.setCommand(
            front_right_pid_.computeCommand(target_velocity_ - front_right_joint_.getVelocity(), period));
        back_left_joint_.setCommand(
            back_left_pid_.computeCommand(target_velocity_ - back_left_joint_.getVelocity(), period));
        back_right_joint_.setCommand(
            back_right_pid_.computeCommand(target_velocity_ - back_right_joint_.getVelocity(), period));
    }

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
