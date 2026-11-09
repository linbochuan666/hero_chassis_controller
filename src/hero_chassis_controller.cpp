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
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
        return true;
    }

    void HeroChassisController::starting(const ros::Time&) {}

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
