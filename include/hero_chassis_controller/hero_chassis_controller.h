// Copyright 2026 Bochuan Lin
// Created by chuan on 2026/11/8.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

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
        // 四个轮子：控制权的入口
        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;

        // 四个独立的 PID：每个轮子一个
        control_toolbox::Pid front_left_pid_;
        control_toolbox::Pid front_right_pid_;
        control_toolbox::Pid back_left_pid_;
        control_toolbox::Pid back_right_pid_;
};

}  // namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
