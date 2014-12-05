#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <baxter_cartesian_controller/mocap_servoing_controller.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baxter_left_arm_mocap_servoing_controller");
    ROS_INFO("Starting baxter_left_arm_mocap_servoing_controller...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string arm_pose_topic;
    std::string target_pose_topic;
    std::string robot_config_topic;
    std::string arm_command_action;
    std::string abort_service;
    double kp = DEFAULT_KP;
    double ki = DEFAULT_KI;
    double kd = DEFAULT_KD;
    nhp.param(std::string("arm_pose_topic"), arm_pose_topic, std::string("/left_arm_pose_controller/pose"));
    nhp.param(std::string("target_pose_topic"), target_pose_topic, std::string("/left_arm_pose_controller/target"));
    nhp.param(std::string("robot_config_topic"), robot_config_topic, std::string("/robot/joint_states"));
    nhp.param(std::string("arm_command_action"), arm_command_action, std::string("/robot/limb/left/follow_joint_trajectory"));
    nhp.param(std::string("abort_service"), abort_service, std::string("/left_arm_pose_controller/abort"));
    nhp.param(std::string("kp"), kp, DEFAULT_KP);
    nhp.param(std::string("ki"), ki, DEFAULT_KI);
    nhp.param(std::string("kd"), kd, DEFAULT_KD);
    if (arm_pose_topic == std::string(""))
    {
        ROS_INFO("Running in INTERNAL_POSE mode");
        baxter_mocap_servoing::MocapServoingController controller(nh, std::string("left_arm"), target_pose_topic, robot_config_topic, arm_command_action, abort_service, kp, ki, kd);
        ROS_INFO("...startup complete");
        controller.Loop();
    }
    else
    {
        ROS_INFO("Running in EXTERNAL_POSE mode");
        baxter_mocap_servoing::MocapServoingController controller(nh, std::string("left_arm"), arm_pose_topic, target_pose_topic, robot_config_topic, arm_command_action, abort_service, kp, ki, kd);
        ROS_INFO("...startup complete");
        controller.Loop();
    }
    return 0;
}
