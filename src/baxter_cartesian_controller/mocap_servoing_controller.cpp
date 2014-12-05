#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <urdf_model/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <baxter_cartesian_controller/mocap_servoing_controller.hpp>
#include <baxter_cartesian_controller/eigen_pinv.hpp>

using namespace baxter_mocap_servoing;

MocapServoingController::MocapServoingController(ros::NodeHandle& nh, std::string group_name, std::string arm_pose_topic, std::string target_pose_topic, std::string robot_config_topic, std::string arm_command_action, std::string arm_controller_state_topic, std::string abort_service, double kp, double ki, double kd) : nh_(nh)
{
    // Set mode
    mode_ = EXTERNAL_POSE;
    // Set up an internal robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    baxter_model_ = robot_model_loader.getModel();
    baxter_kinematic_state_ = robot_model::RobotStatePtr(new robot_state::RobotState(baxter_model_));
    baxter_kinematic_state_->setToDefaultValues();
    baxter_kinematic_state_->update();
    if (group_name == std::string("left_arm"))
    {
        side_ = LEFT;
        baxter_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(baxter_model_->getJointModelGroup(group_name));
        baxter_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("left_hand")));
        baxter_base_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("base")));
        // Set the joint names
        joint_names_.resize(BAXTER_ARM_JOINTS);
        joint_names_[0] = "left_s0";
        joint_names_[1] = "left_s1";
        joint_names_[2] = "left_e0";
        joint_names_[3] = "left_e1";
        joint_names_[4] = "left_w0";
        joint_names_[5] = "left_w1";
        joint_names_[6] = "left_w2";
    }
    else if (group_name == std::string("right_arm"))
    {
        side_ = RIGHT;
        baxter_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(baxter_model_->getJointModelGroup(group_name));
        baxter_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("right_hand")));
        baxter_base_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("base")));
        // Set the joint names
        joint_names_.resize(BAXTER_ARM_JOINTS);
        joint_names_[0] = "right_s0";
        joint_names_[1] = "right_s1";
        joint_names_[2] = "right_e0";
        joint_names_[3] = "right_e1";
        joint_names_[4] = "right_w0";
        joint_names_[5] = "right_w1";
        joint_names_[6] = "right_w2";
    }
    else
    {
        throw std::invalid_argument("Invalid group name");
    }
    // Setup topics
    arm_pose_sub_ = nh_.subscribe(arm_pose_topic, 1, &MocapServoingController::ArmPoseCB, this);
    target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &MocapServoingController::TargetPoseCB, this);
    robot_config_sub_ = nh_.subscribe(robot_config_topic, 1, &MocapServoingController::RobotStateCB, this);
    arm_controller_state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(arm_controller_state_topic, 1, true);
    // Setup abort service
    abort_server_ = nh_.advertiseService(abort_service, &MocapServoingController::AbortCB, this);
    // Setup trajectory controller interface
    arm_client_ = std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_command_action, true));
    ROS_INFO("Waiting for arm controllers to come up...");
    arm_client_->waitForServer();
    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    // Set max size for qdot
    max_joint_correction_ = MAXIMUM_JOINT_CORRECTION;
    // Set execution timestep
    execution_timestep_ = EXECUTION_INTERVAL;
    // Set timeout
    watchdog_timeout_ = WATCHDOG_INTERVAL;
    // Initialize the control variables to safe values
    arm_pose_valid_ = false;
    arm_config_valid_ = false;
    target_pose_valid_ = false;
    // Initialize the PID values to zero
    pose_error_integral_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    last_pose_error_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Start in PAUSED mode
    state_ = PAUSED;
}

MocapServoingController::MocapServoingController(ros::NodeHandle &nh, std::string group_name, std::string target_pose_topic, std::string robot_config_topic, std::string arm_command_action, std::string arm_controller_state_topic, std::string abort_service, double kp, double ki, double kd) : nh_(nh)
{
    // Set mode
    mode_ = INTERNAL_POSE;
    // Set up an internal robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    baxter_model_ = robot_model_loader.getModel();
    baxter_kinematic_state_ = robot_model::RobotStatePtr(new robot_state::RobotState(baxter_model_));
    baxter_kinematic_state_->setToDefaultValues();
    baxter_kinematic_state_->update();
    if (group_name == std::string("left_arm"))
    {
        side_ = LEFT;
        baxter_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(baxter_model_->getJointModelGroup(group_name));
        baxter_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("left_hand")));
        baxter_base_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("base")));
        // Set the joint names
        joint_names_.resize(BAXTER_ARM_JOINTS);
        joint_names_[0] = "left_s0";
        joint_names_[1] = "left_s1";
        joint_names_[2] = "left_e0";
        joint_names_[3] = "left_e1";
        joint_names_[4] = "left_w0";
        joint_names_[5] = "left_w1";
        joint_names_[6] = "left_w2";
    }
    else if (group_name == std::string("right_arm"))
    {
        side_ = RIGHT;
        baxter_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(baxter_model_->getJointModelGroup(group_name));
        baxter_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("right_hand")));
        baxter_base_link_ = std::unique_ptr<const robot_model::LinkModel>(baxter_kinematic_state_->getLinkModel(std::string("base")));
        // Set the joint names
        joint_names_.resize(BAXTER_ARM_JOINTS);
        joint_names_[0] = "right_s0";
        joint_names_[1] = "right_s1";
        joint_names_[2] = "right_e0";
        joint_names_[3] = "right_e1";
        joint_names_[4] = "right_w0";
        joint_names_[5] = "right_w1";
        joint_names_[6] = "right_w2";
    }
    else
    {
        throw std::invalid_argument("Invalid group name");
    }
    // Setup topics
    target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &MocapServoingController::TargetPoseCB, this);
    robot_config_sub_ = nh_.subscribe(robot_config_topic, 1, &MocapServoingController::RobotStateCB, this);
    arm_controller_state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(arm_controller_state_topic, 1, true);
    // Setup abort service
    abort_server_ = nh_.advertiseService(abort_service, &MocapServoingController::AbortCB, this);
    // Setup trajectory controller interface
    arm_client_ = std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_command_action, true));
    ROS_INFO("Waiting for arm controllers to come up...");
    arm_client_->waitForServer();
    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    // Set max size for qdot
    max_joint_correction_ = MAXIMUM_JOINT_CORRECTION;
    // Set execution timestep
    execution_timestep_ = EXECUTION_INTERVAL;
    // Set timeout
    watchdog_timeout_ = WATCHDOG_INTERVAL;
    // Initialize the control variables to safe values
    arm_pose_valid_ = false;
    arm_config_valid_ = false;
    target_pose_valid_ = false;
    // Initialize the PID values to zero
    pose_error_integral_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    last_pose_error_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Start in PAUSED mode
    state_ = PAUSED;
}

std::vector<double> MocapServoingController::ComputeNextStep(Pose& current_arm_pose, Pose& current_target_pose, std::vector<double>& current_configuration)
{
    // Get the current jacobian
    Eigen::MatrixXd current_jacobian = ComputeJacobian(current_configuration);
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current Jacobian: " << current_jacobian << std::endl;
#endif
    // Compute the pose error in our 'world frame'
    Twist pose_error = ComputePoseError(current_arm_pose, current_target_pose);
    // Compute the integral of pose error & update the stored value
    pose_error_integral_ = pose_error_integral_ + (pose_error * CONTROL_INTERVAL);
    // Compute the derivative of pose error
    Twist pose_error_derivative = (pose_error - last_pose_error_) / CONTROL_INTERVAL;
    // Update the stored pose error
    last_pose_error_ = pose_error;
    // Convert pose errors into cartesian velocity
    Twist pose_correction = (pose_error * kp_) + (pose_error_integral_ * ki_) + (pose_error_derivative * kd_);
    // Use the Jacobian pseudoinverse
    Eigen::VectorXd joint_correction = EIGEN_PINV::pinv(current_jacobian, 0.001) * pose_correction;
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current raw joint correction: " << joint_correction << std::endl;
#endif
    // Bound qdot to max magnitude of 0.05
    double joint_correction_magnitude = joint_correction.norm();
    if (joint_correction_magnitude > max_joint_correction_)
    {
        joint_correction = (joint_correction / joint_correction_magnitude) * max_joint_correction_;
    }
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current limited joint correction: " << joint_correction << std::endl;
#endif
    // Combine the joint correction with the current configuration to form the target configuration
    std::vector<double> target_configuration(BAXTER_ARM_JOINTS);
    target_configuration[0] = current_configuration[0] + (joint_correction[0] * SHOULDER_0_DAMPING) + SHOULDER_0_OFFSET;
    target_configuration[1] = current_configuration[1] + (joint_correction[1] * SHOULDER_1_DAMPING) + SHOULDER_1_OFFSET;
    target_configuration[2] = current_configuration[2] + (joint_correction[2] * ELBOW_0_DAMPING) + ELBOW_0_OFFSET;
    target_configuration[3] = current_configuration[3] + (joint_correction[3] * ELBOW_1_DAMPING) + ELBOW_1_OFFSET;
    target_configuration[4] = current_configuration[4] + (joint_correction[4] * WRIST_0_DAMPING) + WRIST_0_OFFSET;
    target_configuration[5] = current_configuration[5] + (joint_correction[5] * WRIST_1_DAMPING) + WRIST_1_OFFSET;
    target_configuration[6] = current_configuration[6] + (joint_correction[6] * WRIST_2_DAMPING) + WRIST_2_OFFSET;
    std::cout << "Current configuration: " << PrettyPrint(current_configuration, true) << std::endl;
    std::cout << "New target configuration: " << PrettyPrint(target_configuration, true) << std::endl;
    return target_configuration;
}

void MocapServoingController::Loop()
{
    ros::Rate spin_rate(CONTROL_RATE);
    while (ros::ok())
    {
        // Do the next step
        if (state_ == RUNNING)
        {
            // Publish the current arm pose
            Eigen::Vector3d current_arm_position = current_arm_pose_.translation();
            Eigen::Quaterniond current_arm_orientation(current_arm_pose_.rotation());
            geometry_msgs::PoseStamped current_arm_pose_stamped;
            current_arm_pose_stamped.header.stamp = ros::Time::now();
            current_arm_pose_stamped.header.frame_id = "/base";
            current_arm_pose_stamped.pose.position.x = current_arm_position.x();
            current_arm_pose_stamped.pose.position.y = current_arm_position.y();
            current_arm_pose_stamped.pose.position.z = current_arm_position.z();
            current_arm_pose_stamped.pose.orientation.x = current_arm_orientation.x();
            current_arm_pose_stamped.pose.orientation.y = current_arm_orientation.y();
            current_arm_pose_stamped.pose.orientation.z = current_arm_orientation.z();
            current_arm_pose_stamped.pose.orientation.w = current_arm_orientation.w();
            arm_controller_state_pub_.publish(current_arm_pose_stamped);
            // Compute the next step
            std::vector<double> target_config = ComputeNextStep(current_arm_pose_, current_target_pose_, current_arm_config_);
            // Command the robot
            CommandToTarget(current_arm_config_, target_config);
        }
        // Process callbacks
        ros::spinOnce();
        // Spin
        spin_rate.sleep();
    }
}

void MocapServoingController::CommandToTarget(std::vector<double>& current_config, std::vector<double>& target_config)
{
    control_msgs::FollowJointTrajectoryGoal command;
    // Populate command
    command.trajectory.joint_names = joint_names_;
    command.trajectory.header.stamp = ros::Time::now();
    // Populate start point (unused)
    trajectory_msgs::JointTrajectoryPoint start_point;
    start_point.positions = current_config;
    start_point.velocities.resize(start_point.positions.size(), 0.0);
    start_point.time_from_start = ros::Duration(0.0);
    // Populate target point
    trajectory_msgs::JointTrajectoryPoint target_point;
    target_point.positions = target_config;
    target_point.velocities.resize(target_point.positions.size(), 0.0);
    // Set the execution time
    target_point.time_from_start = ros::Duration(execution_timestep_);
    // Add point
    command.trajectory.points.push_back(target_point);
    // Command the arm
    arm_client_->sendGoal(command);
}
