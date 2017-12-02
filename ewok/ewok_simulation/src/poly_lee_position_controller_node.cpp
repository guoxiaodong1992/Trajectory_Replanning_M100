
/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <std_msgs/Time.h>

#include "poly_lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {



PolyLeePositionControllerNode::PolyLeePositionControllerNode(
        ros::NodeHandle & nh): nh(nh) {

  ROS_INFO("Started PolyLeePositionControllerNode constructor");

  InitializeParams();


  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &PolyLeePositionControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

 cmd_traj_pub_=nh.advertise<nav_msgs::Odometry>("command_poly_trajectory_point",1);
 init_time_pub_=nh.advertise<std_msgs::Time>("init_time",1);

  ros::NodeHandle pnh("~");
  pnh.param("dt", dt, 0.5);

  poly_.reset(new ewok::PolynomialTrajectory3D<10>());


  ROS_INFO("Finished PolySplineLeePositionControllerNode constructor");

}
 PolyLeePositionControllerNode::~PolyLeePositionControllerNode() { }

void PolyLeePositionControllerNode::InitializeParams() {

  ROS_INFO("InitializeParams");

  ros::NodeHandle pnh("~");
  init_=false;

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();

  ROS_INFO("InitializeParams");
}

void PolyLeePositionControllerNode::SetTraj(ewok::PolynomialTrajectory3D<10>::Ptr traj) {
  ROS_INFO("Set Polynomial Treajectory");

  poly_=traj;
}

void PolyLeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionController got first odometry message.");
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);

  if(!init_)
  {
    init_=true;
    init_time=ros::Time::now();
    std_msgs::Time init_time_data;
    init_time_data.data=init_time;
    init_time_pub_.publish(init_time_data);
  }
  double local_t = (ros::Time::now() - init_time).toSec();



  bool yaw_from_traj;
  mav_msgs::EigenTrajectoryPoint command_trajectory;
  getTrajectoryPoint(local_t, command_trajectory, yaw_from_traj);
  if(!yaw_from_traj) {
    command_trajectory.setFromYaw(last_yaw);
    command_trajectory.setFromYawRate(0);
  } else {
    last_yaw = command_trajectory.getYaw();
  }


  lee_position_controller_.SetTrajectoryPoint(command_trajectory);

  //ROS_INFO_STREAM("Segment time: " << time_since_segment_start);
  //ROS_INFO_STREAM("Segment number: " << polynomial_segments_.front()->number());
  //ROS_INFO_STREAM("Number of segments: " << polynomial_segments_.size());
  //ROS_INFO_STREAM("Number of times: " << segment_start_times_.size());
  //ROS_INFO_STREAM("Setting point: " << command_trajectory.position_W.transpose());


  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}



void PolyLeePositionControllerNode::getTrajectoryPoint(double t,
      mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj) {


  command_trajectory.position_W = poly_->evaluate(t, 0);
  command_trajectory.velocity_W = poly_->evaluate(t, 1);
  command_trajectory.acceleration_W = poly_->evaluate(t, 2);
  cmd_odom.pose.pose.position.x=command_trajectory.position_W(0);
  cmd_odom.pose.pose.position.y=command_trajectory.position_W(1);
  cmd_odom.pose.pose.position.z=command_trajectory.position_W(2);
  cmd_odom.twist.twist.linear.x=command_trajectory.velocity_W(0);
  cmd_odom.twist.twist.linear.y=command_trajectory.velocity_W(1);
  cmd_odom.twist.twist.linear.z=command_trajectory.velocity_W(2);
  cmd_odom.header.stamp=ros::Time::now();
  cmd_traj_pub_.publish(cmd_odom);

  static const double eps = 0.1;
  static const double delta = 0.02;

  Eigen::Vector3d d_t = poly_->evaluate(t + eps, 0) - command_trajectory.position_W;

  yaw_from_traj = false;

  if(std::abs(d_t[0]) > delta || std::abs(d_t[1]) > delta) {
    double yaw = std::atan2(d_t[1], d_t[0]);
    yaw_from_traj = true;

    command_trajectory.setFromYaw(yaw);


    Eigen::Vector3d d_t_e = poly_->evaluate(t + 2*eps, 0) - poly_->evaluate(t + eps, 0);

    if(std::abs(d_t_e[0]) > delta || std::abs(d_t_e[1]) > delta) {
      double yaw_e = std::atan2(d_t_e[1], d_t_e[0]);
      double yaw_rate = (yaw_e - yaw) / eps;
      command_trajectory.setFromYawRate(yaw_rate);
    } else {
      command_trajectory.setFromYawRate(0);
    }

  }

}

}
