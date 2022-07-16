/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include <nav_msgs/Odometry.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

#include "convert.h"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

template <typename TLCM>
void *update_loop(void *param) {
  TLCM *data = (TLCM *)param;
  while (ros::ok) {
    data->Recv();
    usleep(2000);
  }
}

template <typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm) {
  std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
            << "Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  // SetLevel(HIGHLEVEL);
  long motiontime = 0;
  TCmd SendHighLCM = {0};
  TState RecvHighLCM = {0};
  unitree_legged_msgs::HighCmd SendHighROS;
  unitree_legged_msgs::HighState RecvHighROS;
  sensor_msgs::Imu body_imu;
  sensor_msgs::JointState body_jointstate;
  trajectory_msgs::MultiDOFJointTrajectory body_footstate;
  nav_msgs::Odometry body_odom;

  body_jointstate.name = {
      "FR0", "FR1", "FR2", "FL0", "FL1",     "FL2",     "RR0",     "RR1",
      "RR2", "RL0", "RL1", "RL2", "FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  body_jointstate.position.assign(16, 0);
  body_jointstate.velocity.assign(16, 0);
  body_jointstate.effort.assign(16, 0);

  body_footstate.joint_names = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  body_footstate.points.resize(4);
  for (int i = 0; i < 4; i++) {
    body_footstate.points[i].transforms.resize(1);
    body_footstate.points[i].velocities.resize(1);
    body_footstate.points[i].accelerations.resize(1);
  }

  ros::Publisher pub_highstate =
      n.advertise<unitree_legged_msgs::HighState>("/unitree/robot_state", 1000);
  ros::Publisher pub_jointstate =
      n.advertise<sensor_msgs::JointState>("/unitree/joint_state", 1000);
  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("/unitree/imu", 1000);
  ros::Publisher pub_footstate =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "/unitree/foot_state", 1000);
  ros::Publisher pub_bodyodom =
      n.advertise<nav_msgs::Odometry>("/unitree/body_odom", 1000);

  roslcm.SubscribeState();
  pthread_t tid;
  pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

  while (ros::ok()) {
    motiontime = motiontime + 2;
    roslcm.Get(RecvHighLCM);
    RecvHighROS = ToRos(RecvHighLCM);

    RecvHighROS.header.stamp = ros::Time::now();

    body_imu.header.stamp = ros::Time::now();
    body_imu.linear_acceleration.x = RecvHighROS.imu.accelerometer[0];
    body_imu.linear_acceleration.y = RecvHighROS.imu.accelerometer[1];
    body_imu.linear_acceleration.z = RecvHighROS.imu.accelerometer[2];
    body_imu.angular_velocity.x = RecvHighROS.imu.gyroscope[0];
    body_imu.angular_velocity.y = RecvHighROS.imu.gyroscope[1];
    body_imu.angular_velocity.z = RecvHighROS.imu.gyroscope[2];
    body_imu.orientation.x = RecvHighROS.imu.quaternion[0];
    body_imu.orientation.y = RecvHighROS.imu.quaternion[1];
    body_imu.orientation.z = RecvHighROS.imu.quaternion[2];
    body_imu.orientation.w = RecvHighROS.imu.quaternion[3];

    body_jointstate.header.stamp = ros::Time::now();
    for (int i = 0; i < 12; i++) {
      body_jointstate.position[i] = RecvHighROS.motorState[i].q;
      body_jointstate.velocity[i] = RecvHighROS.motorState[i].dq;
      body_jointstate.effort[i] = RecvHighROS.motorState[i].tauEst;
    }

    for (int i = 0; i < 4; i++)
      body_jointstate.effort[12 + i] = RecvHighROS.footForce[i];

    body_footstate.header.stamp = ros::Time::now();
    for (int i = 0; i < 4; i++) {
      body_footstate.points[i].transforms[0].translation.x =
          RecvHighROS.footPosition2Body[i].x;
      body_footstate.points[i].transforms[0].translation.y =
          RecvHighROS.footPosition2Body[i].y;
      body_footstate.points[i].transforms[0].translation.z =
          RecvHighROS.footPosition2Body[i].z;
      body_footstate.points[i].velocities[0].linear.x =
          RecvHighROS.footSpeed2Body[i].x;
      body_footstate.points[i].velocities[0].linear.y =
          RecvHighROS.footSpeed2Body[i].y;
      body_footstate.points[i].velocities[0].linear.z =
          RecvHighROS.footSpeed2Body[i].z;
    }

    body_odom.header.stamp = ros::Time::now();
    body_odom.pose.pose.position.x = RecvHighROS.position[0];
    body_odom.pose.pose.position.y = RecvHighROS.position[1];
    body_odom.pose.pose.position.z = RecvHighROS.position[2];   
    body_odom.twist.twist.linear.x = RecvHighROS.velocity[0];
    body_odom.twist.twist.linear.y = RecvHighROS.velocity[1];
    body_odom.twist.twist.linear.z = RecvHighROS.velocity[2];
    body_odom.twist.twist.angular.z = RecvHighROS.yawSpeed;   //rad/s

    printf("Max Tempurature of Calf Joint: %d %d %d %d\n",
           RecvHighROS.motorState[2].temperature,
           RecvHighROS.motorState[5].temperature,
           RecvHighROS.motorState[8].temperature,
           RecvHighROS.motorState[11].temperature);
    if(RecvHighROS.motorState[2].temperature >=55 || RecvHighROS.motorState[5].temperature >=55 || RecvHighROS.motorState[8].temperature >=55 || RecvHighROS.motorState[11].temperature >=55)       
    ROS_WARN("Your dog will be over-temperature, please take off it immediately!!!!!!");
    // SendHighROS.velocity[0] = 0.0f;    //   forwardspeed
    // SendHighROS.velocity[1]  = 0.0f;   //   sidespeed
    // SendHighROS.yawSpeed = 0.0f;
    // SendHighROS.bodyHeight = 0.0f;

    // SendHighROS.mode = 0;
    // SendHighROS.euler[0]  = 0;       //  roll
    // SendHighROS.euler[1] = 0;        //  pitch
    // SendHighROS.euler[2] = 0;        //  yaw

    SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    roslcm.Send(SendHighLCM);

    pub_highstate.publish(RecvHighROS);
    pub_imu.publish(body_imu);
    pub_jointstate.publish(body_jointstate);
    pub_footstate.publish(body_footstate);
    pub_bodyodom.publish(body_odom);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "walk_ros_mode");
  std::string firmwork;
  ros::param::get("/firmwork", firmwork);

#ifdef SDK3_1
  aliengo::Control control(aliengo::HIGHLEVEL);
  aliengo::LCM roslcm;
  mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv,
                                                                 roslcm);
#endif

#ifdef SDK3_2
  std::string robot_name;
  UNITREE_LEGGED_SDK::LeggedType rname;
  ros::param::get("/robot_name", robot_name);
  if (strcasecmp(robot_name.c_str(), "A1") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
  else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

  // UNITREE_LEGGED_SDK::InitEnvironment();
  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState,
             UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
#endif
}