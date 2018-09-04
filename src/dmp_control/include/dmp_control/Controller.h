/**
 * Copyright (C) 2017 as64_
 */

#ifndef DMP_CONTROL_H
#define DMP_CONTROL_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>

#include <memory>
#include <random>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <string>
#include <thread>
#include <csignal>
#include <mutex>

#include <armadillo>
#include <Eigen/Dense>

#include <dmp_lib/dmp_lib.h>
#include <lwr_robot/lwr_model.h>
#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/robot_sim.h>
#include <autharl_core/robot/controller.h>

#define PACKAGE_NAME "dmp_control"

using namespace as64_;

class DMP_Control
{
public:
  DMP_Control();
  ~DMP_Control();

  void execute();

private:

  double k_d, d_d;
  double ko_d, do_d, kg;

  const int N_DOFS;
  arma::vec q_start;

  void initKuka();

  void initDMP();
  void trainDMP();

  bool loadTrainingData(std::string &err_msg);

  void gotoStartPose();

  arma::vec getTaskOrientation();

  ros::NodeHandle n;

  std::shared_ptr<arl::robot::Robot> robot;

  arma::rowvec Timed;
  arma::mat Yd_data, dYd_data, ddYd_data;

  arma::vec Y_robot;
  arma::vec Yo_robot;

  arma::vec q_robot;
  arma::vec q_robot_prev;
  arma::vec qdot_robot;

  arma::vec v_robot;

  std::vector<std::shared_ptr<DMP>> dmp;
  std::shared_ptr<CanonicalClock> can_clock_ptr;
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr;
  int N_kernels;
  double a_z, b_z;
  std::string train_method;

};

#endif // DMP_CONTROL_H
