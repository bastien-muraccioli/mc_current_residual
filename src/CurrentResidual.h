/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <Eigen/src/Core/Matrix.h>
#include <map>
#include <string>
#include "LpfThreshold.h"


#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

namespace mc_plugin
{

struct CurrentResidual : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  void residual_computation(mc_control::MCGlobalController & controller);

  void addGui(mc_control::MCGlobalController & controller);
  void addPlot(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~CurrentResidual() override;

private:
  double dt_;
  double counter_;

  int jointNumber;
  int residual_shown_ = 0;
  rbd::Coriolis * coriolis;
  rbd::ForwardDynamics forwardDynamics;
  double gear_ratio = 100.0;
  std::map<std::string, double> kt;
  Eigen::VectorXd tau_fric;
  Eigen::VectorXd integralTerm;
  Eigen::VectorXd pzero; //momentum_init
  Eigen::VectorXd residual;
  double k_obs; //observer gain
  Eigen::MatrixXd inertiaMatrix;

  LpfThreshold lpf_threshold_;
  Eigen::VectorXd threshold_offset_;
  double threshold_filtering_;
  Eigen::VectorXd residual_high_;
  Eigen::VectorXd residual_low_;

  bool activate_plot_ = false;
  bool plot_added_ = false;
  bool collision_stop_activated_ = false;
  // bool collision_stop_activated_zurlo_ = false;
  bool obstacle_detected_ = false;
  bool activate_verbose = false;
};

} // namespace mc_plugin
