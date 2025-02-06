/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <Eigen/src/Core/Matrix.h>
#include <map>
#include <string>


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
  void addLog(mc_control::MCGlobalController & controller);

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~CurrentResidual() override;

private:
  int jointNumber;
  rbd::Coriolis * coriolis;
  rbd::ForwardDynamics forwardDynamics;
  double gear_ratio = 100.0;
  std::map<std::string, double> kt;
  Eigen::VectorXd tau_fric;
  Eigen::VectorXd integralTerm;
  Eigen::VectorXd pzero; //momentum_init
  Eigen::VectorXd residual;
  Eigen::VectorXd k_obs; //observer gain
  Eigen::MatrixXd inertiaMatrix;
};

} // namespace mc_plugin
