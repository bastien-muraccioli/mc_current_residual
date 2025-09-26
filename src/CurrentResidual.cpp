#include "CurrentResidual.h"

#include <mc_control/GlobalPluginMacros.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_plugin
{

CurrentResidual::~CurrentResidual() = default;

void CurrentResidual::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  auto & robot = ctl.robot(ctl.robots()[0].name());
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  auto & rjo = robot.refJointOrder();

  dt_ = ctl.timestep();
  counter_ = 0.0;
  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

  // Make sure to have obstacle detection
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }

  ctl.controller().datastore().make<bool>("Current Residual Obstacle detected", false);
  
  auto plugin_config = config("current_residual");

  gear_ratio = plugin_config("gear_ratio", 100.0);
  k_obs = plugin_config("k_obs", 10.0);
  kt = plugin_config("kt");
  if(kt.empty())
  {
    kt = {
      {"joint_1", 0.11}, 
      {"joint_2", 0.11}, 
      {"joint_3", 0.11},
      {"joint_4", 0.11}, 
      {"joint_5", 0.076}, 
      {"joint_6", 0.076},
      {"joint_7", 0.076}
    };
  }
  threshold_filtering_ = plugin_config("threshold_filtering", 0.05);
  threshold_offset_ = plugin_config("threshold_offset");
  if(threshold_offset_.size() != jointNumber)
  {
    threshold_offset_ = Eigen::VectorXd::Constant(jointNumber, 10.0);
  }
  lpf_threshold_.setValues(threshold_offset_, threshold_filtering_, jointNumber);

  // Get and print tau fric from datastore
  if(ctl.controller().datastore().has("torque_fric"))
  {
    tau_fric = ctl.controller().datastore().get<Eigen::VectorXd>("torque_fric");
    for(int i = 0; i < jointNumber; i++)
    {
      mc_rtc::log::info("[CurrentResidual] Joint {} has friction torque {} N.m", rjo[i], tau_fric[i]);
    }
  }
  else
  {
    tau_fric.setZero(jointNumber);
    mc_rtc::log::error("[CurrentResidual] No torque_fric in datastore");
  }

  // For key in kt, get the value of the current joint
  for(auto const& [key, val] : kt)
  {
    double tau_mot = val * gear_ratio * realRobot.jointJointSensor(key).motorCurrent();
    mc_rtc::log::info("[CurrentResidual] Joint {} has kt value {}, motor torque current based {} N.m", key, val, tau_mot);
  }
  inertiaMatrix.resize(jointNumber, jointNumber);
  Eigen::VectorXd qdot(jointNumber);
  for(size_t i = 0; i < jointNumber; i++)
  {
    qdot[i] = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
  }
  residual.setZero(jointNumber);
  residual_high_.setZero(jointNumber);
  residual_low_.setZero(jointNumber);
  integralTerm.setZero(jointNumber);

  coriolis = new rbd::Coriolis(robot.mb());
  forwardDynamics = rbd::ForwardDynamics(robot.mb());

  forwardDynamics.computeH(robot.mb(), robot.mbc());
  inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  pzero = inertiaMatrix * qdot;

  mc_rtc::log::info("[CurrentResidua] inertiaMatrix: {}, pzero: {}", inertiaMatrix, pzero);

  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::info("CurrentResidual::init called with configuration:\n{}", config.dump(true, true));
}

void CurrentResidual::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CurrentResidual::reset called");
}

void CurrentResidual::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  counter_ += dt_;

  if(activate_plot_ && !plot_added_)
  {
    addPlot(controller);
    plot_added_ = true;
  }

  // if(ctl.controller().datastore().has("Zurlo Collision Detection"))
  // {
  //   collision_stop_activated_zurlo_ = ctl.controller().datastore().get<bool>("Zurlo Collision Detection");
  // }

  residual_computation(controller);
  residual_high_ = lpf_threshold_.adaptiveThreshold(residual, true);
  residual_low_ = lpf_threshold_.adaptiveThreshold(residual, false);
  obstacle_detected_ = false;
  for (int i = 0; i < jointNumber; i++)
  {
    if (residual[i] > residual_high_[i] || residual[i] < residual_low_[i])
    {
      obstacle_detected_ = true;
      if(activate_verbose) mc_rtc::log::info("[Current Residual] Obstacle detected on joint {}", i);
      if (collision_stop_activated_)
      {
        ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
      }
      break;
    }
  }
  // if (collision_stop_activated_zurlo_)
  // {
    ctl.controller().datastore().get<bool>("Current Residual Obstacle detected") = obstacle_detected_;
  // }
}

void CurrentResidual::after(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("CurrentResidual::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CurrentResidual::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}


void CurrentResidual::residual_computation(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  if(ctl.robot().encoderVelocities().empty())
  {
    return;
  }

  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  auto & rjo = realRobot.refJointOrder();

  // Get and print tau fric from datastore
  if(ctl.controller().datastore().has("torque_fric"))
  {
    tau_fric = ctl.controller().datastore().get<Eigen::VectorXd>("torque_fric");
  }
  else
  {
    tau_fric.setZero(jointNumber);
    mc_rtc::log::error("[CurrentResidual] No torque_fric in datastore");
  }

  Eigen::VectorXd tau_m = Eigen::VectorXd::Zero(jointNumber);

  int jointIndex = 0;
  for(auto const& [key, val] : kt)
  {
      if (jointIndex >= 0 && jointIndex < jointNumber) {
          // Calculate motor torque and assign it to tau_m
          double tau_mot = val * gear_ratio * realRobot.jointJointSensor(key).motorCurrent();
          tau_m[jointIndex] = tau_mot;
      } else {
          mc_rtc::log::error("[CurrentResidual] Invalid joint name: {} or index out of bounds", key);
      }
      jointIndex++;
  }


  Eigen::VectorXd qdot(jointNumber);
  rbd::paramToVector(realRobot.alpha(), qdot);
 
  forwardDynamics.computeC(realRobot.mb(), realRobot.mbc());
  forwardDynamics.computeH(realRobot.mb(), realRobot.mbc());
  auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
  auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g

  auto inertiaMatrix_prev = inertiaMatrix;
  inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  auto inertiaMatrix_dot = (inertiaMatrix - inertiaMatrix_prev)*ctl.timestep();
  auto pt = inertiaMatrix * qdot; //Momentum
  auto beta_regressor = coriolisGravityTerm - inertiaMatrix_dot * qdot + tau_fric;

  integralTerm += (tau_m - beta_regressor + residual) * ctl.timestep();

  residual = k_obs * Eigen::MatrixXd::Identity(jointNumber, jointNumber) * (pt - integralTerm - pzero);
  if(!ctl.controller().datastore().has("current_residual"))
  {
    ctl.controller().datastore().make<Eigen::VectorXd>("current_residual", residual);
  }
  else
  {
    ctl.controller().datastore().assign("current_residual", residual);
  }
}

void CurrentResidual::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  ctl.controller().gui()->addElement({"Plugins", "CurrentResidual"},
    mc_rtc::gui::NumberInput("k_obs", [this]() { return k_obs; },
      [this](double k) 
      {
        this->integralTerm.setZero();
        this->residual.setZero();
        this->k_obs = k;
      }),
    mc_rtc::gui::IntegerInput("Residual shown", [this]() { return residual_shown_; },
      [this](int r) 
      {
        this->residual_shown_ = r;
      }),
    mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }),
    // Add checkbox to activate the collision stop
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
    mc_rtc::gui::Checkbox("Verbose", activate_verbose), 
    // Add Threshold offset input
    mc_rtc::gui::ArrayInput("Threshold offset", {"q_0", "q_1", "q_2", "q_3", "q_4", "q_5", "q_6"}, 
      [this](){return this->threshold_offset_;},
        [this](const Eigen::VectorXd & offset)
      { 
        threshold_offset_ = offset;
        lpf_threshold_.setOffset(threshold_offset_); 
      }),
    // Add Threshold filtering input
    mc_rtc::gui::NumberInput("Threshold filtering", [this](){return this->threshold_filtering_;},
        [this](double filtering)
      { 
        threshold_filtering_ = filtering;
        lpf_threshold_.setFiltering(threshold_filtering_); 
      })                                               
    );
}

void CurrentResidual::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("CurrentResidual_residual",
                                       [&, this]() { return this->residual; });
  ctl.controller().logger().addLogEntry("CurrentResidual_residual_high",
                                       [&, this]() { return this->residual_high_; });
  ctl.controller().logger().addLogEntry("CurrentResidual_residual_low",
                                       [&, this]() { return this->residual_low_; });
  ctl.controller().logger().addLogEntry("CurrentResidual_obstacleDetected",
                                       [&, this]() { return this->obstacle_detected_; });
}

void CurrentResidual::addPlot(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & gui = *ctl.controller().gui();

  gui.addPlot(
      "CurrentResidual",
      mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
      mc_rtc::gui::plot::Y("residual", [this]() { return residual_high_[residual_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y("residual", [this]() { return residual_low_[residual_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y("residual", [this]() { return residual[residual_shown_]; }, mc_rtc::gui::Color::Red)
  );
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CurrentResidual", mc_plugin::CurrentResidual)
