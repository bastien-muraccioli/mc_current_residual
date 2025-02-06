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

  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

  auto plugin_config = config("current_residual");

  gear_ratio = plugin_config("gear_ratio", 100.0);
  k_obs = plugin_config("k_obs");
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
  residual_computation(controller);
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
  auto coriolisGravityTerm = forwardDynamics.C(); //Coriolis + Gravity term

  auto inertiaMatrix_prev = inertiaMatrix;
  inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  auto inertiaMatrix_dot = (inertiaMatrix - inertiaMatrix_prev)*ctl.timestep();
  auto pt = inertiaMatrix * qdot; //Momentum
  auto beta_regressor = coriolisGravityTerm - inertiaMatrix_dot * qdot + tau_fric;

  integralTerm += (tau_m - beta_regressor + residual) * ctl.timestep();

  residual = k_obs.asDiagonal() * (pt - integralTerm + pzero);
}

void CurrentResidual::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  ctl.controller().gui()->addElement({"Plugins", "CurrentResidual"},
                                     mc_rtc::gui::ArrayInput("k_obs", [this]() { return k_obs; },
                                                             [this](const Eigen::VectorXd & k) { k_obs = k; }));
}

void CurrentResidual::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("CurrentResidual_residual",
                                       [&, this]() { return this->residual; });
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CurrentResidual", mc_plugin::CurrentResidual)
