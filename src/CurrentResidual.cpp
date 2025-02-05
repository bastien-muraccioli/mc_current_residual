#include "CurrentResidual.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CurrentResidual::~CurrentResidual() = default;

void CurrentResidual::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("CurrentResidual::init called with configuration:\n{}", config.dump(true, true));
}

void CurrentResidual::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CurrentResidual::reset called");
}

void CurrentResidual::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("CurrentResidual::before");
}

void CurrentResidual::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CurrentResidual::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CurrentResidual::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CurrentResidual", mc_plugin::CurrentResidual)
