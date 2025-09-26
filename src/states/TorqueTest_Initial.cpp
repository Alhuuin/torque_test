#include "TorqueTest_Initial.h"

#include "../TorqueTest.h"

void TorqueTest_Initial::configure(const mc_rtc::Configuration & config)
{
}

void TorqueTest_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TorqueTest &>(ctl_);
}

bool TorqueTest_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TorqueTest &>(ctl_);
  // output("OK");
  return false;
}

void TorqueTest_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TorqueTest &>(ctl_);
}

EXPORT_SINGLE_STATE("TorqueTest_Initial", TorqueTest_Initial)