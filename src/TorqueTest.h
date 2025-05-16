#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>

#include "api.h"

struct TorqueTest_DLLAPI TorqueTest : public mc_control::fsm::Controller
{
  TorqueTest(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
private:
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask_;
  int dof_i_ = -1;
};