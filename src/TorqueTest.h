#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>
#include <cstddef>
#include <chrono>

#include <torch/csrc/jit/serialization/import.h>
#include <torch/custom_class.h>
#include <torch/script.h>


#include "api.h"

struct TorqueTest_DLLAPI TorqueTest : public mc_control::fsm::Controller
{
  TorqueTest(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void warmUpPolicy(int warmup_steps);
  void getObservations(std::vector<float>& observations);
  void getJointData(std::vector<float>& observations, std::vector<std::vector<double>> joint_data, const std::string& type);
  bool applyPolicy(const torch::Tensor& obs_tensor);

  void add_logs();

private:  
  torch::jit::script::Module policy_;
  std::vector<std::string> joint_names_;
  
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask_;
  Eigen::VectorXd joint_torques_;
  std::map<std::string, std::vector<double>> target_map_; 
  
  std::string policy_path_;
  double dt_;
  
  static double phase_;
  static double period_;
};