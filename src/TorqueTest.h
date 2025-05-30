#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>
#include <cstddef>
#include <chrono>

#include <torch/csrc/jit/serialization/import.h>
#include <torch/custom_class.h>
#include <torch/script.h>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <filesystem> 

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
  std::unordered_map<std::string, size_t> joint_indices_;

  std::unordered_map<int,int> test;
  std::vector<float> test2;
  
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask_;
  Eigen::VectorXd joint_torques_;
  std::map<std::string, std::vector<double>> target_map_; 
  
  std::string policy_path_;
  double dt_;

  std::vector<double> current_action_;
  
  std::vector<float> last_action_;
  std::mutex action_mutex_;
  std::thread policy_thread_;
  std::atomic<bool> policy_running_{false};
  std::chrono::steady_clock::time_point last_policy_time_;
  double policy_dt_{0.005}; // Default to 200Hz
  torch::Tensor obs_tensor_;
  
  static double phase_;
  static double period_;
};