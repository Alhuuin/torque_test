#include "TorqueTest.h"

double TorqueTest::phase_ = 0.0;
double TorqueTest::period_ = 1.0;

TorqueTest::TorqueTest(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{  
  // joint_names_ = {
  //     "RCY", "RCR", "RCP", "RKP", "RAP", "RAR",
  //     "LCY", "LCR", "LCP", "LKP", "LAP", "LAR"
  // };

  joint_names_ = {
    "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE", "R_ANKLE_P", "R_ANKLE_R",
    "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE", "L_ANKLE_P", "L_ANKLE_R"
  };

  target_map_ = [&]{
    std::map<std::string, std::vector<double>> m;
    for (const auto& key : joint_names_) m[key] = {};
    return m;
  }();


  policy_path_ = config("policy_path", std::string("exported_model/actor_scripted.pt"));
  mc_rtc::log::success("Loading model from " + policy_path_);
  try {
    policy_ = torch::jit::load(policy_path_, torch::Device(torch::kCPU));
    mc_rtc::log::success("Model loaded successfully");

    warmUpPolicy(10);
  }
  catch (const c10::Error& e) {
    mc_rtc::log::error("Error loading the model: " + std::string(e.what()));
    return;
  }


  torqueTask_ = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());
  
  // torqueTask_->selectActiveJoints(solver(), joint_names_);
  solver().addTask(torqueTask_);

  add_logs();
  datastore().make<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("TorqueTest init done ");
}

bool TorqueTest::run()
{
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<float> observations;
  getObservations(observations);

  if (observations.size() != 44) {
    mc_rtc::log::error_and_throw("Expected 44 observations but got {}", observations.size());
  }

  std::vector<float> float_observations;
  float_observations.reserve(observations.size());
  for (const auto& obs : observations) {
    float_observations.push_back(static_cast<float>(obs));
  }
  torch::Tensor obs_tensor = torch::from_blob(float_observations.data(), {1, (long)float_observations.size()}, torch::kFloat32).clone();

  if (!applyPolicy(obs_tensor)) {
    return false;
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  double inference_time_ns = duration.count();
  double inference_time_hz = 1.0 / (inference_time_ns / 1e9);
  std::cout << "Inference time: " << inference_time_ns << " ns (" << inference_time_hz << " Hz)" << std::endl;

  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoop);
}

void TorqueTest::warmUpPolicy(int warmup_steps)
{
  std::vector<float> observations;
  getObservations(observations);

  if(observations.size() != 44)
  {
    mc_rtc::log::error("warmUpPolicy: Observations size is incorrect ({})", observations.size());
    return;
  }

  std::vector<float> float_observations(observations.begin(), observations.end());
  torch::Tensor obs_tensor = torch::from_blob(float_observations.data(), {1, (long)float_observations.size()}, torch::kFloat32).clone();

  for (int i = 0; i < warmup_steps; ++i)
  {
    try
    {
      std::vector<torch::jit::IValue> inputs = { obs_tensor };
      policy_.forward(inputs);
    }
    catch (const std::exception& e)
    {
      mc_rtc::log::error("warmUpPolicy failed at step {}: {}", i, e.what());
    }
  }

  mc_rtc::log::success("Policy warmed up with {} inference calls", warmup_steps);
}

inline void TorqueTest::getObservations(std::vector<float>& observations)
{
  // Base orientation
  auto base_orientation = robots().robot().posW().rotation();
  Eigen::Vector3d euler = base_orientation.eulerAngles(0, 1, 2);
  observations.push_back(euler[0]);  // root_r
  observations.push_back(euler[1]);  // root_p

  // Base angular velocity
  auto base_ang_vel = robot().velW().angular();
  observations.push_back(base_ang_vel[0]);
  observations.push_back(base_ang_vel[1]);
  observations.push_back(base_ang_vel[2]);

  // Joint positions, velocities, and torques (same logic)
  getJointData(observations, robots().robot().q(), "position");
  getJointData(observations, robots().robot().alpha(), "velocity");
  getJointData(observations, robots().robot().jointTorque(), "torque");

  // Clock signals
  phase_ += timeStep;
  if (phase_ >= period_) phase_ = 0;
  observations.push_back(std::sin(2 * M_PI * phase_ / period_));
  observations.push_back(std::cos(2 * M_PI * phase_ / period_));

  // Step in place flag
  observations.push_back(1);  // step_in_place = 1 for standing task
}

inline void TorqueTest::getJointData(std::vector<float>& observations, std::vector<std::vector<double>> joint_data, const std::string& type)
{
  for (const auto& joint : joint_names_) {
    try {
      unsigned int idx = robots().robot().jointIndexByName(joint);
      
      for (const auto& value : joint_data[idx]) {
        observations.push_back(static_cast<float>(value));
      }

    } catch (const std::exception& e) {
      mc_rtc::log::error("Failed to get index for joint {} ({}): {}", joint, type, e.what());
      
      // default value to maintain size consistency
      observations.push_back(0.0f);
    }
  }
}

inline bool TorqueTest::applyPolicy(const torch::Tensor& obs_tensor)
{
  try {
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_tensor);
    auto output = policy_.forward(inputs).toTensor();
    auto actions = output[0];

    mc_rtc::log::info("Policy output:");
    for (int i = 0; i < actions.size(0); ++i) {
      mc_rtc::log::info("  Action[{}] = {}", i, actions[i].item<float>());
    }

    if (torqueTask_) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        try {
          double torque = static_cast<double>(actions[i].item<float>());
          target_map_[joint_names_[i]] = {torque};
          mc_rtc::log::info("Joint {} (idx={}): torque={}", joint_names_[i], torque);
        } catch (const std::exception& e) {
          mc_rtc::log::error("Failed to apply torque for joint {}: {}", joint_names_[i], e.what());
          return false;
        }
      }

      try {
        torqueTask_->target(target_map_);

        mc_rtc::log::info("Actual torque:");
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          unsigned int idx = robots().robot().jointIndexByName(joint_names_[i]);
          mc_rtc::log::info("  {} = {}", joint_names_[i], robots().robot().mbc().jointTorque[idx]);
        }
      } catch (const std::exception& e) {
        mc_rtc::log::error("Failed to apply torque to TorqueTask: {}", e.what());
        return false;
      }
    } else {
      mc_rtc::log::error("TorqueTask not initialized");
      return false;
    }
  }
  catch (const c10::Error& e) {
    mc_rtc::log::error("Error during model forward pass: {}", e.what());
    return false;
  }
  catch (const std::exception& e) {
    mc_rtc::log::error("Error in applyPolicyAndTorque(): {}", e.what());
    return false;
  }

  return true;
}

void TorqueTest::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void TorqueTest::add_logs()
{
  std::vector<double> err(robots().robot().refJointOrder().size(), 0);
  logger().addLogEntry(name_ + "_tauIn-tauOut", this,
                        [this, err]() mutable -> const std::vector<double> &
                        {
                          auto & tauIn = robots().robot().jointTorques();
                          auto & robot = robots().robot();
                          for(size_t i = 0; i < err.size(); ++i)
                          {
                            auto mbcIndex = robot.jointIndexInMBC(i);
                            if(mbcIndex != -1)
                            {
                              err[i] = fabs((tauIn[i] - robot.mbc().jointTorque[static_cast<size_t>(mbcIndex)][0])/tauIn[i]);
                            }
                          }
                          return err;
                        }); 
  auto & err2 = err;
  logger().addLogEntry(name_ + "_tauOut-taud", this,
                        [this, err2]() mutable -> const std::vector<double> &
                        {
                          auto & robot = robots().robot();
                          auto tau_d = torqueTask_->torque();
                          for(size_t i = 0; i < err2.size(); ++i)
                          {
                            auto mbcIndex = robot.jointIndexInMBC(i);
                            if(mbcIndex != -1)
                            {
                              err2[i] = fabs((robot.mbc().jointTorque[static_cast<size_t>(mbcIndex)][0] - tau_d[static_cast<size_t>(mbcIndex)][0])/robot.mbc().jointTorque[static_cast<size_t>(mbcIndex)][0]);
                            }
                          }
                          return err2;
                        }); 
}
