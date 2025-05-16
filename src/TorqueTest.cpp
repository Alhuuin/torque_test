#include "TorqueTest.h"

TorqueTest::TorqueTest(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  const std::string &joint = "HY"; 

  torqueTask_ = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());
  torqueTask_->selectActiveJoints(solver(), {joint});
  solver().addTask(torqueTask_);
  
  const auto & robot = robots().robot();
  const auto & mb = robot.mb();
  
  auto joint_i = robot.jointIndexByName(joint);
  dof_i_ = mb.jointPosInDof(joint_i);

  torqueTask_->desiredTorque(joint, 1.0);

  std::vector<double> err(robot.refJointOrder().size(), 0);
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
                              err[i] = fabs(tauIn[i] - robot.mbc().jointTorque[static_cast<size_t>(mbcIndex)][0]);
                            }
                          }
                          return err;
                        }); 
  logger().addLogEntry(name_ + "_tauOut-tau_d", this, 
                        [this]()
                        {
                          Eigen::VectorXd test = (torqueTask_->currentTorques() - torqueTask_->desiredTorque()).cwiseAbs();
                          return test;
                        });

  datastore().make<std::string>("ControlMode", "Torque");
  mc_rtc::log::success("TorqueTest init done ");
}

bool TorqueTest::run()
{
  mc_rtc::log::info("Current torque on HY: {}", torqueTask_->currentTorques()(dof_i_));

  //return mc_control::fsm::Controller::run();
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoop);
}

void TorqueTest::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


