#include "TorqueTest.h"

TorqueTest::TorqueTest(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM) {

  // Add constraints
  contactConstraintTest =std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(contactConstraintTest);

  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(
          robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  // // Add the compliant posture task
  // compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(
  //     solver(), robot().robotIndex(), 1, 1);
  // compPostureTask->reset();
  // compPostureTask->weight(10.0);
  // // compPostureTask->stiffness(10.0);
  // solver().addTask(compPostureTask);

  // // Remove the default posture task
  // solver().removeTask(getPostureTask(robot().name()));
  mc_rtc::log::success("TorqueTest init done ");
}

bool TorqueTest::run()
{
  return mc_control::fsm::Controller::run();
}

void TorqueTest::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}