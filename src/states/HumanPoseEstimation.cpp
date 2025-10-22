#include "HumanPoseEstimation.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/Robot.h>

#include <biRobotTeleop/HumanRobotPose.h>
#include <mc_joystick_plugin/joystick_inputs.h>

void HumanPoseEstimation::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  job_ = std::make_unique<HumanPoseEstimationJob>(ctl, config_, name());

  if(!job_->startedOnce())
  {
    auto & input = job_->input();
    input.init(ctl, config_, name());
  }

  const auto & humanRobot_name = job_->humanRobot_name_;

  // Add an external robot to display the output
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("simple_human");
  auto & human = ctl.external_robots_->load(humanRobot_name, *rm);
  mc_rtc::log::info("[{}] Loaded external robot \"{}\"", name(), human.name());

  // Initialize ros publisher
  mc_rtc::log::info("init robot publisher for {}", "control/" + humanRobot_name);
  mc_rtc::ROSBridge::init_robot_publisher("control/" + humanRobot_name, ctl.timeStep, human, false, true);

  ctl.getGUIBuilder().addElement({"BiRobotTeleop", "Estimated Human"},
                                 mc_rtc::gui::Robot(humanRobot_name,
                                                    [this, &ctl, humanRobot_name]() -> mc_rbdyn::Robot &
                                                    { return ctl.external_robots_->robot(humanRobot_name); }));

  ctl.gui()->addElement({"States", name()},
                        mc_rtc::gui::Robot(humanRobot_name, [this, &ctl, humanRobot_name]() -> const mc_rbdyn::Robot &
                                           { return ctl.external_robots_->robot(humanRobot_name); }));

  run(ctl_);
}

bool HumanPoseEstimation::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  if(!job_->running())
  {
    // initialize input when no job is running
    auto & input = job_->input();
    input.syncState(ctl, job_->human_indx_);

    // Job starts as an async task, use job.checkResult() later to know whether it is finished and retrive its value
    job_->startAsync();
    job_->addToLogger(ctl_.logger(), name());
    // job_->addToGUI(*ctl_.gui(), guiCategory_);
  }
  else if(job_->checkResult())
  {
    const auto & result = *job_->lastResult();
    ctl.updateHumanPose(result.h_estimated_, ctl.getHumanPose(job_->human_indx_, true));

    // copy human robot state for display and publishing to external controller
    auto & threadRobot = job_->ext_robots->robot();
    auto & human = ctl.external_robots_->robot(job_->humanRobot_name_);
    human.mbc() = threadRobot.mbc();
    human.forwardKinematics();
    human.forwardVelocity();
    human.forwardAcceleration();
    // mc_rtc::log::info("Update robot publisher \"control/{}\"", job_->humanRobot_name_);
    mc_rtc::ROSBridge::update_robot_publisher("control/" + job_->humanRobot_name_, ctl.timeStep, human);
  }

  output("True");
  return false;
}

// void HumanPoseEstimation::addLog(mc_control::fsm::Controller & ctl_)
// {
//   ctl_.logger().addLogEntry("perf_" + name() + "_solver", [this]() -> const double { return solving_perf_; });
// }
//
// void HumanPoseEstimation::addGUI(mc_control::fsm::Controller & ctl_) {}

void HumanPoseEstimation::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  auto & robots = ctl.external_robots_;
  const int indx = robots->robotIndex(job_->humanRobot_name_);
  robots->removeRobot(indx);
}

EXPORT_SINGLE_STATE("HumanPoseEstimation", HumanPoseEstimation)
