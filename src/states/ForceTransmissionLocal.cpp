#include "ForceTransmissionLocal.h"

#include "../BiRobotTeleoperation.h"
#include <mc_joystick_plugin/joystick_inputs.h>
#include <sch/CD/CD_Pair.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>

void ForceTransmissionLocal::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);
  config("force_activation_threshold", force_activation_threshold_);
  config("distance_activation_threshold", distance_activation_threshold_);

  config("deactivation_threshold", deactivation_threshold_);

  // robot_ = config("robot", robots.robot(rIndex).name());

  if(config.has("exportValue"))
  {
    auto exportValueConfig = config("exportValue");
    if(exportValueConfig.has("exportContactWrench"))
    {
      exportValueConfig("exportContactWrench", exportContactWrench_);
    }
    if(exportValueConfig.has("exportExternalWrench"))
    {
      exportValueConfig("exportExternalWrench", exportExternalWrench_);
    }
    if(exportValueConfig.has("usingWrench"))
    {
      exportValueConfig("usingWrench", usingWrench_);
      mc_rtc::log::info("[ForceTransmissionLocal] usingWrench_: {}", usingWrench_);
    }
  }
  else
  {
    mc_rtc::log::error("[ForceTransmissionLocal] No exportValue is specified in the config file");
  }
}

void ForceTransmissionLocal::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  dt_ = ctl_.timeStep;

  config_("robot_1")("force_sensor_limbs", force_sensor_limbs_robot_1_);
  config_("robot_2")("force_sensor_limbs", force_sensor_limbs_robot_2_);

  human_1_estimated_ = &ctl.external_robots_->robot("human_1_estimated");
  human_2_estimated_ = &ctl.external_robots_->robot("human_2_estimated");

  for(int _ = 0; _ < force_sensor_limbs_robot_1_.size(); _++)
  {
    activation_force_measurements_robot_1_.push_back(mc_filter::LowPass<sva::ForceVecd>(dt_, 0.5));
    robot_1_force_activation_.push_back(false);
  }
  for(int _ = 0; _ < force_sensor_limbs_robot_2_.size(); _++)
  {
    activation_force_measurements_robot_2_.push_back(mc_filter::LowPass<sva::ForceVecd>(dt_, 0.5));
    robot_2_force_activation_.push_back(false);
  }

  for(int it = biRobotTeleop::Head; it <= biRobotTeleop::RightArm; it++)
  {
    biRobotTeleop::Limbs limb = static_cast<biRobotTeleop::Limbs>(it);
    closests_points_robot_2_[it] = sva::PTransformd::Identity();
    closests_points_robot_1_[it] = sva::PTransformd::Identity();
  }

  dt_ = ctl.timeStep;

  addGUI(ctl_);
  addLog(ctl_);

  auto a = ctl_.datastore().keys();
  for(std::string key : a)
  {
    std::cout << key << std::endl;
  }

  for(int i = 0; i < 6; i++)
  {
    accumulator_1_.push_back(accumulator_t(ba::tag::rolling_window::window_size = 100));
    accumulator_2_.push_back(accumulator_t(ba::tag::rolling_window::window_size = 100));
  }
}

bool ForceTransmissionLocal::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  output("OK");

  // for (int i=0; i< closests_points_robot_1_.size(); i++){
  //   mc_rtc::log::info("key {} and address {}, ", i,(void*)&closests_points_robot_1_[i]);
  // }  // TODO problem with closests_points

  if(!active_) // if not active, look if the force sensors measure a contact or if distance is close enough
  {
    checkActivationTest(ctl_, 1);
    if(!active_)
    {
      checkActivationTest(ctl_, 2);
    }
    return true;
  }

  std::cout << " just checking the distance guys ! \n";
  double d_b = getContactDistance(ctl_, indx_, limb_a_, limb_b_).norm(); // robot A human B
  double d_a = getContactDistance(ctl_, indx_ == 1 ? 2 : 1, limb_b_, limb_a_).norm(); // robot B human A

  if(active_force_measurement_ != nullptr) // filtered wrench value on robot A's limb (limb_a_)
  {
    active_force_measurement_->update(task_a_->frame().wrench()); // adds new measurement to the lowpass vector
  }
  // if((active_force_measurement_->eval().vector().norm() < force_activation_threshold_ ||
  //     d_a > deactivation_threshold_) && !activation_enforced_)
  if((d_a > deactivation_threshold_))
  {
    mc_rtc::log::info("[{}] Contact has been broken, deactivate force control\nd_a {} d_b {}", name(), d_a, d_b);
    auto & frame = task_b_->frame();
    ctl.solver().removeTask(task_a_);
    ctl.solver().removeTask(task_b_);
    if(active_force_measurement_ != nullptr) // filtered wrench value on robot A's limb (limb_a_)
    {
      active_force_measurement_->reset(sva::ForceVecd::Zero());
      active_force_measurement_ = nullptr;
    }
    contact_limb_ = "None";
    indx_ = 0;
    // activation_enforced_ = false;
    active_ = false;
    return true;
  }

  // sva::ForceVecd measured_wrench_a;
  // sva::ForceVecd measured_wrench_b;

  const biRobotTeleop::HumanPose & h_b = ctl.getHumanPose((indx_ == 1) ? 1 : 0);
  const biRobotTeleop::HumanPose & h_a = ctl.getHumanPose((indx_ == 1) ? 0 : 1);

  /// A REVOIR
  const sva::PTransformd X_f_contactF_a =
      h_b.getOffset(limb_b_) * h_b.getPose(limb_b_)
      * task_a_->frame().position().inv(); // transformation between limb b of human and the limb of task a
  const sva::PTransformd X_f_contactF_b =
      h_a.getOffset(limb_a_) * h_a.getPose(limb_a_)
      * task_b_->frame().position().inv(); // transformation between human limb a and the limb of task b

  // mc_rtc::log::info("getOffsetGetPose \n{} \n frame position \n{}", task_a_->frame().position())

  if(task_a_->frame().hasForceSensor() && task_a_->frame().forceSensor().name() != robot_a_custom_force_sensor_name_)
  {
    measured_wrench_a = task_a_->frame().wrench();

    // w = ctl_.robots().robot(robot_name).frame(f).wrench(); la mm chose ??
    if(activation_enforced_)
    {
      std::cout << "what are you doing here bro??\n";
      auto indx = ctl.robot(robot_a_name_).data()->forceSensorsIndex["LeftHandForceSensor"];
      // minus between frame and force sensor
      ctl.robot(robot_a_name_)
          .data()
          ->forceSensors[indx]
          .wrench(sva::ForceVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d{0, 0, 20}));
    }
  }
  else
  {
    mc_rtc::log::info("robot {} does not have a force sensor ", robot_a_name_);

    // if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in
    // the mbc at the fb (floating base ?)

    const mc_rbdyn::Robot & robot_a = ctl.robot(robot_a_name_);
    const auto X_0_fb = ctl.robot(robot_a_name_).posW();

    getestimatedExternalWrench(ctl_, robot_a_name_, indx_);
    calculateMovingAverage(indx_);

    const auto X_0_centroid = worldCentroidKinePTrans_[indx_ - 1];

    const auto R_fb_limb_b =
        sva::PTransformd(h_a.getOffset(limb_b_) * h_a.getPose(limb_b_).rotation())
        * sva::PTransformd(X_0_fb.rotation()).inv(); // TRANSFORMATION BW limb a of human and fb of robot b

    const auto R_centroid_limb_b =
        sva::PTransformd(h_b.getOffset(limb_b_) * h_b.getPose(limb_b_).rotation())
        * sva::PTransformd(X_0_centroid.rotation()).inv(); // TRANSFORMATION BW limb a and fb of robot b

    const auto X_0_frame = task_a_->frame().position();

    auto wrench_fb = ctl.getCalibratedExtWrench(ctl.realRobot(robot_a_name_));
    auto wrench_fb_centroid = estimatedExternalWrench_centroid_without_bias_[indx_ - 1];

    measured_wrench_a =
        (X_f_contactF_a.inv() * R_fb_limb_b).dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), wrench_fb.force()));

    measured_wrench_a_centroid =
        (X_f_contactF_a.inv() * R_centroid_limb_b)
            .dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), wrench_fb_centroid.force()));

    measured_wrench_a_centroid_trasnform =
        transformExternalWrench(estimatedExternalWrench_centroid_without_bias_[indx_ - 1], limb_b_, indx_, X_0_frame);

    // if(activation_enforced_)
    // {
    //   const sva::ForceVecd fake_wrench = 0*sva::ForceVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d{0,-10,-10});
    //   measured_wrench_b = (X_0_frame * X_0_fb.inv()).dualMul( fake_wrench  );
    // }

    mc_rtc::log::info("robot a is {}, measured wrench at fb is \n {}\nat centroid \n{}\n with centroid transform\n{}",
                      robot_a_name_, measured_wrench_a, measured_wrench_a_centroid,
                      measured_wrench_a_centroid_trasnform);

    const auto fs_indx = robot_a.data()->forceSensorsIndex.at(robot_a_custom_force_sensor_name_);
    robot_a.data()->forceSensors[fs_indx].wrench(measured_wrench_a);
  }

  mc_rtc::log::info("HELLO guysss ^^");
  if(task_b_->frame().hasForceSensor() && task_b_->frame().forceSensor().name() != robot_b_custom_force_sensor_name_)
  {
    measured_wrench_b = task_b_->frame().wrench();

    mc_rtc::log::info("robot {} has a force sensor named {}", robot_b_name_, task_b_->frame().forceSensor().name());
  }
  else
  {
    mc_rtc::log::info("robot {} does not have a force sensor ", robot_b_name_);

    // if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in
    // the mbc at the fb (floating base ?)
    getestimatedExternalWrench(ctl_, robot_b_name_, indx_ == 2 ? 1 : 2);
    calculateMovingAverage(indx_ == 2 ? 1 : 2);

    const mc_rbdyn::Robot & robot_b = ctl.robot(robot_b_name_);
    const auto X_0_fb = ctl.robot(robot_b_name_).posW();

    const auto X_0_centroid = worldCentroidKinePTrans_[indx_ == 2 ? 0 : 1];

    const auto R_fb_limb_a =
        sva::PTransformd(h_a.getOffset(limb_a_) * h_a.getPose(limb_a_).rotation())
        * sva::PTransformd(X_0_fb.rotation()).inv(); // TRANSFORMATION BW limb a of human and fb of robot b

    const auto R_centroid_limb_a =
        sva::PTransformd(h_a.getOffset(limb_a_) * h_a.getPose(limb_a_).rotation())
        * sva::PTransformd(X_0_centroid.rotation()).inv(); // TRANSFORMATION BW limb a and fb of robot b

    const auto X_0_frame = task_b_->frame().position();

    auto wrench_fb = ctl.getCalibratedExtWrench(ctl.realRobot(robot_b_name_));

    auto wrench_fb_centroid = estimatedExternalWrench_centroid_without_bias_[indx_ == 2 ? 0 : 1];

    measured_wrench_b = (X_f_contactF_b.inv() * R_fb_limb_a)
                            .dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(),
                                                    wrench_fb.force())); // tester de mettre la transfo plus simplement
    // transformation between the limb of task b and limb a of human  * transformation (only rot) bw limb a and fb of robot b

    // h_a.getOffset(limb_a_) * h_a.getPose(limb_a_) * task_b_->frame().position().inv();

    measured_wrench_b_centroid =
        (X_f_contactF_b.inv() * R_centroid_limb_a)
            .dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), wrench_fb_centroid.force()));

    measured_wrench_b_centroid_trasnform =
        transformExternalWrench(estimatedExternalWrench_centroid_without_bias_[indx_ == 2 ? 0 : 1], limb_b_,
                                indx_ == 2 ? 1 : 2, task_b_->frame().position());

    // if(activation_enforced_)
    // {
    //   const sva::ForceVecd fake_wrench = 0*sva::ForceVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d{0,-10,-10});
    //   measured_wrench_b = (X_0_frame * X_0_fb.inv()).dualMul( fake_wrench  );
    // }

    mc_rtc::log::info("robot b is {}, measured wrench at fb is \n {}\nat centroid \n{}\n with centroid transform\n{}",
                      robot_b_name_, measured_wrench_b, measured_wrench_b_centroid,
                      measured_wrench_b_centroid_trasnform);

    const auto fs_indx = robot_b.data()->forceSensorsIndex.at(robot_b_custom_force_sensor_name_);
    robot_b.data()->forceSensors[fs_indx].wrench(measured_wrench_b);

    // ctl_.robots().robot(robot_name).frame(f).wrench(); // this is where i am getting the wrench for now

    // task_b_->setMeasuredWrench(measured_wrench_b);
  }

  // mc_rtc::log::info("human_a is {}\nX_f_contactF_b trans {}",h_a.name(),X_f_contactF_b.translation());

  const biRobotTeleop::RobotPose & robot_a_pose = indx_ == 1 ? ctl.r_1_ : ctl.r_2_;
  const biRobotTeleop::RobotPose & robot_b_pose = indx_ == 2 ? ctl.r_1_ : ctl.r_2_;

  const sva::ForceVecd targetWrench_b =
      (X_f_contactF_b.inv() * robot_a_pose.getOffset(limb_a_))
          .dualMul(-measured_wrench_a); // transformation between  limb a and the limb of task b

  const sva::ForceVecd targetWrench_a = (X_f_contactF_a.inv() * robot_b_pose.getOffset(limb_b_))
                                            .dualMul(-measured_wrench_b); // measured wrench b at frame b
  // but moved to

  task_a_->targetWrench(targetWrench_a);
  task_b_->targetWrench(targetWrench_b);

  return true;
}

/// @brief Adds the force damping tasks to the robots, with robot A being the one checked, that is : if (1) the human
/// controlling robot A is close to the robot B it's interacting with, (2) the (filtered) force measured on one of the
/// robot A's limbs that has a force sensor is over a certain threshold, or (3) a boolean condition in the
/// force_activation vector
/// @param ctl_
/// @param robot_indx
/// @return
bool ForceTransmissionLocal::checkActivation(mc_control::fsm::Controller & ctl_, const int robot_indx)
{
  // mc_rtc::log::info("[CheckActivation] for robot {} ", robot_indx);

  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const std::string robot_name = "robot_" + std::to_string(robot_indx);
  const biRobotTeleop::HumanPose h_a = robot_indx == 1 ? ctl.hp_1_ : ctl.hp_2_;
  const biRobotTeleop::RobotPose & robot_a_pose = robot_indx == 1 ? ctl.r_1_ : ctl.r_2_;
  const biRobotTeleop::RobotPose & robot_b_pose = robot_indx == 1 ? ctl.r_2_ : ctl.r_1_;
  int filter_indx = 0; // sort of an iterator over the fs limbs
  const std::vector<std::string> fs_limbs = robot_indx == 1 ? force_sensor_limbs_robot_1_ : force_sensor_limbs_robot_2_;
  std::vector<mc_filter::LowPass<sva::ForceVecd>> & activation_force_measurements =
      robot_indx == 1 ? activation_force_measurements_robot_1_ : activation_force_measurements_robot_2_;

  std::vector<bool> & force_activation = robot_indx == 1 ? robot_1_force_activation_ : robot_2_force_activation_;

  getestimatedExternalWrench(ctl_, robot_name, robot_indx);
  calculateMovingAverage(robot_indx);

  // mc_rtc::log::info("[CheckActivation] Wrench without bias on robot {} is \n {}", robot_name,
  //                   estimatedExternalWrench_centroid_without_bias_[robot_indx - 1]);

  for(auto & l : fs_limbs) // prpblemaic the fs limbs  for(int int_limb = 1; int_limb <= biRobotTeleop::Limbs::RightArm;
                           // int_limb++)
  {
    const auto fs_limb_a = biRobotTeleop::str2Limb(l);
    const auto f = robot_indx == 1 ? ctl.r_1_.getName(fs_limb_a) : ctl.r_2_.getName(fs_limb_a);
    auto w = ctl_.robots().robot(robot_name).frame(f).wrench();

    // transformExternalWrench
    // w=estimatedExternalWrench_centroid_without_bias_[robot_indx-1];

    activation_force_measurements[filter_indx].update(w); // update the low pass filter corresponding to the fs limb

    double min_d = 1e9;
    for(int i = 1; i <= biRobotTeleop::Limbs::RightArm; i++)
    {
      const auto limb_i = static_cast<biRobotTeleop::Limbs>(i);
      // const auto frame_b_i = robot_b_pose.getName(limb_i);
      const double d = getContactDistance(ctl_, robot_indx == 1 ? 2 : 1, limb_i, fs_limb_a)
                           .norm(); // contact distance for the pair h/r B and the considered limbs
      min_d = std::min(d, min_d);
    } // loops on the other pair B and gets the closest distance between the fs limb (here of the human controlling
      // robot A) and all the robot limbs

    // plusieurs conditions :
    // la force mesuree sur robot A (apres filtre) est plus grande que le threshold
    // la paire B est suffisemment proche
    // force_activation[filter_indx]  (jai pas ecnore compris a quoi ca servait)
    if(min_d < distance_activation_threshold_ || force_activation[filter_indx]
       || activation_force_measurements[filter_indx].eval().vector().norm() > force_activation_threshold_
       || estimatedExternalWrench_centroid_without_bias_[robot_indx - 1].vector().norm() > force_activation_threshold_)
    {
      activation_enforced_ = force_activation[filter_indx]; // pas onblige d etre a true.... j ai limpression au il
                                                            // change pas de valeur ???

      // Once a force sensor is in contact, we set the limb in contact and activate the force task;
      mc_rtc::log::info("[{}] contact measured on frame {}, adding task.. Force is {}, distance is {}, force "
                        "activation is {}, centroid force is {}",
                        name(), f,
                        activation_force_measurements[filter_indx].eval().vector().norm() > force_activation_threshold_,
                        min_d<distance_activation_threshold_, force_activation[filter_indx],
                              estimatedExternalWrench_centroid_without_bias_[robot_indx - 1].vector().norm()>
                            force_activation_threshold_);

      task_a_ = std::make_shared<mc_tasks::force::DampingTask>(ctl.robots().robot(robot_name).frame(f)); // a revoir
      task_a_->load(ctl.solver(), config_(robot_name)("task"));
      task_a_->velFilterGain(0.9);
      task_a_->name(task_a_->name() + "_a");
      limb_a_ = fs_limb_a;
      ctl.solver().addTask(task_a_);
      active_force_measurement_ =
          &activation_force_measurements[filter_indx]; // the wrench value, with the lowpass filter
      indx_ = robot_indx;
      robot_a_name_ = robot_name;

      mc_rbdyn::Robot & robot_a = ctl.robots().robot(robot_a_name_);

      const std::string link_a = robot_indx == 1 ? ctl.r_1_.getName(limb_a_) : ctl.r_2_.getName(limb_a_);
      robot_a_custom_force_sensor_name_ = robot_a_name_ + "_" + biRobotTeleop::limb2Str(limb_a_);
      if(!robot_a.bodyHasForceSensor(link_a))
      {
        mc_rbdyn::ForceSensor sensor =
            mc_rbdyn::ForceSensor(robot_a_custom_force_sensor_name_, link_a, sva::PTransformd::Identity());
        robot_a.addForceSensor(sensor); // on met le faux force sensor (utile pour la main fonction (run))
      }

      robot_b_name_ = (robot_a_name_ == "robot_1") ? "robot_2" : "robot_1";

      limb_b_ = getContactLimb(ctl, robot_indx,
                               fs_limb_a); // returns the limb of the human that is closest to the robot limb specified
                                           // for the pair A (human limb close to the fs limb of robot A)
      mc_rbdyn::Robot & robot_b = ctl.robots().robot(robot_b_name_);
      contact_limb_ = biRobotTeleop::limb2Str(
          limb_b_); // it will have to be the limb that robot B moves to be in contact with his human
      const std::string link_b = robot_indx == 1 ? ctl.r_2_.getName(limb_b_) : ctl.r_1_.getName(limb_b_);
      robot_b_custom_force_sensor_name_ = robot_b_name_ + "_" + contact_limb_;
      if(!robot_b.bodyHasForceSensor(link_b))
      {
        mc_rbdyn::ForceSensor sensor =
            mc_rbdyn::ForceSensor(robot_b_custom_force_sensor_name_, link_b, sva::PTransformd::Identity());
        robot_b.addForceSensor(sensor); // on met le faux force sensor (utile pour la main fonction (run))
      }

      task_b_ = std::make_shared<mc_tasks::force::DampingTask>(robot_b.frame(link_b));
      task_b_->load(ctl.solver(), config_(robot_b_name_)("task"));
      task_b_->velFilterGain(0.9);
      task_b_->name(task_b_->name() + "_b");
      ctl.solver().addTask(task_b_);

      task_robot_1_ = robot_indx == 1 ? task_a_ : task_b_;
      task_robot_2_ = robot_indx == 2 ? task_a_ : task_b_;

      active_ = true;
      mc_rtc::log::info("[{}] robot_a is {} robot_b is {}", name(), robot_a_name_, robot_b_name_);
      mc_rtc::log::info("[{}] task limb_a is {} task limb_b is {}", name(), biRobotTeleop::limb2Str(limb_a_),
                        biRobotTeleop::limb2Str(limb_b_));
      return true;
    }
    filter_indx += 1;
  }

  if(!active_)
  {
    addDataToAverage(estimatedExternalWrench_centroid_[robot_indx - 1], robot_indx);
    mc_rtc::log::info("adding data to average");
  }

  return false;
}

bool ForceTransmissionLocal::checkActivationTest(mc_control::fsm::Controller & ctl_, const int robot_indx)
{
  mc_rtc::log::info("[CheckActivation] for robot {} ", robot_indx);

  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const std::string robot_name = "robot_" + std::to_string(robot_indx);
  const biRobotTeleop::HumanPose h_a = robot_indx == 1 ? ctl.hp_1_ : ctl.hp_2_;
  const biRobotTeleop::RobotPose & robot_a_pose = robot_indx == 1 ? ctl.r_1_ : ctl.r_2_;
  const biRobotTeleop::RobotPose & robot_b_pose = robot_indx == 1 ? ctl.r_2_ : ctl.r_1_;
  int filter_indx = 0; // sort of an iterator over the fs limbs
  const std::vector<std::string> fs_limbs = robot_indx == 1 ? force_sensor_limbs_robot_1_ : force_sensor_limbs_robot_2_;
  for(auto & l : fs_limbs)
  {
    mc_rtc::log::info("fs limbs {}", l);
  }
  std::vector<mc_filter::LowPass<sva::ForceVecd>> & activation_force_measurements =
      robot_indx == 1 ? activation_force_measurements_robot_1_ : activation_force_measurements_robot_2_;

  std::vector<bool> & force_activation = robot_indx == 1 ? robot_1_force_activation_ : robot_2_force_activation_;

  getestimatedExternalWrench(ctl_, robot_name, robot_indx);
  calculateMovingAverage(robot_indx);

  // mc_rtc::log::info("[CheckActivation] Wrench without bias on robot {} is \n {}", robot_name,
  //                   estimatedExternalWrench_centroid_without_bias_[robot_indx - 1]);

  // for(auto & l : fs_limbs) // prpblemaic the fs limbs
  for(int int_limb = 1; int_limb <= biRobotTeleop::Limbs::RightArm; int_limb++)
  {

    const auto limb_a = static_cast<biRobotTeleop::Limbs>(int_limb);
    if(limb_a == biRobotTeleop::Limbs::Pelvis || limb_a == biRobotTeleop::Limbs::Head)
    {
      continue;
    }

    const auto f = robot_indx == 1 ? ctl.r_1_.getName(limb_a) : ctl.r_2_.getName(limb_a);
    bool is_fs = std::find(fs_limbs.begin(), fs_limbs.end(), biRobotTeleop::limb2Str(limb_a)) != fs_limbs.end();
    if(is_fs)
    {
      auto w = ctl_.robots().robot(robot_name).frame(f).wrench();
      activation_force_measurements[filter_indx].update(w); // update the low pass filter corresponding to the fs limb

      active_force_measurement_ =
          &activation_force_measurements[filter_indx]; // the wrench value, with the lowpass filter
    }

    double min_d = 1e9;
    for(int i = 1; i <= biRobotTeleop::Limbs::RightArm; i++)
    {
      const auto limb_i = static_cast<biRobotTeleop::Limbs>(i);
      if(limb_i == biRobotTeleop::Limbs::Pelvis || limb_i == biRobotTeleop::Limbs::Head)
      {
        continue;
      }
      // const auto frame_b_i = robot_b_pose.getName(limb_i);
      const double d = getContactDistance(ctl_, robot_indx == 1 ? 2 : 1, limb_i, limb_a)
                           .norm(); // contact distance for the pair h/r B and the considered limbs
      min_d = std::min(d, min_d);
    } // loops on the other pair B and gets the closest distance between the fs limb (here of the human controlling
      // robot A) and all the robot limbs

    mc_rtc::log::info("[{}] on frame {},  distance is {} centroid force is {}", name(), f, min_d,
                      estimatedExternalWrench_centroid_without_bias_[robot_indx - 1].vector().norm());

    // plusieurs conditions :
    // la force mesuree sur robot A (apres filtre) est plus grande que le threshold
    // la paire B est suffisemment proche
    // force_activation[filter_indx]  (jai pas ecnore compris a quoi ca servait)
    if(min_d < distance_activation_threshold_
       || (is_fs && activation_force_measurements[filter_indx].eval().vector().norm() > force_activation_threshold_)
       || estimatedExternalWrench_centroid_without_bias_[robot_indx - 1].vector().norm()
              > force_activation_threshold_) //|| (activation_force_measurements[filter_indx].eval().vector().norm() >
                                             // force_activation_threshold_ && is_fs)
    {
      // activation_enforced_ = force_activation[filter_indx]; // pas onblige d etre a true.... j ai limpression au il
      // change pas de valeur ???

      // Once a force sensor is in contact, we set the limb in contact and activate the force task;
      mc_rtc::log::info("[{}] contact measured on frame {}, adding task..  distance is {}, centroid force is {}",
                        name(), f,
                        min_d<distance_activation_threshold_,
                              estimatedExternalWrench_centroid_without_bias_[robot_indx - 1].vector().norm()>
                            force_activation_threshold_);

      indx_ = robot_indx;
      robot_a_name_ = robot_name;
      mc_rbdyn::Robot & robot_a = ctl.robots().robot(robot_a_name_);
      robot_a_custom_force_sensor_name_ = robot_a_name_ + "_" + biRobotTeleop::limb2Str(limb_a_);
      if(!robot_a.bodyHasForceSensor(f))
      {
        mc_rbdyn::ForceSensor sensor =
            mc_rbdyn::ForceSensor(robot_a_custom_force_sensor_name_, f, sva::PTransformd::Identity());
        robot_a.addForceSensor(
            sensor); // on met le faux force sensor (utile pour la main fonction (run)) et pr la damping task
      }

      task_a_ = std::make_shared<mc_tasks::force::DampingTask>(ctl.robots().robot(robot_name).frame(f)); // a revoir
      task_a_->load(ctl.solver(), config_(robot_name)("task"));
      task_a_->velFilterGain(0.9);
      task_a_->name(task_a_->name() + "_a");
      limb_a_ = limb_a;
      ctl.solver().addTask(task_a_);

      robot_b_name_ = (robot_a_name_ == "robot_1") ? "robot_2" : "robot_1";

      limb_b_ = getContactLimb(ctl, robot_indx,
                               limb_a); // returns the limb of the human that is closest to the robot limb specified
                                        // for the pair A (human limb close to the fs limb of robot A)
      mc_rbdyn::Robot & robot_b = ctl.robots().robot(robot_b_name_);
      contact_limb_ = biRobotTeleop::limb2Str(
          limb_b_); // it will have to be the limb that robot B moves to be in contact with his human
      const std::string link_b = robot_indx == 1 ? ctl.r_2_.getName(limb_b_) : ctl.r_1_.getName(limb_b_);
      robot_b_custom_force_sensor_name_ = robot_b_name_ + "_" + contact_limb_;
      if(!robot_b.bodyHasForceSensor(link_b))
      {
        mc_rbdyn::ForceSensor sensor =
            mc_rbdyn::ForceSensor(robot_b_custom_force_sensor_name_, link_b, sva::PTransformd::Identity());
        robot_b.addForceSensor(sensor); // on met le faux force sensor (utile pour la main fonction (run))
      }

      task_b_ = std::make_shared<mc_tasks::force::DampingTask>(robot_b.frame(link_b));
      task_b_->load(ctl.solver(), config_(robot_b_name_)("task"));
      task_b_->velFilterGain(0.9);
      task_b_->name(task_b_->name() + "_b");
      ctl.solver().addTask(task_b_);

      task_robot_1_ = robot_indx == 1 ? task_a_ : task_b_;
      task_robot_2_ = robot_indx == 2 ? task_a_ : task_b_;

      active_ = true;
      mc_rtc::log::info("[{}] robot_a is {} robot_b is {}", name(), robot_a_name_, robot_b_name_);
      mc_rtc::log::info("[{}] task limb_a is {} task limb_b is {}", name(), biRobotTeleop::limb2Str(limb_a_),
                        biRobotTeleop::limb2Str(limb_b_));
      return true;
    }
    filter_indx += 1;
  }

  if(!active_)
  {
    addDataToAverage(estimatedExternalWrench_centroid_[robot_indx - 1], robot_indx);
  }

  return false;
}

/// @brief Given a robot (1 or 2), finds its human's closest limb to this robot's limb (specified)
/// @return limb of the human that is closest to the robot limb specified
const biRobotTeleop::Limbs ForceTransmissionLocal::getContactLimb(mc_control::fsm::Controller & ctl_,
                                                                  const int robot_indx,
                                                                  const biRobotTeleop::Limbs & robot_limb)
{

  biRobotTeleop::Limbs output_limb = biRobotTeleop::Head;
  double min_d = 1e9;

  for(int int_limb = 1; int_limb <= biRobotTeleop::Limbs::RightArm; int_limb++)
  {
    const auto limb = static_cast<biRobotTeleop::Limbs>(int_limb);
    if(limb == biRobotTeleop::Limbs::Pelvis || limb == biRobotTeleop::Limbs::Head)
    {
      continue;
    }

    const auto d =
        getContactDistance(ctl_, robot_indx, robot_limb, limb).norm(); // gets distance for the r/h pair and the limbs

    if(d < min_d)
    {
      output_limb = limb;
      min_d = d;
    }
  }
  mc_rtc::log::info("[getContactLimb] robot checked is robot_{},min distance is {}", robot_indx, min_d);
  return output_limb;
}

/// @brief Given a robot (1 or 2), computes the distance between this robot's limb and the limb of the human it's
/// interacting with. Also registers the closest point on the robot for this limb
const Eigen::Vector3d ForceTransmissionLocal::getContactDistance(mc_control::fsm::Controller & ctl_,
                                                                 const int robot_indx,
                                                                 const biRobotTeleop::Limbs limb_robot,
                                                                 const biRobotTeleop::Limbs limb_human)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const std::string robot_name = "robot_" + std::to_string(robot_indx);
  const auto & robot = ctl.robot(robot_name);
  const auto rp = robot_indx == 1 ? ctl.r_1_ : ctl.r_2_; // robot pose
  const auto h = ctl.getHumanPose(robot_indx == 1 ? 1 : 0); // c'est human 1 et robot 2 par exemple
  // paire H/R en interaction

  auto human_cvx = h.getConvex(limb_human, *human_1_estimated_);

  if(robot_indx == 1)
  {
    human_cvx = h.getConvex(limb_human, *human_2_estimated_);
  }

  const auto link_name = rp.getName(limb_robot);

  const auto & robot_cvx = robot.convex(rp.getConvexName(limb_robot));

  sch::CD_Pair pair_limb_frame(human_cvx.get(), robot_cvx.second.get());

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1, p2);

  Eigen::Vector3d robot_point;
  robot_point << p2.m_x, p2.m_y, p2.m_z;

  sva::PTransformd X_0_robot_link = rp.getOffset(limb_robot) * robot.bodyPosW(link_name);

  // mc_rtc::log::info("limb \n{}   size of map {}", limb_robot, closests_points_robot_1_.size() );
  // for (int i=0; i< closests_points_robot_2_.size(); i++){

  //   mc_rtc::log::info("key {} and address {}, ", i,(void*)&closests_points_robot_1_[i]);
  // }

  if(robot_indx == 1)
  {
    // closests_points_robot_1_[limb_robot] =
    //     sva::PTransformd(X_0_robot_link.rotation(), robot_point) * X_0_robot_link.inv();
  }
  else if(robot_indx == 2)
  {
    // closests_points_robot_2_[limb_robot] =
    //     sva::PTransformd(X_0_robot_link.rotation(), robot_point) * X_0_robot_link.inv();
  }

  Eigen::Vector3d out;
  out << p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2];
  return out;
}

// sva::ForceVecd ForceTransmissionLocal::transformExternalWrench(const sva::ForceVecd wrench,
//                                                                const biRobotTeleop::Limbs limb_robot,
//                                                                int rIndex)
// {
//   sva::PTransformd X_0_surface = closests_points_robot_1_[limb_robot]; // ^surface X_0
//   if(rIndex == 2)
//   {
//     X_0_surface = closests_points_robot_2_[limb_robot];
//   }

//   // task_b_->frame().position();
//   sva::PTransformd X_0_centroid = worldCentroidKinePTrans_[rIndex - 1]; // ^controid X_0

//   sva::PTransformd X_surface_com = X_0_surface * X_0_centroid.inv();

//   sva::ForceVecd wrench_out = X_surface_com.dualMul(wrench);

//   return wrench_out;
// }

sva::ForceVecd ForceTransmissionLocal::transformExternalWrench(const sva::ForceVecd wrench,
                                                               const biRobotTeleop::Limbs limb_robot,
                                                               int rIndex,
                                                               sva::PTransformd X_0_surface)
{
  sva::PTransformd X_0_centroid = worldCentroidKinePTrans_[rIndex - 1]; // ^controid X_0

  sva::PTransformd X_surface_com = X_0_surface * X_0_centroid.inv();

  sva::ForceVecd wrench_out = X_surface_com.dualMul(wrench);

  return wrench_out;
}

void ForceTransmissionLocal::getestimatedExternalWrench(mc_control::fsm::Controller & ctl_,
                                                        const std::string & robot_name,
                                                        int robot_index)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  if(exportExternalWrench_)
  {
    if(ctl.datastore().has(robot_name + "::estimatedExternalWrench_Force")
       && ctl.datastore().has(robot_name + "::estimatedExternalWrench_Torque"))
    {
      estimatedExternalWrench_centroid_[robot_index - 1].force() =
          ctl.datastore().get<Eigen::Vector3d>(robot_name + "::estimatedExternalWrench_Force");
      estimatedExternalWrench_centroid_[robot_index - 1].couple() =
          ctl.datastore().get<Eigen::Vector3d>(robot_name + "::estimatedExternalWrench_Torque");
    }
    if(ctl.datastore().has(robot_ + "::worldCentroidKinePTrans"))
    {
      worldCentroidKinePTrans_[robot_index - 1] =
          ctl.datastore().get<sva::PTransformd>(robot_ + "::worldCentroidKinePTrans");
    }
  }
  // else { mc_rtc::log::error("[ObserverbasedImpedanceTask] No EstimatedExternalWrench is exported"); }

  //  mc_rtc::log::info("[getestimatedExternalWrench] Wrench on robot {} is \n {}",
  //   robot_name,estimatedExternalWrench_centroid_[robot_index-1]);
  return;
}

sva::ForceVecd ForceTransmissionLocal::replaceForceTorque(sva::ForceVecd target)
{
  sva::ForceVecd tmp = sva::ForceVecd::Zero();
  tmp.couple() = target.force();
  tmp.force() = target.couple();

  return tmp;
}

void ForceTransmissionLocal::addLog(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();
  logger.addLogEntry(name() + "_robot1_external_centroid", [this]() -> const sva::ForceVecd & { return estimatedExternalWrench_centroid_[0]; });
  logger.addLogEntry(name() + "_robot1_external_centroid_without_bias", [this]() -> const sva::ForceVecd & { return estimatedExternalWrench_centroid_without_bias_[0]; });
  logger.addLogEntry(name() + "_robot2_external_centroid", [this]() -> const sva::ForceVecd & { return estimatedExternalWrench_centroid_[1]; });
  logger.addLogEntry(name() + "_robot2_external_centroid_without_bias", [this]() -> const sva::ForceVecd & { return estimatedExternalWrench_centroid_without_bias_[1]; });

  logger.addLogEntry(name() + "_robota_on_frame_fs", [this]() -> const sva::ForceVecd & { return measured_wrench_a; });
  logger.addLogEntry(name() + "_robota_on_frame_centroid", [this]() -> const sva::ForceVecd & { return measured_wrench_a_centroid; });
  logger.addLogEntry(name() + "_robota_on_frame_with_full_transfo", [this]() -> const sva::ForceVecd & { return measured_wrench_a_centroid_trasnform; });
  logger.addLogEntry(name() + "_robotb_on_frame_fs", [this]() -> const sva::ForceVecd & { return measured_wrench_b; });
  logger.addLogEntry(name() + "_robotb_on_frame_centroid", [this]() -> const sva::ForceVecd & { return measured_wrench_b_centroid; });
  logger.addLogEntry(name() + "_robotb_on_frame_with_full_transfo", [this]() -> const sva::ForceVecd & { return measured_wrench_b_centroid_trasnform; });
}



void ForceTransmissionLocal::addGUI(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  auto & gui = *ctl_.gui();
  gui.addElement({"States", name()},
                 mc_rtc::gui::Checkbox("Active", [this]() -> bool { return !active_; }, [this]() {}));

  size_t count = 0;
  for(auto & f : force_sensor_limbs_robot_1_)
  {
    gui.addElement(
        {"States", name()},
        mc_rtc::gui::ArrayLabel(
            "Measure Wrench robot_1: " + f, {"cx", "cy", "cz", "fx", "fy", "fz"},
            [this, &ctl, f]() -> Eigen::Vector6d
            {
              return ctl.robots().robot("robot_1").frame(ctl.r_1_.getName(biRobotTeleop::str2Limb(f))).wrench().vector();
            }));
    gui.addElement(
        {"States", name()},
        mc_rtc::gui::Label("Activation value robot_1: " + f, [this, &ctl, count]() -> double
                           { return activation_force_measurements_robot_1_[count].eval().vector().norm(); }));
    gui.addElement({"States", name()},
                   mc_rtc::gui::Checkbox(
                       "Force activation robot_1: " + f,
                       [this, &ctl, count]() -> bool { return robot_1_force_activation_[count]; }, [this, &ctl, count]()
                       { return robot_1_force_activation_[count] = !robot_1_force_activation_[count]; }));
    count += 1;
  }
  count = 0;
  for(auto & f : force_sensor_limbs_robot_2_)
  {
    gui.addElement(
        {"States", name()},
        mc_rtc::gui::ArrayLabel(
            "Measure Wrench robot_2: " + f, {"cx", "cy", "cz", "fx", "fy", "fz"},
            [this, &ctl, f]() -> Eigen::Vector6d
            {
              return ctl.robots().robot("robot_2").frame(ctl.r_2_.getName(biRobotTeleop::str2Limb(f))).wrench().vector();
            }));
    gui.addElement(
        {"States", name()},
        mc_rtc::gui::Label("Activation value robot_2: " + f, [this, &ctl, count]() -> double
                           { return activation_force_measurements_robot_2_[count].eval().vector().norm(); }));
    gui.addElement({"States", name()},
                   mc_rtc::gui::Checkbox(
                       "Force activation robot_2: " + f,
                       [this, &ctl, count]() -> bool { return robot_2_force_activation_[count]; }, [this, &ctl, count]()
                       { return robot_2_force_activation_[count] = !robot_2_force_activation_[count]; }));
    count += 1;
  }
}

void ForceTransmissionLocal::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.gui()->removeCategory({"States", name()});
  ctl_.solver().removeTask(task_a_);
  ctl_.solver().removeTask(task_b_);
}

EXPORT_SINGLE_STATE("ForceTransmissionLocal", ForceTransmissionLocal)
