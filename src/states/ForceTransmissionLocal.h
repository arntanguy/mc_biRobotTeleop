#pragma once

#include <mc_control/fsm/State.h>
#include <mc_filter/LowPass.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Input.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/DampingTask.h>
#include <mc_tasks/TransformTask.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

#include <biRobotTeleop/HumanRobotPose.h>
#include <biRobotTeleop/type.h>

// #include <numeric>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
// #include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace ba = boost::accumulators;
using namespace boost::numeric::operators;
typedef ba::accumulator_set<double, ba::stats<ba::tag::rolling_mean>> accumulator_t;

struct ForceTransmissionLocal : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addGUI(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

  /**
   * @brief Compute the human contact limb with robot limb
   *
   * @param ctl_
   * @param robot_indx
   * @param robot_limb
   * @return const biRobotTeleop::Limbs
   */
  const biRobotTeleop::Limbs getContactLimb(mc_control::fsm::Controller & ctl_,
                                            const int robot_indx,
                                            const biRobotTeleop::Limbs & robot_limb);

  /**
   * @brief Get the contact location on the robot indx limb, it suppose contact exist
   *
   * @param ctl_
   * @param robot_indx
   * @param limb_robot
   * @param limb_human
   * @return Eigen::Vector3d
   */
  std::tuple<const Eigen::Vector3d,const Eigen::Vector3d, const Eigen::Vector3d>  getDistanceAndContactPoint(mc_control::fsm::Controller & ctl_,
                                           const int robot_indx,
                                           const biRobotTeleop::Limbs limb_robot,
                                           const biRobotTeleop::Limbs limb_human);

  const Eigen::Vector3d getContactDistance(mc_control::fsm::Controller & ctl_,
                                                                 const int robot_indx,
                                                                 const biRobotTeleop::Limbs limb_robot,
                                                                 const biRobotTeleop::Limbs limb_human);

  bool checkActivation(mc_control::fsm::Controller & ctl_, const int robot_indx);

  bool checkActivationTest(mc_control::fsm::Controller & ctl_, const int robot_indx);

  sva::ForceVecd replaceForceTorque(sva::ForceVecd target);
  void getestimatedContactWrench(mc_control::fsm::Controller & ctl_, const std::string & surface);

  sva::ForceVecd transformExternalWrench(mc_control::fsm::Controller & ctl_,const sva::ForceVecd wrench,
                                         const biRobotTeleop::Limbs limb_robot,
                                         int rIndex);
  sva::ForceVecd transformExternalWrench(const sva::ForceVecd wrench,
                                         const biRobotTeleop::Limbs limb_robot,
                                         int rIndex,
                                         sva::PTransformd X_0_surface);

  void getestimatedExternalWrench(mc_control::fsm::Controller & ctl_, const std::string & robot_name, int robot_index);

  void calculateMovingAverage(int robot_index)
  {
    std::vector<accumulator_t> * accumulator = &accumulator_1_;
    if(robot_index == 2)
    {
      accumulator = &accumulator_2_;
    }
    Eigen::Vector6d temp;
    if(ba::rolling_count((*accumulator)[0]) < 100)
    {
      std::cout << "not enough samples " << std::endl;
      return;
    }
    for(int i = 0; i < 6; i++)
    {
      temp[i] = ba::rolling_mean((*accumulator)[i]);
    }
    sva::ForceVecd estimatedExternalWrench_centroid_bias = temp;
    // mc_rtc::log::info("[calculateMovingAverage] mean on robot {} is \n {}", robot_index, temp);
    estimatedExternalWrench_centroid_without_bias_[robot_index - 1] =
        estimatedExternalWrench_centroid_[robot_index - 1] - estimatedExternalWrench_centroid_bias;
    mc_rtc::log::info("[calculateMovingAverage] Wrench without bias on robot {} is \n {}", robot_index,
                      estimatedExternalWrench_centroid_without_bias_[robot_index - 1]);
  }

  void addDataToAverage(const sva::ForceVecd vec, int robot_index)
  {
    mc_rtc::log::info("adding data to average");
    if(robot_index == 1)
    {
      // mc_rtc::log::info("[addDataToAverage] added robot {} is \n {}", robot_index, vec.vector());
      for(int i = 0; i < 6; i++)
      {
        accumulator_1_[i](vec.vector()[i]);
      }
    }
    else
    {
      for(int i = 0; i < 6; i++)
      {
        accumulator_2_[i](vec.vector()[i]);
      }
    }
  }





  double dt_ = 5e-3;

  std::shared_ptr<mc_tasks::force::DampingTask> task_a_;
  std::shared_ptr<mc_tasks::force::DampingTask> task_b_;

  std::shared_ptr<mc_tasks::force::DampingTask> task_robot_2_;
  std::shared_ptr<mc_tasks::force::DampingTask> task_robot_1_;

  std::string robot_b_custom_force_sensor_name_;
  std::string robot_a_custom_force_sensor_name_;

  int indx_ = 0; // map robot to a either 1 or 2

  biRobotTeleop::Limbs limb_a_ = biRobotTeleop::Limbs::Head; // limbs of task on robot a
  biRobotTeleop::Limbs limb_b_ = biRobotTeleop::Limbs::Head; // limbs of task on robot b

  const mc_rbdyn::Robot * human_1_estimated_ = nullptr;
  const mc_rbdyn::Robot * human_2_estimated_ = nullptr;

  bool done_ = false;

  bool active_ = false;

  sva::PTransformd X_r1_r2_ = sva::PTransformd::Identity(); // offset transfo from r1 task frame to r2 task frame;

  std::vector<mc_filter::LowPass<sva::ForceVecd>>
      activation_force_measurements_robot_1_; // the threshold must be over a low pass filtered value of the force/sensor
  std::vector<mc_filter::LowPass<sva::ForceVecd>>
      activation_force_measurements_robot_2_; // the threshold must be over a low pass filtered value of the force/sensor
  std::vector<bool> robot_1_force_activation_; // to enforce activation
  std::vector<bool> robot_2_force_activation_; // to enforce activation
  bool activation_enforced_ = false;

  std::string robot_b_name_ = "robot_1";
  std::string robot_a_name_ = "robot_2";

  mc_filter::LowPass<sva::ForceVecd> * active_force_measurement_ =
      nullptr; // If the active force control filtered measurement is below the threshold, the force control is
               // deactivated

  std::string contact_limb_; // limb in contact with a robot link equipped of force sensors
  std::vector<std::string> force_sensor_limbs_robot_1_;
  std::vector<std::string> force_sensor_limbs_robot_2_;

  sva::ForceVecd measured_wrench_a = sva::ForceVecd::Zero(); // measured force in the link frame
  sva::ForceVecd measured_wrench_b = sva::ForceVecd::Zero(); // measured force in the link frame
  sva::ForceVecd measured_wrench_a_centroid = sva::ForceVecd::Zero(); // measured force in the link frame
  sva::ForceVecd measured_wrench_b_centroid = sva::ForceVecd::Zero(); // measured force in the link frame
  sva::ForceVecd measured_wrench_b_centroid_trasnform = sva::ForceVecd::Zero(); // measured force in the link frame
  sva::ForceVecd measured_wrench_a_centroid_trasnform = sva::ForceVecd::Zero(); // measured force in the link frame

  double force_activation_threshold_ = 10; // force threshold on which the force control is activated;
  double distance_activation_threshold_ = 0.02;
  double deactivation_threshold_ = 0.15; // distance threshold on which the force control is deactivated

private:
  std::string robot_;
  bool exportContactWrench_ = true;
  bool exportExternalWrench_ = true;
  int MaxContacts_ = 4;
  std::string usingWrench_ = "Sensor";

  bool addPlot_ = false;

  sva::ForceVecd estimatedContactWrench_;
  sva::ForceVecd estimatedContactWrench_sensorFrame_;

  sva::ForceVecd estimationError_;

  sva::ForceVecd surfaceWrench_;

  // sva::PTransformd worldCentroidKinePTrans_2_;
  // sva::ForceVecd estimatedExternalWrench_centroid_2_;
  // sva::ForceVecd estimatedExternalWrench_centroid_without_bias_2_;
  // sva::ForceVecd estimatedExternalWrench_centroid_bias_;

  std::vector<sva::PTransformd> worldCentroidKinePTrans_{sva::PTransformd::Identity(), sva::PTransformd::Identity()};
  std::vector<sva::ForceVecd> estimatedExternalWrench_centroid_{sva::ForceVecd::Zero(), sva::ForceVecd::Zero()};
  std::vector<sva::ForceVecd> estimatedExternalWrench_centroid_without_bias_{sva::ForceVecd::Zero(),
                                                                             sva::ForceVecd::Zero()};
  // sva::ForceVecd estimatedExternalWrench_centroid_bias_;

  std::vector<accumulator_t> accumulator_1_;
  std::unordered_map<int, Eigen::Vector3d> closests_points_robot_1_;

  std::vector<accumulator_t> accumulator_2_;
  std::unordered_map<int, Eigen::Vector3d> closests_points_robot_2_;

  Eigen::Vector3d closest_point_;

  // ba::accumulator_set<std::vector<double>, ba::stats<ba::tag::rolling_mean>>  acc_of_vectors_;
};
