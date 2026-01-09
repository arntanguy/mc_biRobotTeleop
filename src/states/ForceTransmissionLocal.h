#pragma once

#include <mc_control/fsm/State.h>
#include <mc_filter/LowPass.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Input.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/DampingTask.h>
#include <mc_tasks/TransformTask.h>

#include <biRobotTeleop/HumanRobotPose.h>
#include <biRobotTeleop/type.h>

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
                                            const biRobotTeleop::Limbs & robot_limb) const;

  /**
   * @brief Get the contact location on the robot indx limb, it suppose contact exist
   *
   * @param ctl_
   * @param robot_indx
   * @param limb_robot
   * @param limb_human
   * @return Eigen::Vector3d
   */
  const Eigen::Vector3d getContactDistance(mc_control::fsm::Controller & ctl_,
                                           const int robot_indx,
                                           const biRobotTeleop::Limbs limb_robot,
                                           const biRobotTeleop::Limbs limb_human) const;

  bool checkActivation(mc_control::fsm::Controller & ctl_, const int robot_indx);

  sva::ForceVecd ForceTransmissionLocal::replaceForceTorque(sva::ForceVecd target);
  void ForceTransmissionLocal::getestimatedContactWrench(mc_control::fsm::Controller & ctl_,
                                                         const std::string & surface);


  sva::ForceVecd ObserverbasedImpedanceTask::transformExternalWrench(const sva::ForceVecd wrench,
                                                                   const std::string surface)
    {
    sva::PTransformd X_0_surface = robots.robot(rIndex).frame(surface).position(); // ^surface X_0

    sva::PTransformd X_0_centroid = worldCentroidKinePTrans_; // ^controid X_0

    sva::PTransformd X_surface_com = X_0_surface * X_0_centroid.inv();

    sva::ForceVecd wrench_out = X_surface_com.dualMul(wrench);

    return wrench_out;
    }

    void ObserverbasedImpedanceTask::getestimatedExternalWrench()
    {
        if(exportExternalWrench_)
        {
            if(controller_->datastore().has(robot_ + "::estimatedExternalWrench_Force")
            && controller_->datastore().has(robot_ + "::estimatedExternalWrench_Torque"))
            {
            estimatedExternalWrench_centroid_.force() =
                controller_->datastore().get<Eigen::Vector3d>(robot_ + "::estimatedExternalWrench_Force");
            estimatedExternalWrench_centroid_.couple() =
                controller_->datastore().get<Eigen::Vector3d>(robot_ + "::estimatedExternalWrench_Torque");
            }
            else
            {
            auto keys = controller_->datastore().keys();
            std::string keys_str;
            for(size_t i = 0; i < keys.size(); ++i)
            {
                keys_str += keys[i];
                if(i < keys.size() - 1) keys_str += ", ";
            }

            // mc_rtc::log::error("[ObserverbasedImpedanceTask] {} is empty. \n Available keys are {}",
            //                    robot_ + "::estimatedExternalWrench", keys_str);
            }
            if(controller_->datastore().has(robot_ + "::worldCentroidKinePTrans"))
            {
            worldCentroidKinePTrans_ = controller_->datastore().get<sva::PTransformd>(robot_ + "::worldCentroidKinePTrans");
            }
        }
    // else { mc_rtc::log::error("[ObserverbasedImpedanceTask] No EstimatedExternalWrench is exported"); }
    return;
    }

  double dt_ = 5e-3;

  std::shared_ptr<mc_tasks::force::DampingTask> task_a_;
  std::shared_ptr<mc_tasks::force::DampingTask> task_b_;

  std::shared_ptr<mc_tasks::force::DampingTask> task_robot_2_;
  std::shared_ptr<mc_tasks::force::DampingTask> task_robot_1_;

  std::string robot_b_custom_force_sensor_name_;

  int indx_ = 0; // map robot to a either 1 or 2

  biRobotTeleop::Limbs limb_a_ = biRobotTeleop::Limbs::Head; // limbs of task on robot a
  biRobotTeleop::Limbs limb_b_ = biRobotTeleop::Limbs::Head; // limbs of task on robot b

  const mc_rbdyn::Robot & human_1_estimated;
  const mc_rbdyn::Robot & human_2_estimated;

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

  std::string robot_b_name_;
  std::string robot_a_name_;

  mc_filter::LowPass<sva::ForceVecd> * active_force_measurement_ =
      nullptr; // If the active force control filtered measurement is below the threshold, the force control is
               // deactivated

  std::string contact_limb_; // limb in contact with a robot link equipped of force sensors
  std::vector<std::string> force_sensor_limbs_robot_1_;
  std::vector<std::string> force_sensor_limbs_robot_2_;

  sva::ForceVecd measured_force_robot_1_ = sva::ForceVecd::Zero(); // measured force in the link frame
  sva::ForceVecd measured_force_robot_2_ = sva::ForceVecd::Zero(); // measured force in the link frame

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

  sva::ForceVecd estimatedExternalWrench_centroid_;
  mc_control::MCController * controller_;

  sva::ForceVecd estimationError_;

  sva::ForceVecd surfaceWrench_;

  sva::PTransformd worldCentroidKinePTrans_;
};
