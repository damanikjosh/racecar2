/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "racecar_gazebo/RacecarController.hh"

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Model.hh"

using namespace racecar_gazebo;

class racecar_gazebo::RacecarControllerPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const gz::msgs::Float &_msg);

  /// \brief Callback for steering subscription
  /// \param[in] _msg Steering message
  public: void OnCmdSteer(const gz::msgs::Float &_msg);

  public: void PublishOdometry(const std::chrono::steady_clock::duration &_now,
    const double x, const double y, const double heading,
    const double linear, const double angular);

  /// \brief Gazebo communication node.
  public: gz::transport::Node node;

  public: gz::transport::Node::Publisher odomPub;
  public: gz::transport::Node::Publisher tfPub;

  public: std::string robotFrame{"base_link"};
  public: std::string odomFrame{"odom"};

  public: std::chrono::steady_clock::duration odomPeriod{0};
  public: std::chrono::steady_clock::duration lastOdomTime{0};

  public: bool initialized{false};
  public: bool publishTf{true};

  public: public: double gaussianNoise = 0.0;

  /// \brief Wheels Entity
  public: gz::sim::Entity leftFrontWheelEntity{gz::sim::kNullEntity};
  public: gz::sim::Entity rightFrontWheelEntity{gz::sim::kNullEntity};
  public: gz::sim::Entity leftRearWheelEntity{gz::sim::kNullEntity};
  public: gz::sim::Entity rightRearWheelEntity{gz::sim::kNullEntity};

  /// \brief Steering Entity
  public: gz::sim::Entity leftSteeringEntity{gz::sim::kNullEntity};
  public: gz::sim::Entity rightSteeringEntity{gz::sim::kNullEntity};

  /// \brief Wheels joint names
  public: std::string leftFrontWheelJointName;
  public: std::string rightFrontWheelJointName;
  public: std::string leftRearWheelJointName;
  public: std::string rightRearWheelJointName;

  /// \brief Steering joint names
  public: std::string leftSteeringJointName;
  public: std::string rightSteeringJointName;

  /// \brief Ackermann steering parameters
  public: double wheelBase{0.0};
  public: double trackWidth{0.0};
  public: double wheelRadius{0.0};

  /// \brief Commanded joint velocity
  public: double velCmd{0.0};
  public: double steerCmd{0.0};

  /// \brief mutex to protect jointVelCmd
  public: std::mutex cmdMutex;

  /// \brief Model interface
  public: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Velocity PID controller.
  public: gz::math::PID leftFrontWheelPid;
  public: gz::math::PID rightFrontWheelPid;
  public: gz::math::PID leftRearWheelPid;
  public: gz::math::PID rightRearWheelPid;
  public: gz::math::PID leftSteeringPid;
  public: gz::math::PID rightSteeringPid;
};

//////////////////////////////////////////////////
RacecarController::RacecarController()
  : dataPtr(std::make_unique<RacecarControllerPrivate>())
{
}

//////////////////////////////////////////////////
void RacecarController::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "RacecarController plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("left_front_wheel_joint");
  if (sdfElem)
  {
    this->dataPtr->leftFrontWheelJointName = sdfElem->Get<std::string>();
  }
  else
  {
    gzerr << "Failed to get <left_front_wheel_joint>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("right_front_wheel_joint");
  if (sdfElem)
  {
    this->dataPtr->rightFrontWheelJointName = sdfElem->Get<std::string>();
  }
  else
  {
    gzerr << "Failed to get <right_front_wheel_joint>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("left_rear_wheel_joint");
  if (sdfElem)
  {
    this->dataPtr->leftRearWheelJointName = sdfElem->Get<std::string>();
  }
  else
  {
    gzerr << "Failed to get <left_rear_wheel_joint>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("right_rear_wheel_joint");
  if (sdfElem)
  {
    this->dataPtr->rightRearWheelJointName = sdfElem->Get<std::string>();
  }
  else
  {
    gzerr << "Failed to get <right_rear_wheel_joint>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("left_steering_joint");
  if (sdfElem)
  {
    this->dataPtr->leftSteeringJointName = sdfElem->Get<std::string>();
  }
  else
  {
    gzerr << "Failed to get <left_steering_joint>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("right_steering_joint");
  if (sdfElem)
  {
    this->dataPtr->rightSteeringJointName = sdfElem->Get<std::string>();
  }
  else
  {
    gzerr << "Failed to get <right_steering_joint>." << std::endl;
    return;
  }

  // Get Ackermann steering parameters
  sdfElem = _sdf->FindElement("wheel_base");
  if (sdfElem)
  {
    this->dataPtr->wheelBase = sdfElem->Get<double>();
  }
  else
  {
    gzerr << "Failed to get <wheel_base>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("track_width");
  if (sdfElem)
  {
    this->dataPtr->trackWidth = sdfElem->Get<double>();
  }
  else
  {
    gzerr << "Failed to get <track_width>." << std::endl;
    return;
  }

  sdfElem = _sdf->FindElement("wheel_radius");
  if (sdfElem)
  {
    this->dataPtr->wheelRadius = sdfElem->Get<double>();
  }
  else
  {
    gzerr << "Failed to get <wheel_radius>." << std::endl;
    return;
  }

  // Velocity PID parameters
  double p         = _sdf->Get<double>("vel_p_gain",     1.0).first;
  double i         = _sdf->Get<double>("vel_i_gain",     0.0).first;
  double d         = _sdf->Get<double>("vel_d_gain",     0.0).first;
  double iMax      = _sdf->Get<double>("vel_i_max",      1.0).first;
  double iMin      = _sdf->Get<double>("vel_i_min",     -1.0).first;
  double cmdMax    = _sdf->Get<double>("vel_cmd_max",    1000.0).first;
  double cmdMin    = _sdf->Get<double>("vel_cmd_min",   -1000.0).first;
  double cmdOffset = _sdf->Get<double>("vel_cmd_offset", 0.0).first;

  // this->dataPtr->velPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->leftFrontWheelPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->rightFrontWheelPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->leftRearWheelPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->rightRearWheelPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  gzdbg << "[RacecarController] Velocity PID controllers defined with parameters:" << std::endl;
  gzdbg << "p_gain: ["     << p         << "]"             << std::endl;
  gzdbg << "i_gain: ["     << i         << "]"             << std::endl;
  gzdbg << "d_gain: ["     << d         << "]"             << std::endl;
  gzdbg << "i_max: ["      << iMax      << "]"             << std::endl;
  gzdbg << "i_min: ["      << iMin      << "]"             << std::endl;
  gzdbg << "cmd_max: ["    << cmdMax    << "]"             << std::endl;
  gzdbg << "cmd_min: ["    << cmdMin    << "]"             << std::endl;
  gzdbg << "cmd_offset: [" << cmdOffset << "]"             << std::endl;


  // Steering PID parameters
  p         = _sdf->Get<double>("steer_p_gain",     1.0).first;
  i         = _sdf->Get<double>("steer_i_gain",     0.0).first;
  d         = _sdf->Get<double>("steer_d_gain",     0.0).first;
  iMax      = _sdf->Get<double>("steer_i_max",      1.0).first;
  iMin      = _sdf->Get<double>("steer_i_min",     -1.0).first;
  cmdMax    = _sdf->Get<double>("steer_cmd_max",    1000.0).first;
  cmdMin    = _sdf->Get<double>("steer_cmd_min",   -1000.0).first;
  cmdOffset = _sdf->Get<double>("steer_cmd_offset", 0.0).first;

  // this->dataPtr->steerPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->leftSteeringPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
  this->dataPtr->rightSteeringPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  gzdbg << "[RacecarController] Steering PID controllers defined with parameters:" << std::endl;

  gzdbg << "p_gain: ["     << p         << "]"             << std::endl;
  gzdbg << "i_gain: ["     << i         << "]"             << std::endl;
  gzdbg << "d_gain: ["     << d         << "]"             << std::endl;
  gzdbg << "i_max: ["      << iMax      << "]"             << std::endl;
  gzdbg << "i_min: ["      << iMin      << "]"             << std::endl;
  gzdbg << "cmd_max: ["    << cmdMax    << "]"             << std::endl;
  gzdbg << "cmd_min: ["    << cmdMin    << "]"             << std::endl;
  gzdbg << "cmd_offset: [" << cmdOffset << "]"             << std::endl;

  odometry.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  odometry.set_wheel_params(this->dataPtr->wheelRadius, this->dataPtr->wheelBase, this->dataPtr->trackWidth);
  odometry.init();

  if (_sdf->HasElement("robot_frame"))
  {
    this->dataPtr->robotFrame = _sdf->Get<std::string>("robot_frame");
  }

  if (_sdf->HasElement("odom_frame"))
  {
    this->dataPtr->odomFrame = _sdf->Get<std::string>("odom_frame");
  }

  if (_sdf->HasElement("gaussian_noise"))
  {
    this->dataPtr->gaussianNoise = _sdf->Get<double>("gaussian_noise");
  }

  if (_sdf->HasElement("publish_tf"))
  {
    this->dataPtr->publishTf = _sdf->Get<bool>("publish_tf");
  }


  // Define publishers
  double odomFreq = _sdf->Get<double>("odom_freq", 50.0).first;
  if (odomFreq > 0.0)
  {
    std::chrono::duration<double> period{1.0 / odomFreq};
    this->dataPtr->odomPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }
  std::string topic = gz::transport::TopicUtils::AsValidTopic("odom");
  if (_sdf->HasElement("odom_topic"))
    topic = gz::transport::TopicUtils::AsValidTopic(_sdf->Get<std::string>("odom_topic"));
  if (topic.empty())
  {
    gzerr << "Failed to create topic [" << _sdf->Get<std::string>("odom_topic") << "]" << std::endl;
    return;
  }

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<gz::msgs::OdometryWithCovariance>(topic);

  gzmsg << "RacecarController publishing OdometryWithCovariance to [" << topic << "]" << std::endl;

  if (this->dataPtr->publishTf)
  {
    // Define publishers
    topic = gz::transport::TopicUtils::AsValidTopic("tf");
    if (_sdf->HasElement("tf_topic"))
      topic = gz::transport::TopicUtils::AsValidTopic(_sdf->Get<std::string>("tf_topic"));
    if (topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("tf_topic") << "]" << std::endl;
      return;
    }

    this->dataPtr->tfPub = this->dataPtr->node.Advertise<gz::msgs::Pose_V>(topic);

    gzmsg << "RacecarController publishing Pose_V to [" << topic << "]" << std::endl;
  }


  // Subscribe to commands
  
  if (_sdf->HasElement("vel_topic"))
  {
    topic = gz::transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("vel_topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create velocity topic [" << _sdf->Get<std::string>("topic") << "]" << std::endl;
      return;
    }

    this->dataPtr->node.Subscribe(topic, &RacecarControllerPrivate::OnCmdVel, this->dataPtr.get());

    gzmsg << "RacecarController subscribing to Float messages on [" << topic << "]" << std::endl;

  }

  if (_sdf->HasElement("steer_topic"))
  {
    topic = gz::transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("steer_topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create steering topic [" << _sdf->Get<std::string>("topic") << "]" << std::endl;
      return;
    }

    this->dataPtr->node.Subscribe(topic, &RacecarControllerPrivate::OnCmdSteer, this->dataPtr.get());

    gzmsg << "RacecarController subscribing to Float messages on [" << topic << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void RacecarController::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("RacecarController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Get the joint entities
  if (this->dataPtr->leftFrontWheelEntity == gz::sim::kNullEntity)
    this->dataPtr->leftFrontWheelEntity = this->dataPtr->model.JointByName(_ecm, this->dataPtr->leftFrontWheelJointName);
  if (this->dataPtr->rightFrontWheelEntity == gz::sim::kNullEntity)
    this->dataPtr->rightFrontWheelEntity = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rightFrontWheelJointName);
  if (this->dataPtr->leftRearWheelEntity == gz::sim::kNullEntity)
    this->dataPtr->leftRearWheelEntity = this->dataPtr->model.JointByName(_ecm, this->dataPtr->leftRearWheelJointName);
  if (this->dataPtr->rightRearWheelEntity == gz::sim::kNullEntity)
    this->dataPtr->rightRearWheelEntity = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rightRearWheelJointName);
  if (this->dataPtr->leftSteeringEntity == gz::sim::kNullEntity)
    this->dataPtr->leftSteeringEntity = this->dataPtr->model.JointByName(_ecm, this->dataPtr->leftSteeringJointName);
  if (this->dataPtr->rightSteeringEntity == gz::sim::kNullEntity)
    this->dataPtr->rightSteeringEntity = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rightSteeringJointName);

  if (this->dataPtr->leftFrontWheelEntity == gz::sim::kNullEntity ||
      this->dataPtr->rightFrontWheelEntity == gz::sim::kNullEntity ||
      this->dataPtr->leftRearWheelEntity == gz::sim::kNullEntity ||
      this->dataPtr->rightRearWheelEntity == gz::sim::kNullEntity ||
      this->dataPtr->leftSteeringEntity == gz::sim::kNullEntity ||
      this->dataPtr->rightSteeringEntity == gz::sim::kNullEntity)
  {
    gzwarn << "Failed to find joint." << std::endl;
    return;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Create joint velocity component if one doesn't exist
  auto leftFrontWheelVelComp = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->leftFrontWheelEntity);
  if (!leftFrontWheelVelComp)
    _ecm.CreateComponent(this->dataPtr->leftFrontWheelEntity, gz::sim::components::JointVelocity());

  auto rightFrontWheelVelComp = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->rightFrontWheelEntity);
  if (!rightFrontWheelVelComp)
    _ecm.CreateComponent(this->dataPtr->rightFrontWheelEntity, gz::sim::components::JointVelocity());

  auto leftRearWheelVelComp = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->leftRearWheelEntity);
  if (!leftRearWheelVelComp)
    _ecm.CreateComponent(this->dataPtr->leftRearWheelEntity, gz::sim::components::JointVelocity());

  auto rightRearWheelVelComp = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->rightRearWheelEntity);
  if (!rightRearWheelVelComp)
    _ecm.CreateComponent(this->dataPtr->rightRearWheelEntity, gz::sim::components::JointVelocity());

  auto leftSteeringPosComp = _ecm.Component<gz::sim::components::JointPosition>(this->dataPtr->leftSteeringEntity);
  if (!leftSteeringPosComp)
    _ecm.CreateComponent(this->dataPtr->leftSteeringEntity, gz::sim::components::JointPosition());

  auto rightSteeringPosComp = _ecm.Component<gz::sim::components::JointPosition>(this->dataPtr->rightSteeringEntity);
  if (!rightSteeringPosComp)
    _ecm.CreateComponent(this->dataPtr->rightSteeringEntity, gz::sim::components::JointPosition());

  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  // if (jointVelComp == nullptr || jointVelComp->Data().empty())
  if (leftFrontWheelVelComp == nullptr || leftFrontWheelVelComp->Data().empty() ||
      rightFrontWheelVelComp == nullptr || rightFrontWheelVelComp->Data().empty() ||
      leftRearWheelVelComp == nullptr || leftRearWheelVelComp->Data().empty() ||
      rightRearWheelVelComp == nullptr || rightRearWheelVelComp->Data().empty() ||
      leftSteeringPosComp == nullptr || leftSteeringPosComp->Data().empty() ||
      rightSteeringPosComp == nullptr || rightSteeringPosComp->Data().empty())
    return;

  double targetVel = 0.0, targetSteer = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->cmdMutex);
    // TODO: Calculate the target velocity and steering angle for each joints using Ackeramn steering
    targetVel = this->dataPtr->velCmd;
    targetSteer = this->dataPtr->steerCmd;
    
    // For now, we will just set the target velocity to the same value for all wheels
  }

  odometry.update_from_velocity(
    rightRearWheelVelComp->Data()[0], leftRearWheelVelComp->Data()[0],
    rightFrontWheelVelComp->Data()[0], leftFrontWheelVelComp->Data()[0],
    rightSteeringPosComp->Data()[0], leftSteeringPosComp->Data()[0],
    std::chrono::duration<double>(_info.dt).count());
  
  auto [targetTractions, targetSteerings] = odometry.get_commands_ackermann(targetVel, targetSteer);

  double Wrr = targetTractions[0];
  double Wlr = targetTractions[1];
  double Wrf = targetTractions[2];
  double Wlf = targetTractions[3];

  double Srr = targetSteerings[0];
  double Slr = targetSteerings[1];

  // Update force command.
  double velEffort = this->dataPtr->leftFrontWheelPid.Update(leftFrontWheelVelComp->Data()[0] - Wlf, _info.dt);
  auto leftFrontWheelForceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->leftFrontWheelEntity);
  if (leftFrontWheelForceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->leftFrontWheelEntity,
        gz::sim::components::JointForceCmd({velEffort}));
  }
  else
  {
    *leftFrontWheelForceComp = gz::sim::components::JointForceCmd({velEffort});
  }
  
  velEffort = this->dataPtr->rightFrontWheelPid.Update(rightFrontWheelVelComp->Data()[0] - Wrf, _info.dt);
  auto rightFrontWheelForceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->rightFrontWheelEntity);
  if (rightFrontWheelForceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->rightFrontWheelEntity,
        gz::sim::components::JointForceCmd({velEffort}));
  }
  else
  {
    *rightFrontWheelForceComp = gz::sim::components::JointForceCmd({velEffort});
  }

  velEffort = this->dataPtr->leftRearWheelPid.Update(leftRearWheelVelComp->Data()[0] - Wlr, _info.dt);
  auto leftRearWheelForceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->leftRearWheelEntity);
  if (leftRearWheelForceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->leftRearWheelEntity,
        gz::sim::components::JointForceCmd({velEffort}));
  }
  else
  {
    *leftRearWheelForceComp = gz::sim::components::JointForceCmd({velEffort});
  }

  velEffort = this->dataPtr->rightRearWheelPid.Update(rightRearWheelVelComp->Data()[0] - Wrr, _info.dt);
  auto rightRearWheelForceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->rightRearWheelEntity);
  if (rightRearWheelForceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->rightRearWheelEntity,
        gz::sim::components::JointForceCmd({velEffort}));
  }
  else
  {
    *rightRearWheelForceComp = gz::sim::components::JointForceCmd({velEffort});
  }

  // Update steering command.
  double steerEffort = this->dataPtr->leftSteeringPid.Update(leftSteeringPosComp->Data()[0] - Slr, _info.dt);
  auto leftSteeringForceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->leftSteeringEntity);
  if (leftSteeringForceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->leftSteeringEntity,
        gz::sim::components::JointForceCmd({steerEffort}));
  }
  else
  {
    *leftSteeringForceComp = gz::sim::components::JointForceCmd({steerEffort});
  }

  steerEffort = this->dataPtr->rightSteeringPid.Update(rightSteeringPosComp->Data()[0] - Srr, _info.dt);
  auto rightSteeringForceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->rightSteeringEntity);
  if (rightSteeringForceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->rightSteeringEntity,
        gz::sim::components::JointForceCmd({steerEffort}));
  }
  else
  {
    *rightSteeringForceComp = gz::sim::components::JointForceCmd({steerEffort});
  }

  this->dataPtr->PublishOdometry(_info.simTime, odometry.get_x(), odometry.get_y(), odometry.get_heading(),
    odometry.get_linear(), odometry.get_angular());

}

//////////////////////////////////////////////////
void RacecarControllerPrivate::OnCmdVel(const gz::msgs::Float &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->velCmd = _msg.data();
}

//////////////////////////////////////////////////
void RacecarControllerPrivate::OnCmdSteer(const gz::msgs::Float &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->steerCmd = _msg.data();

  // TODO: Constrain the angle
  // If the value is more than pi/2, set it to pi/2
  // If the value is less than -pi/2, set it to -pi/2
  this->steerCmd = std::min(std::max(this->steerCmd, -M_PI/2+0.001), M_PI/2-0.001);
}

//////////////////////////////////////////////////
void RacecarControllerPrivate::PublishOdometry(
  const std::chrono::steady_clock::duration &_now,
  const double x, const double y, const double heading,
  const double linear, const double angular)
{
  if (!this->initialized)
  {
    this->lastOdomTime = _now;
    this->initialized = true;
    return;
  }

  if (_now - this->lastOdomTime < this->odomPeriod)
    return;

  this->lastOdomTime = _now;

  gz::msgs::Header header;
  header.mutable_stamp()->set_sec(
      std::chrono::duration_cast<std::chrono::seconds>(_now).count());
  header.mutable_stamp()->set_nsec(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_now).count() % 1000000000);
  
  auto frame = header.add_data();
  frame->set_key("frame_id");
  frame->add_value(this->odomFrame);
  auto childFrame = header.add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(this->robotFrame);

  gz::msgs::OdometryWithCovariance odomMsg;
  odomMsg.mutable_header()->CopyFrom(header);

  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_position()->set_x(x);
  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_position()->set_y(y);
  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_position()->set_z(0.0);
  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_orientation()->set_x(0.0);
  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_orientation()->set_y(0.0);
  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_orientation()->set_z(sin(heading / 2));
  odomMsg.mutable_pose_with_covariance()->mutable_pose()->mutable_orientation()->set_w(cos(heading / 2));

  odomMsg.mutable_twist_with_covariance()->mutable_twist()->mutable_linear()->set_x(linear);
  odomMsg.mutable_twist_with_covariance()->mutable_twist()->mutable_linear()->set_y(0.0);
  odomMsg.mutable_twist_with_covariance()->mutable_twist()->mutable_linear()->set_z(0.0);
  odomMsg.mutable_twist_with_covariance()->mutable_twist()->mutable_angular()->set_x(0.0);
  odomMsg.mutable_twist_with_covariance()->mutable_twist()->mutable_angular()->set_y(0.0);
  odomMsg.mutable_twist_with_covariance()->mutable_twist()->mutable_angular()->set_z(angular);
  
  auto gn2 = this->gaussianNoise * this->gaussianNoise;
  for (int i = 0; i < 36; i++)
  {
    if (i % 7 == 0)
    {
      odomMsg.mutable_pose_with_covariance()->
        mutable_covariance()->add_data(gn2);
      odomMsg.mutable_twist_with_covariance()->
        mutable_covariance()->add_data(gn2);
    }
    else
    {
      odomMsg.mutable_pose_with_covariance()->
        mutable_covariance()->add_data(0);
      odomMsg.mutable_twist_with_covariance()->
        mutable_covariance()->add_data(0);
    }
  }

  this->odomPub.Publish(odomMsg);

  if (!this->publishTf)
    return;

  gz::msgs::Pose_V tfMsg;
  auto tfMsgPose = tfMsg.add_pose();
  tfMsgPose->CopyFrom(odomMsg.pose_with_covariance().pose());
  tfMsgPose->mutable_header()->CopyFrom(header);

  this->tfPub.Publish(tfMsg);
}
      

GZ_ADD_PLUGIN(RacecarController,
                    gz::sim::System,
                    RacecarController::ISystemConfigure,
                    RacecarController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RacecarController,
                          "racecar_gazebo::RacecarController")