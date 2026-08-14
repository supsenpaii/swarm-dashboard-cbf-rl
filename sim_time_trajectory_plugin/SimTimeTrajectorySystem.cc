#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace swarm::simulation
{
class SimTimeTrajectorySystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  private: struct Command
  {
    bool stop{false};
    std::string session;
    double startRange{0};
    double endRange{0};
    double lateral{0};
    double z{0};
    double yawStartDeg{0};
    double yawEndDeg{0};
    double movementDuration{0};
    double holdDuration{0};
    std::string version;
    std::string checksum;
  };

  public: void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->node.Subscribe("/swarm/sim_time_trajectory/command",
        &SimTimeTrajectorySystem::OnCommand, this);
    if (const char *path = std::getenv("SWARM_SIM_TIME_TRAJECTORY_LOG"))
      this->logPath = path;
    gzmsg << "[swarm::simulation::SimTimeTrajectorySystem] Configure() called; "
          << "subscribed to /swarm/sim_time_trajectory/command; logPath="
          << (this->logPath.empty() ? "(unset)" : this->logPath) << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    std::optional<Command> incoming;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      incoming = std::move(this->pending);
      this->pending.reset();
    }
    if (incoming)
    {
      if (incoming->stop)
      {
        if (this->active && incoming->session == this->current.session)
          this->Log("trajectory_stopped", _info.simTime, 1.0, this->lastPose);
        this->active = false;
      }
      else
      {
        this->current = *incoming;
        this->startSimTime = _info.simTime;
        this->nextLogTime = _info.simTime;
        this->active = true;
        this->Log("trajectory_activated", _info.simTime, 0.0, this->PoseAt(0.0));
      }
    }
    if (!this->active || _info.paused)
      return;

    const double elapsed = std::chrono::duration<double>(_info.simTime-this->startSimTime).count();
    const auto pose = this->PoseAt(elapsed);
    auto entity = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name("x500_custom_1"));
    if (entity == gz::sim::kNullEntity)
      return;
    gz::sim::Model(entity).SetWorldPoseCmd(_ecm,
        gz::math::Pose3d(pose.x, this->current.lateral, this->current.z,
          0.0, 0.0, pose.yawRad));
    this->lastPose = pose;
    if (_info.simTime >= this->nextLogTime)
    {
      this->Log(elapsed >= this->current.movementDuration ? "trajectory_hold" : "trajectory_step",
          _info.simTime, pose.progress, pose);
      this->nextLogTime += std::chrono::milliseconds(200);
    }
    if (elapsed >= this->current.movementDuration+this->current.holdDuration)
    {
      this->Log("trajectory_completed", _info.simTime, 1.0, pose);
      this->active = false;
    }
  }

  private: struct Pose
  {
    double range{0};
    double x{0};
    double yawRad{0};
    double progress{0};
    double radialVelocity{0};
    double xVelocity{0};
    double xAcceleration{0};
  };

  private: Pose PoseAt(double _elapsed) const
  {
    Pose value;
    const double elapsed = std::max(0.0, _elapsed);
    value.progress = std::min(1.0, elapsed/this->current.movementDuration);
    value.range = this->current.startRange + value.progress*(this->current.endRange-this->current.startRange);
    value.x = std::sqrt(value.range*value.range-this->current.lateral*this->current.lateral);
    const double yawDeg = this->current.yawStartDeg + value.progress*(this->current.yawEndDeg-this->current.yawStartDeg);
    value.yawRad = yawDeg*M_PI/180.0;
    if (elapsed < this->current.movementDuration)
    {
      value.radialVelocity = (this->current.endRange-this->current.startRange)/this->current.movementDuration;
      value.xVelocity = value.range*value.radialVelocity/value.x;
      value.xAcceleration = -this->current.lateral*this->current.lateral*value.radialVelocity*value.radialVelocity/
          (value.x*value.x*value.x);
    }
    return value;
  }

  private: void OnCommand(const gz::msgs::StringMsg &_message)
  {
    std::vector<std::string> fields;
    std::stringstream stream(_message.data());
    std::string field;
    while (std::getline(stream, field, '|')) fields.push_back(field);
    try
    {
      Command command;
      if (fields.size() == 2 && fields[0] == "stop")
      {
        command.stop = true;
        command.session = fields[1];
      }
      else if (fields.size() == 12 && fields[0] == "start")
      {
        command.session = fields[1];
        command.startRange = std::stod(fields[2]);
        command.endRange = std::stod(fields[3]);
        command.lateral = std::stod(fields[4]);
        command.z = std::stod(fields[5]);
        command.yawStartDeg = std::stod(fields[6]);
        command.yawEndDeg = std::stod(fields[7]);
        command.movementDuration = std::stod(fields[8]);
        command.holdDuration = std::stod(fields[9]);
        command.version = fields[10];
        command.checksum = fields[11];
        if (command.movementDuration <= 0 || command.holdDuration < 0 ||
            command.startRange < 3 || command.startRange > 12 ||
            command.endRange < 3 || command.endRange > 12 ||
            std::abs(command.lateral) > 2 ||
            std::min(command.startRange, command.endRange) <= std::abs(command.lateral))
          return;
      }
      else return;
      std::lock_guard<std::mutex> guard(this->mutex);
      this->pending = command;  // bounded latest-value mailbox
    }
    catch (...) { return; }
  }

  private: void Log(const std::string &_kind,
      const std::chrono::steady_clock::duration &_simTime,
      double _progress, const Pose &_pose)
  {
    if (this->logPath.empty()) return;
    std::ofstream output(this->logPath, std::ios::app);
    output << "{\"kind\":\"" << _kind << "\",\"session\":\"" << this->current.session
      << "\",\"sim_timestamp_s\":" << std::chrono::duration<double>(_simTime).count()
      << ",\"progress\":" << _progress << ",\"range_m\":" << _pose.range
      << ",\"x_m\":" << _pose.x << ",\"y_m\":" << this->current.lateral
      << ",\"z_m\":" << this->current.z << ",\"yaw_rad\":" << _pose.yawRad
      << ",\"radial_velocity_m_s\":" << _pose.radialVelocity
      << ",\"x_velocity_m_s\":" << _pose.xVelocity
      << ",\"x_acceleration_m_s2\":" << _pose.xAcceleration
      << ",\"clock_domain\":\"gazebo_sim_time\",\"driver_version\":\"" << this->current.version
      << "\",\"contract_sha256\":\"" << this->current.checksum << "\"}\n";
  }

  private: gz::transport::Node node;
  private: std::mutex mutex;
  private: std::optional<Command> pending;
  private: Command current;
  private: bool active{false};
  private: std::chrono::steady_clock::duration startSimTime{0};
  private: std::chrono::steady_clock::duration nextLogTime{0};
  private: Pose lastPose;
  private: std::string logPath;
};
}

GZ_ADD_PLUGIN(swarm::simulation::SimTimeTrajectorySystem,
    gz::sim::System,
    swarm::simulation::SimTimeTrajectorySystem::ISystemConfigure,
    swarm::simulation::SimTimeTrajectorySystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(swarm::simulation::SimTimeTrajectorySystem,
    "swarm::simulation::SimTimeTrajectorySystem")
