#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace wolf_sim = gz::sim;
#define WOLF_ADD_PLUGIN GZ_ADD_PLUGIN
#define WOLF_ADD_PLUGIN_ALIAS GZ_ADD_PLUGIN_ALIAS

namespace wolf::sim::systems
{

class JointPositionReset final
    : public wolf_sim::System,
      public wolf_sim::ISystemConfigure,
      public wolf_sim::ISystemPreUpdate
{
  public: void Configure(
              const wolf_sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              wolf_sim::EntityComponentManager &_ecm,
              wolf_sim::EventManager & /*_eventMgr*/) override
  {
    this->model = wolf_sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      std::cerr << "[wolf_joint_position_reset_system] Attached entity is not a model."
                << std::endl;
      this->done = true;
      return;
    }

    if (!_sdf)
    {
      std::cerr << "[wolf_joint_position_reset_system] Missing plugin SDF."
                << std::endl;
      this->done = true;
      return;
    }

    if (!_sdf->HasElement("joint"))
    {
      this->done = true;
      return;
    }

    for (auto jointElem = _sdf->FindElement("joint");
         jointElem;
         jointElem = jointElem->GetNextElement("joint"))
    {
      if (!jointElem->HasElement("name") || !jointElem->HasElement("position"))
      {
        std::cerr << "[wolf_joint_position_reset_system] Ignoring malformed joint entry."
                  << std::endl;
        continue;
      }

      const auto jointName = jointElem->Get<std::string>("name");
      const auto position = jointElem->Get<double>("position");
      this->jointPositions.emplace(jointName, position);
    }

    if (this->jointPositions.empty())
    {
      this->done = true;
      return;
    }

    if (_sdf->HasElement("hold_iterations"))
    {
      auto holdIterations = _sdf->Get<int>("hold_iterations");
      if (holdIterations > 0)
      {
        this->holdIterations = static_cast<std::size_t>(holdIterations);
      }
    }

    if (_sdf->HasElement("settle_iterations"))
    {
      auto settleIterations = _sdf->Get<int>("settle_iterations");
      if (settleIterations > 0)
      {
        this->settleIterations = static_cast<std::size_t>(settleIterations);
      }
    }

    if (_sdf->HasElement("position_tolerance"))
    {
      auto tolerance = _sdf->Get<double>("position_tolerance");
      if (tolerance > 0.0)
      {
        this->positionTolerance = tolerance;
      }
    }

    this->maxApplyIterations = std::max(this->holdIterations * 2u, this->holdIterations + this->settleIterations);
  }

  public: void PreUpdate(
              const wolf_sim::UpdateInfo &,
              wolf_sim::EntityComponentManager &_ecm) override
  {
    if (this->done)
    {
      return;
    }

    std::vector<std::string> missingJoints;
    for (const auto &[jointName, position] : this->jointPositions)
    {
      (void) position;
      const auto jointEntity = this->model.JointByName(_ecm, jointName);
      if (jointEntity == wolf_sim::kNullEntity)
      {
        missingJoints.push_back(jointName);
        continue;
      }
    }

    if (missingJoints.empty())
    {
      bool allWithinTolerance = true;

      for (const auto &[jointName, position] : this->jointPositions)
      {
        const auto jointEntity = this->model.JointByName(_ecm, jointName);
        if (jointEntity == wolf_sim::kNullEntity)
        {
          allWithinTolerance = false;
          continue;
        }

        wolf_sim::Joint joint(jointEntity);
        joint.EnablePositionCheck(_ecm, true);
        joint.ResetPosition(_ecm, {position});
        joint.ResetVelocity(_ecm, {0.0});

        const auto currentPosition = joint.Position(_ecm);
        if (!currentPosition || currentPosition->empty())
        {
          allWithinTolerance = false;
          continue;
        }

        const auto error = std::abs(currentPosition->front() - position);
        if (error > this->positionTolerance)
        {
          allWithinTolerance = false;
        }
      }

      if (!this->startedApplying)
      {
        std::clog << "[wolf_joint_position_reset_system] Applying "
                  << this->jointPositions.size()
                  << " initial joint position reset(s) for "
                  << this->holdIterations << " iteration(s) minimum."
                  << " Tolerance: " << this->positionTolerance
                  << ", settle iterations: " << this->settleIterations
                  << "." << std::endl;
        this->startedApplying = true;
      }

      ++this->appliedIterations;
      if (allWithinTolerance)
      {
        ++this->convergedIterations;
      }
      else
      {
        this->convergedIterations = 0;
      }

      if (this->appliedIterations >= this->holdIterations &&
          this->convergedIterations >= this->settleIterations)
      {
        std::clog << "[wolf_joint_position_reset_system] Applied "
                  << this->jointPositions.size()
                  << " initial joint position reset(s)." << std::endl;
        this->done = true;
      }
      else if (this->appliedIterations >= this->maxApplyIterations)
      {
        std::cerr << "[wolf_joint_position_reset_system] Stopping after "
                  << this->appliedIterations
                  << " iterations without stable convergence." << std::endl;
        this->done = true;
      }
      return;
    }

    ++this->attempts;
    if (this->attempts == 1 || this->attempts % 50 == 0)
    {
      std::cerr << "[wolf_joint_position_reset_system] Waiting for "
                << missingJoints.size()
                << " joint(s) to appear in the model." << std::endl;
    }

    if (this->attempts >= 1000)
    {
      std::cerr << "[wolf_joint_position_reset_system] Giving up after "
                << this->attempts << " attempts." << std::endl;
      this->done = true;
    }
  }

  private: wolf_sim::Model model;
  private: std::unordered_map<std::string, double> jointPositions;
  private: std::size_t attempts{0};
  private: std::size_t holdIterations{10000};
  private: std::size_t settleIterations{100};
  private: std::size_t maxApplyIterations{20000};
  private: std::size_t appliedIterations{0};
  private: std::size_t convergedIterations{0};
  private: double positionTolerance{0.02};
  private: bool startedApplying{false};
  private: bool done{false};
};

}  // namespace wolf::sim::systems

WOLF_ADD_PLUGIN(
    wolf::sim::systems::JointPositionReset,
    wolf_sim::System,
    wolf_sim::ISystemConfigure,
    wolf_sim::ISystemPreUpdate)

WOLF_ADD_PLUGIN_ALIAS(
    wolf::sim::systems::JointPositionReset,
    "wolf::sim::systems::JointPositionReset")
