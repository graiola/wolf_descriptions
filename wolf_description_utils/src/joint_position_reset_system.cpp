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

namespace wolf::sim::systems
{

class JointPositionReset final
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
  public: void Configure(
              const ignition::gazebo::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              ignition::gazebo::EntityComponentManager &_ecm,
              ignition::gazebo::EventManager & /*_eventMgr*/) override
  {
    this->model = ignition::gazebo::Model(_entity);
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
    }
  }

  public: void PreUpdate(
              const ignition::gazebo::UpdateInfo &,
              ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (this->done)
    {
      return;
    }

    std::vector<std::string> missingJoints;
    for (const auto &[jointName, position] : this->jointPositions)
    {
      const auto jointEntity = this->model.JointByName(_ecm, jointName);
      if (jointEntity == ignition::gazebo::kNullEntity)
      {
        missingJoints.push_back(jointName);
        continue;
      }

      ignition::gazebo::Joint joint(jointEntity);
      joint.ResetPosition(_ecm, {position});
    }

    if (missingJoints.empty())
    {
      std::clog << "[wolf_joint_position_reset_system] Applied "
                << this->jointPositions.size()
                << " initial joint position reset(s)." << std::endl;
      this->done = true;
      return;
    }

    ++this->attempts;
    if (this->attempts == 1 || this->attempts % 50 == 0)
    {
      std::cerr << "[wolf_joint_position_reset_system] Waiting for "
                << missingJoints.size()
                << " joint(s) to appear in the model." << std::endl;
    }

    if (this->attempts >= 200)
    {
      std::cerr << "[wolf_joint_position_reset_system] Giving up after "
                << this->attempts << " attempts." << std::endl;
      this->done = true;
    }
  }

  private: ignition::gazebo::Model model;
  private: std::unordered_map<std::string, double> jointPositions;
  private: std::size_t attempts{0};
  private: bool done{false};
};

}  // namespace wolf::sim::systems

IGNITION_ADD_PLUGIN(
    wolf::sim::systems::JointPositionReset,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    wolf::sim::systems::JointPositionReset,
    "wolf::sim::systems::JointPositionReset")
