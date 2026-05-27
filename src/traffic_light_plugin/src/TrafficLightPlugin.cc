#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/msgs/color.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <sdf/Material.hh>
#include <chrono>
#include <string>

namespace traffic_light
{

enum class Phase { RED = 0, GREEN = 1, YELLOW = 2 };

class TrafficLightPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->modelName = this->model.Name(_ecm);

    if (_sdf->HasElement("cycle_duration"))
      this->cycleDuration = _sdf->Get<double>("cycle_duration");

    // 월드 이름 찾기
    auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
    if (worldEntity != gz::sim::kNullEntity)
    {
      auto nameComp = _ecm.Component<gz::sim::components::Name>(worldEntity);
      if (nameComp) this->worldName = nameComp->Data();
    }

    gzmsg << "[TrafficLight] cycle_duration = "
          << this->cycleDuration << "s, world = "
          << this->worldName << "\n";

    auto linkEntity = this->model.LinkByName(_ecm, "link");
    if (linkEntity == gz::sim::kNullEntity) {
      gzerr << "[TrafficLight] 'link' not found.\n";
      return;
    }

    // visual 엔티티 + 이름 캐싱 (서비스 호출 시 이름 필요)
    this->redVisual    = this->FindVisual(_ecm, linkEntity, "red");
    this->yellowVisual = this->FindVisual(_ecm, linkEntity, "yellow");
    this->greenVisual  = this->FindVisual(_ecm, linkEntity, "green");

    if (this->redVisual    == gz::sim::kNullEntity ||
        this->yellowVisual == gz::sim::kNullEntity ||
        this->greenVisual  == gz::sim::kNullEntity)
    {
      gzerr << "[TrafficLight] red/yellow/green visual not found.\n";
      return;
    }

    this->initialized = true;
    gzmsg << "[TrafficLight] 초기화 완료 — 🔴 RED\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->initialized || _info.paused) return;

    double t = std::chrono::duration<double>(_info.simTime).count();
    double period = this->cycleDuration * 3.0;
    double phase_t = std::fmod(t, period);

    Phase newPhase;
    if      (phase_t < this->cycleDuration)        newPhase = Phase::RED;
    else if (phase_t < this->cycleDuration * 2.0)  newPhase = Phase::GREEN;
    else                                            newPhase = Phase::YELLOW;

    if (newPhase != this->currentPhase)
    {
      this->currentPhase = newPhase;
      this->ApplyPhaseViaService(_ecm, newPhase);

      const char *names[] = {"🔴 RED", "🟢 GREEN", "🟡 YELLOW"};
      gzmsg << "[TrafficLight] → "
            << names[static_cast<int>(newPhase)] << "\n";
    }
  }

private:
  gz::sim::Entity FindVisual(
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::Entity _linkEntity,
      const std::string &_name)
  {
    gz::sim::Entity result = gz::sim::kNullEntity;
    _ecm.Each<gz::sim::components::Visual,
              gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Visual *,
          const gz::sim::components::Name *_nameComp) -> bool
      {
        if (_nameComp->Data() == _name &&
            _ecm.ParentEntity(_entity) == _linkEntity)
        {
          result = _entity;
          return false;
        }
        return true;
      });
    return result;
  }

  // visual_config 서비스로 material 변경 메시지 전송
  void SetVisualColor(
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::Entity _visualEntity,
      const std::string &_visualName,
      float r, float g, float b,
      bool _isOn)
  {
    gz::msgs::Visual req;
    req.set_id(_visualEntity);
    req.set_name(_visualName);
    req.set_parent_name(this->modelName);

    auto *mat = req.mutable_material();

    auto *ambient = mat->mutable_ambient();
    auto *diffuse = mat->mutable_diffuse();
    auto *specular = mat->mutable_specular();
    auto *emissive = mat->mutable_emissive();

    if (_isOn) {
      ambient->set_r(r * 0.4f);  ambient->set_g(g * 0.4f);  ambient->set_b(b * 0.4f);  ambient->set_a(1);
      diffuse->set_r(r);         diffuse->set_g(g);         diffuse->set_b(b);         diffuse->set_a(1);
      specular->set_r(r);        specular->set_g(g);        specular->set_b(b);        specular->set_a(1);
      emissive->set_r(r);        emissive->set_g(g);        emissive->set_b(b);        emissive->set_a(1);
    } else {
      ambient->set_r(r * 0.05f); ambient->set_g(g * 0.05f); ambient->set_b(b * 0.05f); ambient->set_a(1);
      diffuse->set_r(r * 0.1f);  diffuse->set_g(g * 0.1f);  diffuse->set_b(b * 0.1f);  diffuse->set_a(1);
      specular->set_r(0);        specular->set_g(0);        specular->set_b(0);        specular->set_a(1);
      emissive->set_r(0);        emissive->set_g(0);        emissive->set_b(0);        emissive->set_a(1);
    }

    std::string service = "/world/" + this->worldName + "/visual_config";
    gz::msgs::Boolean rep;
    bool result = false;
    unsigned int timeout = 200;
    this->node.Request(service, req, timeout, rep, result);
  }

  void ApplyPhaseViaService(
      gz::sim::EntityComponentManager &_ecm,
      Phase _phase)
  {
    SetVisualColor(_ecm, this->redVisual,    "red",    1, 0, 0, _phase == Phase::RED);
    SetVisualColor(_ecm, this->yellowVisual, "yellow", 1, 0.85, 0, _phase == Phase::YELLOW);
    SetVisualColor(_ecm, this->greenVisual,  "green",  0, 1, 0, _phase == Phase::GREEN);
  }

  // 멤버
  gz::sim::Model  model;
  std::string     modelName;
  std::string     worldName {"default"};
  gz::sim::Entity redVisual    {gz::sim::kNullEntity};
  gz::sim::Entity yellowVisual {gz::sim::kNullEntity};
  gz::sim::Entity greenVisual  {gz::sim::kNullEntity};

  gz::transport::Node node;

  double cycleDuration {10.0};
  Phase  currentPhase  {Phase::RED};
  bool   initialized   {false};
};

}  // namespace traffic_light

GZ_ADD_PLUGIN(
  traffic_light::TrafficLightPlugin,
  gz::sim::System,
  traffic_light::TrafficLightPlugin::ISystemConfigure,
  traffic_light::TrafficLightPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  traffic_light::TrafficLightPlugin,
  "traffic_light::TrafficLightPlugin")
