// dynamic_gripper_plugin.cpp
// Ignition Fortress (Gazebo Fortress) 버전용 동적 Attach/Detach 플러그인
// COMPLETE VERSION with Detached Objects Blacklist + Detailed Logging

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Sensor.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/common/Console.hh>

#include <memory>
#include <string>
#include <optional>
#include <mutex>
#include <set>

namespace dynamic_gripper
{
  class DynamicGripperPlugin : 
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
  public:
    void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &) override
    {
      ignmsg << "DynamicGripperPlugin Configure START" << std::endl;
      
      model_ = ignition::gazebo::Model(_entity);
      
      if (!_sdf->HasElement("parent_link"))
      {
        ignerr << "DynamicGripperPlugin: Missing <parent_link> parameter" << std::endl;
        return;
      }
      
      std::string parent_link_name = _sdf->Get<std::string>("parent_link");
      ignmsg << "Looking for parent link: " << parent_link_name << std::endl;
      
      gripper_link_ = model_.LinkByName(_ecm, parent_link_name);
      
      if (gripper_link_ == ignition::gazebo::kNullEntity)
      {
        ignerr << "DynamicGripperPlugin: Could not find parent link: " 
              << parent_link_name << std::endl;
        return;
      }
      
      ignmsg << "Parent link found! Entity ID: " << gripper_link_ << std::endl;

      // ign_node_ 생성
      ignmsg << "Creating Ignition Transport Node..." << std::endl;
      ign_node_ = std::make_shared<ignition::transport::Node>();
      ignmsg << "Ignition Transport Node created!" << std::endl;

      // Contact 센서 subscribe
      if (_sdf->HasElement("contact_sensor_topic"))
      {
        std::string contact_topic = _sdf->Get<std::string>("contact_sensor_topic");
        ignmsg << "Subscribing to contact sensor: " << contact_topic << std::endl;
        bool result = ign_node_->Subscribe(contact_topic, &DynamicGripperPlugin::OnContact, this);
        ignmsg << "Contact sensor subscription result: " << (result ? "SUCCESS" : "FAILED") << std::endl;
      }
      else
      {
        ignwarn << "No contact_sensor_topic specified!" << std::endl;
      }
      
      std::string attach_topic = "/tm12_gripper/attach";
      std::string detach_topic = "/tm12_gripper/detach";
      std::string state_topic = "/tm12_gripper/state";
      
      if (_sdf->HasElement("attach_topic"))
        attach_topic = _sdf->Get<std::string>("attach_topic");
      if (_sdf->HasElement("detach_topic"))
        detach_topic = _sdf->Get<std::string>("detach_topic");
      if (_sdf->HasElement("state_topic"))
        state_topic = _sdf->Get<std::string>("state_topic");

      ignmsg << "Subscribing to attach topic: " << attach_topic << std::endl;
      bool attach_result = ign_node_->Subscribe(attach_topic, &DynamicGripperPlugin::OnAttachRequest, this);
      ignmsg << "Attach subscription result: " << (attach_result ? "SUCCESS" : "FAILED") << std::endl;
      
      ignmsg << "Subscribing to detach topic: " << detach_topic << std::endl;
      bool detach_result = ign_node_->Subscribe(detach_topic, &DynamicGripperPlugin::OnDetachRequest, this);
      ignmsg << "Detach subscription result: " << (detach_result ? "SUCCESS" : "FAILED") << std::endl;
      
      ignmsg << "Advertising state topic: " << state_topic << std::endl;
      state_pub_ = ign_node_->Advertise<ignition::msgs::Boolean>(state_topic);
      ignmsg << "State publisher created!" << std::endl;
      
      attach_distance_ = _sdf->Get<double>("attach_distance", 0.01).first;
      
      ignmsg << "DynamicGripperPlugin: Initialized successfully" << std::endl;
      ignmsg << "Parent link: " << parent_link_name << std::endl;
      ignmsg << "Attach topic: " << attach_topic << std::endl;
      ignmsg << "Detach topic: " << detach_topic << std::endl;
      ignmsg << "State topic: " << state_topic << std::endl;
      ignmsg << "Attach distance: " << attach_distance_ << std::endl;
    }

    void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override
    {
      if (_info.paused)
        return;

      std::lock_guard<std::mutex> lock(state_mutex_);

      if (attach_requested_ && !is_attached_)
      {
        ignmsg << "ATTACH PROCESSING" << std::endl;
        
        std::string target_model;
        {
          std::lock_guard<std::mutex> contact_lock(contact_mutex_);
          ignmsg << "Current contact_model_name_: '" << contact_model_name_ << "'" << std::endl;
          
          if (contact_model_name_.empty())
          {
            ignwarn << "DynamicGripperPlugin: No object in contact, cannot attach" << std::endl;
            attach_requested_ = false;
            return;
          }
          
          // 이미 detach한 물체는 재연결 금지!
          if (detached_objects_.count(contact_model_name_) > 0)
          {
            ignerr << "CANNOT ATTACH to already detached object: " << contact_model_name_ << std::endl;
            ignerr << "   This object was previously placed and cannot be picked up again!" << std::endl;
            attach_requested_ = false;
            return;
          }
          
          target_model = contact_model_name_;
        }

        ignmsg << "Searching for model: " << target_model << std::endl;
        
        auto target_model_entity = _ecm.EntityByComponents(
          ignition::gazebo::components::Name(target_model),
          ignition::gazebo::components::Model());
          
        if (target_model_entity == ignition::gazebo::kNullEntity)
        {
          ignwarn << "DynamicGripperPlugin: Could not find model: " 
                 << target_model << std::endl;
          attach_requested_ = false;
          return;
        }

        ignmsg << "Model found! Entity ID: " << target_model_entity << std::endl;
        ignmsg << "Searching for link in model..." << std::endl;
        
        auto target_link = FindLinkInModel(_ecm, target_model_entity);
        
        if (target_link != ignition::gazebo::kNullEntity)
        {
          ignmsg << "Target link found! Creating DetachableJoint..." << std::endl;
          ignmsg << "Gripper Link Entity: " << gripper_link_ << std::endl;
          ignmsg << "Target Model: " << target_model << std::endl;
          ignmsg << "Target Model Entity: " << target_model_entity << std::endl;
          ignmsg << "Target Link Entity: " << target_link << std::endl;

          joint_entity_ = _ecm.CreateEntity();
          _ecm.CreateComponent(joint_entity_,
            ignition::gazebo::components::DetachableJoint({
              gripper_link_,
              target_link,
              "fixed"
            }));
          
          is_attached_ = true;
          attached_model_name_ = target_model;
          attached_model_entity_ = target_model_entity;
          attached_link_entity_ = target_link;

          ignmsg << "SUCCESSFULLY ATTACHED " << std::endl;
          ignmsg << "Model: " << target_model << std::string(44 - target_model.length(), ' ') << "║" << std::endl;
          ignmsg << "Model Entity ID: " << target_model_entity << std::string(33, ' ') << "║" << std::endl;
          ignmsg << "Link Entity ID: " << target_link << std::string(34, ' ') << "║" << std::endl;
          
          PublishState(true);
        }
        else
        {
          ignwarn << "DynamicGripperPlugin: Could not find link in model: " 
                 << target_model << std::endl;
        }
        
        attach_requested_ = false;
      }
      
      if (detach_requested_ && is_attached_)
      {
        ignmsg << "DETACH PROCESSING" << std::endl;
        ignmsg << "Detaching from: " << attached_model_name_ << std::endl;
        ignmsg << "Model Entity ID: " << attached_model_entity_ << std::endl;
        ignmsg << "Link Entity ID: " << attached_link_entity_ << std::endl;
        
        _ecm.RequestRemoveEntity(joint_entity_);
        joint_entity_ = ignition::gazebo::kNullEntity;
        is_attached_ = false;
        
        // Detached 물체 블랙리스트에 추가!
        detached_objects_.insert(attached_model_name_);
        
        ignmsg << "SUCCESSFULLY DETACHED" << std::endl;
        ignmsg << "Model: " << attached_model_name_ << std::string(44 - attached_model_name_.length(), ' ') << "║" << std::endl;
        ignmsg << "Added to BLACKLIST (cannot re-attach)" << std::endl;
        
        ignmsg << "Current Blacklist (" << detached_objects_.size() << " objects):" << std::endl;
        for (const auto& obj : detached_objects_)
        {
          ignmsg << "  - " << obj << std::endl;
        }
        
        attached_model_name_.clear();
        attached_model_entity_ = ignition::gazebo::kNullEntity;
        attached_link_entity_ = ignition::gazebo::kNullEntity;
        
        // Contact 정보 클리어
        {
          std::lock_guard<std::mutex> contact_lock(contact_mutex_);
          contact_model_name_.clear();
          ignmsg << "Cleared contact_model_name_ after detach" << std::endl;
        }
        
        PublishState(false);
        
        detach_requested_ = false;
      }
    }

    void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override
    {
      static int counter = 0;
      if (++counter % 100 == 0)
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        PublishState(is_attached_);
      }
    }

  private:
    void OnContact(const ignition::msgs::Contacts &_msg)
    {
      std::lock_guard<std::mutex> lock(contact_mutex_);
      
      contact_model_name_.clear();
      
      if (_msg.contact_size() == 0)
        return;

      std::string collision_name = _msg.contact(0).collision2().name();
      
      size_t first_delimiter = collision_name.find("::");
      if (first_delimiter != std::string::npos)
      {
        contact_model_name_ = collision_name.substr(0, first_delimiter);

        if (detached_objects_.count(contact_model_name_) > 0)
        {
          igndbg << "Contact detected with BLACKLISTED object: " << contact_model_name_ 
                << "(will NOT attach)" << std::endl;
        }
        else
        {
          igndbg << " Contact detected: " << contact_model_name_ << " (can attach)" << std::endl;
        }
      }
    }
    
    void OnAttachRequest(const ignition::msgs::Empty &)
    {
      ignmsg << "ATTACH REQUEST RECEIVED" << std::endl;
      
      std::lock_guard<std::mutex> lock(state_mutex_);
      
      ignmsg << "Current state:" << std::endl;
      ignmsg << "  is_attached_ = " << (is_attached_ ? "true" : "false") << std::endl;
      
      {
        std::lock_guard<std::mutex> contact_lock(contact_mutex_);
        ignmsg << "  contact_model_name_ = '" << contact_model_name_ << "'" << std::endl;
        
        if (!contact_model_name_.empty() && detached_objects_.count(contact_model_name_) > 0)
        {
          ignmsg << "  Contact is in BLACKLIST - will be rejected!" << std::endl;
        }
      }
      
      ignmsg << "  attached_model_name_ = '" << attached_model_name_ << "'" << std::endl;
      ignmsg << "  blacklist size = " << detached_objects_.size() << std::endl;
      
      if (!is_attached_)
      {
        attach_requested_ = true;
        ignmsg << " Attach flag set to TRUE" << std::endl;
      }
      else
      {
        ignwarn << "Already attached to: " << attached_model_name_ << std::endl;
      }
    }
    
    void OnDetachRequest(const ignition::msgs::Empty &)
    {
      ignmsg << "DETACH REQUEST RECEIVED" << std::endl;
      
      std::lock_guard<std::mutex> lock(state_mutex_);
      
      if (is_attached_)
      {
        detach_requested_ = true;
        ignmsg << "Detach flag set to TRUE" << std::endl;
      }
      else
      {
        ignwarn << "Not holding any object" << std::endl;
      }
    }
    
    ignition::gazebo::Entity FindLinkInModel(
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::Entity model_entity)
    {
      ignition::gazebo::Model target_model(model_entity);

      std::vector<std::string> common_names = {"link", "base_link", "body", "box_link"};
      for (const auto& name : common_names)
      {
        auto link = target_model.LinkByName(_ecm, name);
        if (link != ignition::gazebo::kNullEntity)
        {
          ignmsg << "Found link '" << name << "' (Entity: " << link << ")" << std::endl;
          return link;
        }
      }

      auto links = target_model.Links(_ecm);
      if (!links.empty())
      {
        ignmsg << "Using first link in model (Entity: " << links[0] << ")" << std::endl;
        return links[0];
      }
      
      ignwarn << "No links found in model!" << std::endl;
      return ignition::gazebo::kNullEntity;
    }

    void PublishState(bool attached)
    {
      ignition::msgs::Boolean msg;
      msg.set_data(attached);
      state_pub_.Publish(msg);
      igndbg << "Published state: " << (attached ? "ATTACHED" : "DETACHED") << std::endl;
    }

    ignition::gazebo::Model model_;
    ignition::gazebo::Entity gripper_link_ = ignition::gazebo::kNullEntity;
    ignition::gazebo::Entity joint_entity_ = ignition::gazebo::kNullEntity;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    ignition::transport::Node::Publisher state_pub_;
    
    std::string contact_model_name_;
    std::string attached_model_name_;
    ignition::gazebo::Entity attached_model_entity_ = ignition::gazebo::kNullEntity;
    ignition::gazebo::Entity attached_link_entity_ = ignition::gazebo::kNullEntity;
    double attach_distance_;
    
    bool attach_requested_ = false;
    bool detach_requested_ = false;
    bool is_attached_ = false;

    std::set<std::string> detached_objects_;
    
    std::mutex state_mutex_;
    std::mutex contact_mutex_;
  };
}

IGNITION_ADD_PLUGIN(
  dynamic_gripper::DynamicGripperPlugin,
  ignition::gazebo::System,
  dynamic_gripper::DynamicGripperPlugin::ISystemConfigure,
  dynamic_gripper::DynamicGripperPlugin::ISystemPreUpdate,
  dynamic_gripper::DynamicGripperPlugin::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(
  dynamic_gripper::DynamicGripperPlugin,
  "dynamic_gripper::DynamicGripperPlugin"
)