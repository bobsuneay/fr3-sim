// Explicitly idealised Gazebo grasp assistance, NOT a friction/contact validator.
// A single fixed joint owns the bolt. Transfer validates the receiver first.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include "contact_evidence.hpp"

namespace fr3_inspection {
class AssistedGrasp : public gazebo::WorldPlugin {
  using SetBool = std_srvs::srv::SetBool;
  struct Pending {
    std::string side, message;
    bool close = false, done = false, success = false, expired = false;
  };
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  gazebo::event::ConnectionPtr contact_update_;
  std::array<ContactEvidence, 2> contacts_;
  gazebo::physics::JointPtr grasp_;
  std::array<rclcpp::Service<SetBool>::SharedPtr, 2> services_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fingers_;
  double last_finger_sample_ = -1.0;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::shared_ptr<Pending> pending_;
  std::string robot_, object_, owner_;

 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
    node_ = gazebo_ros::Node::Get(sdf);
    robot_ = sdf->Get<std::string>("robot_model");
    object_ = sdf->Get<std::string>("object_model");
    fingers_ = node_->create_publisher<sensor_msgs::msg::JointState>("gripper_states", 10);
    for (size_t i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      services_[i] = node_->create_service<SetBool>(side+"_grasp",
        [this, side](const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res) {
          std::unique_lock<std::mutex> lock(mutex_);
          if (pending_) { res->message = "Another operation is pending"; return; }
          auto p = std::make_shared<Pending>();
          p->side = side; p->close = req->data; pending_ = p;
          if (!cv_.wait_for(lock, std::chrono::seconds(2), [&p] { return p->done; })) {
            p->expired = true; pending_.reset();
            res->message = "Simulation did not advance; request expired"; return;
          }
          res->success = p->success; res->message = p->message;
        });
    }
    status_ = node_->create_service<std_srvs::srv::Trigger>("owner",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        res->success = true; res->message = owner_;
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) { Update(); });
    contact_update_ = gazebo::event::Events::ConnectWorldUpdateEnd([this]() { SampleContacts(); });
    RCLCPP_WARN(node_->get_logger(), "ASSISTED SIMULATION GRASP: sustained bilateral contact + synchronized fingers + fixed joint; not a friction validation");
  }

 private:
  void SampleContacts() {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = world_->SimTime().Double();
    auto robot = world_->ModelByName(robot_);
    auto object = world_->ModelByName(object_);
    if (!robot || !object) return;
    const auto bolt = object->GetLink("body");
    std::array<std::array<double, 2>, 2> depths{{{{-1, -1}}, {{-1, -1}}}};
    auto manager = world_->Physics()->GetContactManager();
    for (unsigned int i=0; i<manager->GetContactCount(); ++i) {
      const auto contact = manager->GetContact(i);
      if (!contact || !contact->collision1 || !contact->collision2 || !contact->count) continue;
      auto a = contact->collision1->GetLink(), b = contact->collision2->GetLink();
      auto other = a == bolt ? b : (b == bolt ? a : gazebo::physics::LinkPtr());
      if (!other) continue;
      double depth = 0;
      for (unsigned int k=0; k<contact->count; ++k) depth = std::max(depth, contact->depths[k]);
      for (size_t s=0; s<2; ++s) for (size_t f=0; f<2; ++f) {
        const std::string name = std::string(s == 0 ? "left" : "right")+
          (f == 0 ? "_left_finger" : "_right_finger");
        if (other != robot->GetLink(name)) continue;
        depths[s][f] = std::max(depths[s][f], depth);
      }
    }
    for (size_t s=0; s<2; ++s) contacts_[s].sample(now, depths[s]);
  }
  void PublishFingers() {
    const auto stamp = world_->SimTime();
    const double now = stamp.Double();
    if (last_finger_sample_ >= 0 && now >= last_finger_sample_ &&
        now-last_finger_sample_ < 1.0/30.0) return;
    last_finger_sample_ = now;
    auto robot = world_->ModelByName(robot_);
    if (!robot) return;  // The world plugin loads before spawn_entity finishes.
    sensor_msgs::msg::JointState state;
    state.header.stamp.sec = stamp.sec;
    state.header.stamp.nanosec = stamp.nsec;
    for (const auto & side : {std::string("left"), std::string("right")}) {
      for (const auto & finger : {std::string("left"), std::string("right")}) {
        const auto name = side+"_"+finger+"_finger_joint";
        auto joint = robot->GetJoint(name);
        if (!joint) continue;
        const double position = joint->Position(0);
        if (!std::isfinite(position)) continue;
        state.name.push_back(name);
        state.position.push_back(position);
        state.velocity.push_back(joint->GetVelocity(0));
      }
    }
    // This is actual Gazebo physics state, independent of mimic interface
    // aliases. Missing joints are omitted and expire at the consumers.
    if (!state.name.empty()) fingers_->publish(state);
  }

  bool FingerFeedbackReady(const std::string & side, gazebo::physics::ModelPtr robot) {
    // Size-independent gate. Actual bilateral part contact is checked separately.
    auto left = robot->GetJoint(side+"_left_finger_joint");
    auto right = robot->GetJoint(side+"_right_finger_joint");
    if (!left || !right) return false;
    const double a = left->Position(0), b = right->Position(0);
    return std::isfinite(a) && std::isfinite(b) && a >= 0 && b >= 0 &&
           std::abs(a-b) <= .001;
  }
  void Update() {
    PublishFingers();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->expired) return;
    auto p = pending_;
    try {
      auto robot = world_->ModelByName(robot_);
      auto object = world_->ModelByName(object_);
      if (!robot || !object) throw std::runtime_error("Robot/bolt model missing");
      auto palm = robot->GetLink(p->side+"_gripper_palm");
      auto bolt = object->GetLink("body");
      if (!palm || !bolt) throw std::runtime_error("Required physical link missing");
      if (p->close) {
        const size_t index = p->side == "left" ? 0 : 1;
        const double now = world_->SimTime().Double();
        const auto &contact = contacts_[index].seen;
        RCLCPP_INFO(node_->get_logger(), "Grasp check %s: finger contact age=(%.3f, %.3f) s, duration=(%.3f, %.3f) s",
          p->side.c_str(), now-contact[0], now-contact[1],
          contact[0]-contacts_[index].since[0], contact[1]-contacts_[index].since[1]);
        if (!contacts_[index].ready(now))
          throw std::runtime_error("Require >=100 ms sustained contact on BOTH fingers with part; no attachment");
        if (!FingerFeedbackReady(p->side, robot))
          throw std::runtime_error("Invalid or unsynchronized physical finger feedback");
        if (owner_ != p->side) {
          auto next = world_->Physics()->CreateJoint("fixed", robot);
          if (!next) throw std::runtime_error("Cannot create assisted grasp joint");
          next->SetName("inspection_grasp_"+p->side);
          next->Attach(palm, bolt);
          // Joint anchor in child frame. Keeps the CURRENT relative pose; no snapping.
          next->Load(palm, bolt, ignition::math::Pose3d::Zero);
          next->SetModel(object);
          next->Init();
          if (grasp_) { grasp_->Detach(); grasp_->Fini(); }
          grasp_ = next; owner_ = p->side;
        }
      } else if (owner_ == p->side) {
        if (grasp_) { grasp_->Detach(); grasp_->Fini(); grasp_.reset(); }
        owner_.clear();
      }
      p->success = true; p->message = owner_;
    } catch (const std::exception & e) {
      p->message = e.what();
    }
    p->done = true; pending_.reset(); cv_.notify_all();
  }
};
GZ_REGISTER_WORLD_PLUGIN(AssistedGrasp)
}  // namespace fr3_inspection
