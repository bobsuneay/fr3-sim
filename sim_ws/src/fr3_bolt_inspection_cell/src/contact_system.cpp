// Retain GazeboSystem for arms; own both fingers so neither takes its SetPosition path.
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mutex>
#include "linked_finger_servo.hpp"

namespace fr3_inspection {
class ContactSystem : public gazebo_ros2_control::GazeboSystemInterface {
  using Base = gazebo_ros2_control::GazeboSystemInterface;
  using Return = hardware_interface::return_type;
  using Callback = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  pluginlib::ClassLoader<Base> loader_{"gazebo_ros2_control", "gazebo_ros2_control::GazeboSystemInterface"};
  std::shared_ptr<Base> arms_;
  hardware_interface::HardwareInfo arm_info_;
  struct Pair {
    std::string master;
    std::array<gazebo::physics::JointPtr, 2> joint;
    std::array<double, 2> q{}, v{}, force{};
    double command = 0, target = 0;
    LinkedFingerServo servo;
  };
  std::array<Pair, 2> pairs_;
  gazebo::event::ConnectionPtr update_;
  double last_time_ = -1;
  std::mutex mutex_;
  static bool finger(const std::string &name) {
    return name.find("_finger_joint") != std::string::npos;
  }
  static std::vector<std::string> arm_modes(const std::vector<std::string> &modes) {
    std::vector<std::string> out;
    for (const auto &name : modes) if (!finger(name)) out.push_back(name);
    return out;
  }
 public:
  ~ContactSystem() override { update_.reset(); }
  bool initSim(rclcpp::Node::SharedPtr &node, gazebo::physics::ModelPtr model,
      const hardware_interface::HardwareInfo &info, sdf::ElementPtr sdf) override {
    nh_ = node;
    arm_info_ = info;
    auto &joints = arm_info_.joints;
    joints.erase(std::remove_if(joints.begin(), joints.end(),
      [](const auto &j) { return finger(j.name); }), joints.end());
    arms_ = loader_.createSharedInstance("gazebo_ros2_control/GazeboSystem");
    if (!arms_->initSim(node, model, arm_info_, sdf)) return false;
    for (size_t s=0; s<2; ++s) {
      const std::string side = s == 0 ? "left" : "right";
      auto &p = pairs_[s];
      p.master = side+"_left_finger_joint";
      p.servo.max_force = std::stod(info.hardware_parameters.at("finger_max_force"));
      p.servo.max_speed = std::stod(info.hardware_parameters.at("finger_max_speed"));
      if (!std::isfinite(p.servo.max_force) || p.servo.max_force <= 0 || p.servo.max_force > 10 ||
          !std::isfinite(p.servo.max_speed) || p.servo.max_speed <= 0 || p.servo.max_speed > .02)
        return false;
      for (size_t f=0; f<2; ++f) {
        const auto name = side+(f == 0 ? "_left_finger_joint" : "_right_finger_joint");
        p.joint[f] = model->GetJoint(name);
        if (!p.joint[f]) { RCLCPP_ERROR(node->get_logger(), "Missing physical finger %s", name.c_str()); return false; }
        const auto config = std::find_if(info.joints.begin(), info.joints.end(),
          [&name](const auto &j) { return j.name == name; });
        if (config == info.joints.end()) return false;
        const auto pos = std::find_if(config->state_interfaces.begin(), config->state_interfaces.end(),
          [](const auto &i) { return i.name == "position"; });
        if (pos == config->state_interfaces.end()) return false;
        const double initial = std::stod(pos->initial_value);
        if (!std::isfinite(initial) || initial < 0 || initial > .05) return false;
        // Spawn only. During all subsequent motion, exclusively apply bounded force.
        p.joint[f]->SetPosition(0, initial, true);
        p.q[f] = initial;
      }
      p.command = p.target = p.servo.reference = p.q[0];
    }
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &info) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = info.simTime.Double();
        const double dt = last_time_ < 0 ? 0 : now-last_time_;
        last_time_ = now;
        for (auto &p : pairs_) {
          std::array<double, 2> q, v;
          for (size_t f=0; f<2; ++f) { q[f]=p.joint[f]->Position(0); v[f]=p.joint[f]->GetVelocity(0); }
          if (dt <= 0 || dt > .1) p.servo.reference = p.target = (q[0]+q[1])/2;
          p.force = p.servo.step(p.target, q, v, dt);
          for (size_t f=0; f<2; ++f) p.joint[f]->SetForce(0, p.force[f]);
        }
      });
    RCLCPP_INFO(node->get_logger(), "CONTACT GRIPPER: one target per pair, force <= %.1f N/finger, speed <= %.1f mm/s; no position teleport",
      pairs_[0].servo.max_force, pairs_[0].servo.max_speed*1000);
    return true;
  }
  Callback on_init(const hardware_interface::HardwareInfo &info) override {
    if (Base::on_init(info) != Callback::SUCCESS) return Callback::ERROR;
    return arms_->on_init(arm_info_);
  }
  Callback on_activate(const rclcpp_lifecycle::State &s) override { return arms_->on_activate(s); }
  Callback on_deactivate(const rclcpp_lifecycle::State &s) override {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &p : pairs_) p.target = p.command = p.servo.reference = (p.q[0]+p.q[1])/2;
    return arms_->on_deactivate(s);
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    auto out = arms_->export_state_interfaces();
    for (auto &p : pairs_) for (size_t f=0; f<2; ++f) {
      auto name = p.master;
      if (f) name.replace(name.find("_left_finger"), 12, "_right_finger");
      out.emplace_back(name, "position", &p.q[f]);
      out.emplace_back(name, "velocity", &p.v[f]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    auto out = arms_->export_command_interfaces();
    for (auto &p : pairs_) out.emplace_back(p.master, "position", &p.command);
    return out;
  }
  Return perform_command_mode_switch(const std::vector<std::string> &start,
                                     const std::vector<std::string> &stop) override {
    for (auto &p : pairs_) if (std::find(stop.begin(), stop.end(), p.master+"/position") != stop.end()) {
      std::lock_guard<std::mutex> lock(mutex_);
      p.target = p.command = p.servo.reference = (p.q[0]+p.q[1])/2;
    }
    return arms_->perform_command_mode_switch(arm_modes(start), arm_modes(stop));
  }
  Return read(const rclcpp::Time &t, const rclcpp::Duration &dt) override {
    const auto result = arms_->read(t, dt);
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &p : pairs_) for (size_t f=0; f<2; ++f) {
      p.q[f]=p.joint[f]->Position(0); p.v[f]=p.joint[f]->GetVelocity(0);
    }
    return result;
  }
  Return write(const rclcpp::Time &t, const rclcpp::Duration &dt) override {
    const auto result = arms_->write(t, dt);
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &p : pairs_) {
      if (!std::isfinite(p.command)) { p.target=p.servo.reference=(p.q[0]+p.q[1])/2; return Return::ERROR; }
      p.target = std::clamp(p.command, 0.0, .05);
    }
    return result;
  }
};
}
PLUGINLIB_EXPORT_CLASS(fr3_inspection::ContactSystem, gazebo_ros2_control::GazeboSystemInterface)
