#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <franka/exception.h>
#include <franka/gripper.h>

class GripperListener : public rclcpp::Node {
public:
  GripperListener()
    : Node("gripper_listener"),
      gripper_("192.168.1.100")  // Initialisation de l'attribut gripper
  {
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "gripper_command", 10,
      std::bind(&GripperListener::callback, this, std::placeholders::_1));
    
    try {
      gripper_.move(0.08, 0.1);  // Ouvrir au démarrage
    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Erreur init gripper: %s", e.what());
    }
  }

private:
  void callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Received TRUE. Attempting to grasp.");
      try {
        double grasping_width = 0.035;
        if (!gripper_.grasp(0, 0.1, 30)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to grasp object.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Grasp succeeded.");
        }
        gripper_.stop();
      } catch (const franka::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Franka exception: %s", e.what());
      }

      // Si tu veux juste agir une fois, tu peux shutdown ici :
      // rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  franka::Gripper gripper_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperListener>());
  return 0;
}
