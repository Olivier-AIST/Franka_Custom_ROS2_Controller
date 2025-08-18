  // Copyright (c) 2023 Franka Robotics GmbH
  //
  // Licensed under the Apache License, Version 2.0 (the "License");
  // you may not use this file except in compliance with the License.
  // You may obtain a copy of the License at
  //
  //     http://www.apache.org/licenses/LICENSE-2.0
  //
  // Unless required by applicable law or agreed to in writing, software
  // distributed under the License is distributed on an "AS IS" BASIS,
  // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  // See the License for the specific language governing permissions and
  // limitations under the License.

  #include <franka_example_controllers/joint_position_example_controller.hpp>
  #include <franka_example_controllers/robot_utils.hpp>

  #include <cassert>
  #include <cmath>
  #include <exception>
  #include <string>

  #include <Eigen/Eigen>

  namespace franka_example_controllers {

  controller_interface::InterfaceConfiguration
  JointPositionExampleController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration
  JointPositionExampleController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }

    // add the robot time interface
    if (!is_gazebo_) {
      config.names.push_back(arm_id_ + "/robot_time");
    }

    return config;
  }

  controller_interface::return_type JointPositionExampleController::update(
      const rclcpp::Time& /*time*/,
      const rclcpp::Duration& /*period*/) {
    if (initialization_flag_) {
      for (int i = 0; i < num_joints; ++i) {
        initial_q_.at(i) = state_interfaces_[i].get_value();
      }

      initialization_flag_ = false;
      if (!is_gazebo_) {
        initial_robot_time_ = state_interfaces_.back().get_value();

      }
      elapsed_time_ = 0.0;
    }
    else {
    if (!is_gazebo_) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    }
    
    else {
      
      elapsed_time_ += trajectory_period_;
    }}

    // Ajout ici : copie locale thread-safe des joint positions
    std::vector<double> current_joint_positions;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      current_joint_positions = last_joint_positions_;
    }

    // Utilisation des joint_states reçus pour mettre à jour les commandes
    if (current_joint_positions.size() == 7) {
      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(current_joint_positions[i]);
      }
    }
    

    return controller_interface::return_type::OK;
  }


  CallbackReturn JointPositionExampleController::on_init() {
    try {
      auto_declare<bool>("gazebo", false);
      auto_declare<std::string>("robot_description", "");
    } catch (const std::exception& e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionExampleController::on_configure(
      const rclcpp_lifecycle::State& /*previous_state*/) {
      joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states_isaac", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      // 7 valeurs
      if (msg->name.size() >= 7 && msg->position.size() >= 7) {
        last_joint_positions_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
          last_joint_positions_[i] = msg->position[i];
        }
      }
    });

    is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
    parameters_client->wait_for_service();

    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if (!result.empty()) {
      robot_description_ = result[0].value_to_string();
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    }

    arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JointPositionExampleController::on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/) {
    initialization_flag_ = true;
    elapsed_time_ = 0.0;
    return CallbackReturn::SUCCESS;
  }

  }  // namespace franka_example_controllers
  #include "pluginlib/class_list_macros.hpp"
  // NOLINTNEXTLINE
  PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                        controller_interface::ControllerInterface)