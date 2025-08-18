// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/robot.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <cmath> 
#include <iostream>
#include <limits>
#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <vector>
#include "examples_common.h"
#include <fstream>
#include </home/pbd/franka_ros2_ws/src/libfranka/include/ruckig/include/ruckig/ruckig.hpp> // Change to yours
#include "std_msgs/msg/bool.hpp"
using namespace std;
using namespace ruckig;
class SubscriberJointStates : public rclcpp::Node {
 public:
  SubscriberJointStates() : Node("subscriberJointStates") {
    subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_isaac", 10,

        [this](sensor_msgs::msg::JointState::SharedPtr msg) { this->last_msg = msg; });
  }
  shared_ptr<sensor_msgs::msg::JointState> get_last_msg() { return this->last_msg; }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
  sensor_msgs::msg::JointState::SharedPtr last_msg;
};

array<double, 8> vector_to_array(const std::vector<double>& vec) {
  std::array<double, 8> arr;
  std::copy_n(vec.begin(), 8, arr.begin());
  return arr;
}

int main(int argc, char* argv[]) {
  std::chrono::time_point<std::chrono::steady_clock> start;
  try {
    franka::Robot robot("192.168.1.100");
    size_t time = 0;
    rclcpp::init(argc, argv);
    shared_ptr<SubscriberJointStates> node = make_shared<SubscriberJointStates>();
    vector<shared_ptr<sensor_msgs::msg::JointState>> tab_mesg;
    double duration_sec = 10.0;
    
    setDefaultBehavior(robot);
    auto timeinit = chrono::steady_clock::now();



    auto node_pub = rclcpp::Node::make_shared("gripper_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Bool>("gripper_command", 10);

    auto msg_pub = std_msgs::msg::Bool();
    msg_pub.data = true;








    // Listen the joints states and stock them
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - timeinit).count() <
           duration_sec) {
      rclcpp::spin_some(node);
      shared_ptr<sensor_msgs::msg::JointState> message = node->get_last_msg();
      if (message) {
        if (tab_mesg.empty() || message->position != tab_mesg.back()->position ){
          tab_mesg.push_back(message);
          }
      }
      cout << std::chrono::duration<double>(std::chrono::steady_clock::now() - timeinit).count()
              << endl;
    }

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{tab_mesg[0]->position[0],tab_mesg[0]->position[1],tab_mesg[0]->position[2],tab_mesg[0]->position[3],tab_mesg[0]->position[4],tab_mesg[0]->position[5],tab_mesg[0]->position[6]}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Print collected input data
    
    const size_t maxWaypoints =tab_mesg.size();
    double periode_In = duration_sec/(double)tab_mesg.size();
    cout<<"Duration: "<<duration_sec<<endl;
    cout<<"Number of waypoints: "<<maxWaypoints<<endl;
    cout<<"Mean period: "<<periode_In<<"Mean frequency: "<<1/periode_In<<endl;
    int nb_interpolation = static_cast<int>(ceil(periode_In / 0.001));
    cout<<"Minimum needed interpolations: : "<<nb_interpolation<<endl;





    // Trajectory generation
    
    const double control_cycle = 0.001;
    const size_t DOFS = 8;
    Ruckig<DOFS> ruckig(control_cycle,maxWaypoints);
    InputParameter<DOFS> input;
    OutputParameter<DOFS> output(maxWaypoints);
    input.max_velocity= {2.175,2.175,2.175,2.175,2.61,2.61,2.61,0.1};
    input.max_acceleration={2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0};
    input.max_jerk = {3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,3000.0};  // en rad/s³
    


    input.current_position=vector_to_array(tab_mesg[0]->position);
    input.intermediate_positions = {vector_to_array(tab_mesg[1]->position)};
    for(size_t index_intermediatepos =1;index_intermediatepos<tab_mesg.size()-1;index_intermediatepos++){
      input.intermediate_positions.push_back(vector_to_array(tab_mesg[index_intermediatepos]->position));
    }
    input.target_position =vector_to_array(tab_mesg[maxWaypoints-1]->position);
    

      std::array<double, 8> per_section_minimum_duration;
   per_section_minimum_duration.fill(0.1);
    input.intermediate_positions = ruckig.filter_intermediate_positions(input,per_section_minimum_duration);

    cout<<"Input are valid."<<ruckig.validate_input(input, false,true)<<endl;


    std::vector<std::array<double, 8>> trajectory;
    trajectory.push_back(vector_to_array(tab_mesg[0]->position));

    





    while (ruckig.update(input, output) == Result::Working) {
        trajectory.push_back(output.new_position);
        output.pass_to_input(input);

        
    }
    cout<<"Number of joint states sended: "<<trajectory.size()<<endl;

    
    





    // // verif
    // for (int i =0;i<50;i++){
    //   cout<<"tabmessage: : "<<tab_mesg[i]->position[0]<<endl;

    // }
    // for (int i =0;i<50;i++){
    //   cout<<"traj_interpol: : "<<trajectory[i][0]<<endl;
    //   if(i%(nb_interpolation) == 0){
    //   cout<<"trajinterpol VERIFICATION: : "<<trajectory[i][0]<<endl;}

    // }
    


























    std::ofstream traj_file("/home/pbd/franka_ros2_ws/src/libfranka/examples/trajectory.csv");
    for (size_t i = 0; i < trajectory.size(); ++i) {
      for (size_t j = 0; j < trajectory[i].size(); ++j) {
        traj_file << trajectory[i][j];
        if (j < trajectory[i].size() - 1)
          traj_file << ",";
      }
      traj_file << "\n";
    }
    if (!traj_file.is_open()) {
  std::cerr << " Erreur : impossible de créer le fichier trajectory.csv" << std::endl;
  return -1;
}

    
    traj_file.close();

    



    
    bool already_published = false;
    auto control_callback = [&](const franka::RobotState&,franka::Duration period) -> franka::JointPositions{
      time += period.toMSec();

      if (time >= trajectory.size()){
        time = trajectory.size() - 1;
      }
        
      
      // franka::JointPositions output =q_goal;
      franka::JointPositions output = {{0, 0, 0, 0, 0, 0, 0}};
      
      output.q={trajectory[time][0],trajectory[time][1],trajectory[time][2],trajectory[time][3],trajectory[time][4],trajectory[time][5],trajectory[time][6]};
      if (!already_published && trajectory[time][7]<0.0795 ){
        publisher->publish(msg_pub);
        cout<<"Time pub:"<<time << endl;
        already_published = true;
      }

      

      if (time >= trajectory.size() - 1) {
        std::cout << std::endl << "Finished motion" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    };


    // Replay

    
    auto active_control = robot.startJointPositionControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
    
      bool motion_finished = false;
      start = std::chrono::steady_clock::now();      
      while (!motion_finished) {
        auto read_once_return = active_control->readOnce();
        auto robot_state = read_once_return.first;
        auto duration = read_once_return.second;
        auto joint_positions = control_callback(robot_state, duration);
        motion_finished = joint_positions.motion_finished;
        active_control->writeOnce(joint_positions);
        
      }
      auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Execution time: " << elapsed.count() << " seconds" << std::endl;
    

  }

  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Execution time: " << elapsed.count() << " seconds" << std::endl;
    return -1;
  }
  return 0;
}