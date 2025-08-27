//
// Created by hex on 4/9/25.
//
#include <fmt/core.h>
#include <nanobind/nanobind.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <../include/franka/control_types.h>
#include <../include/franka/exception.h>
#include <../include/franka/robot.h>

#include <../include/franka/gripper.h>
#include <franka/model.h>

#include "../build/_deps/json-src/include/nlohmann/json.hpp"
#include <fstream>
#include "examples_common.h"

#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using json = nlohmann::json;

using namespace nb::literals;

int run(const std::vector<std::vector<double>>& traj,
        std::string robot_ip,
        int time_factor,
        float sleep_time,
        bool safe = true,
        float frequency = 100.0) {
  try {
    std::vector<std::vector<double>> new_traj;
    for (size_t i = 0; i < traj.size(); i++) {
      // new_traj.push_back(traj[traj.size() - 1 - i]);
      new_traj.push_back(traj[i]);
    }
    if (safe == true) {
      frequency = 100.0;
    }

    // franka::Robot robot(argv[1]);
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;
    //std::cout << "traj 0 ";
    for (int i = 0; i < 7; i++) {
      q_goal[i] = new_traj[0][i];
      //std::cout << q_goal[i] << " ";
    }
    std::cout << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    // MotionGenerator motion_generator(speed_factor, q_goal);

    // std::cout << "WARNING: This example will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance;

    // robot.control(motion_generator,controller_mode,false);
    // robot.control(motion_generator);
    // std::cout << "Finished moving to initial joint configuration." << std::endl;

    std::array<double, 7> initial_position;
    double time = 0.0;
    int timefactor = 0;
    timefactor = time_factor;
    robot.control(
        [&initial_position, &time, &new_traj, &timefactor, &sleep_time](
            const franka::RobotState& robot_state,
            franka::Duration period) -> franka::JointPositions {
          time += period.toMSec();

          if (time == 0.0) {
            initial_position = robot_state.q;
            // std::cout << "run start " << initial_position[0] << " " << initial_position[1] << " "
            //           << initial_position[2] << " " << initial_position[3] << " "
            //           << initial_position[4] << " " << initial_position[5] << " "
            //           << initial_position[6] << std::endl;
          }

          size_t itraj = time / timefactor;
          itraj = (itraj >= new_traj.size() ? new_traj.size() - 1 : itraj);
          size_t itraj_next = itraj + 1;
          itraj_next = (itraj_next >= new_traj.size() ? new_traj.size() - 1 : itraj_next);
          double ratio = (time / timefactor - itraj);
          if (time >= timefactor * new_traj.size()) {
            ratio = 1.0;
          }
          franka::JointPositions output = {{
              new_traj[itraj][0] * (1 - ratio) + new_traj[itraj_next][0] * ratio,
              new_traj[itraj][1] * (1 - ratio) + new_traj[itraj_next][1] * ratio,
              new_traj[itraj][2] * (1 - ratio) + new_traj[itraj_next][2] * ratio,

              new_traj[itraj][3] * (1 - ratio) + new_traj[itraj_next][3] * ratio,
              new_traj[itraj][4] * (1 - ratio) + new_traj[itraj_next][4] * ratio,
              new_traj[itraj][5] * (1 - ratio) + new_traj[itraj_next][5] * ratio,
              new_traj[itraj][6] * (1 - ratio) + new_traj[itraj_next][6] * ratio,
          }};

          if (time >= timefactor * new_traj.size() + sleep_time) {
            std::cout << std::endl << "Finished motion, shutting down" << std::endl;
            return franka::MotionFinished(output);
          }
          return output;
        },
        controller_mode, safe, frequency);

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (...) {
    std::cout << "unknown error" << std::endl;
    return -1;
  }

  return 0;
}

int move_to_init(const std::vector<double>& target_state, std::string robot_ip) {
  try {
    // franka::Robot robot(argv[1]);
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    std::array<double, 7> initial_position;
    double time = 0.0;
    double tot_time = 10.0;
    robot.control([&initial_position, &time, &target_state, &tot_time](
                      const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      if (time == 0.0) {
        initial_position = robot_state.q;
        std::cout << "move_to_init start " << initial_position[0] << " " << initial_position[1]
                  << " " << initial_position[2] << " " << initial_position[3] << " "
                  << initial_position[4] << " " << initial_position[5] << " " << initial_position[6]
                  << std::endl;
      }

      double clip_time = time > tot_time ? 1 : time / tot_time;

      franka::JointPositions output = {{
          initial_position[0] * (1 - clip_time) + target_state[0] * clip_time,
          initial_position[1] * (1 - clip_time) + target_state[1] * clip_time,
          initial_position[2] * (1 - clip_time) + target_state[2] * clip_time,
          initial_position[3] * (1 - clip_time) + target_state[3] * clip_time,
          initial_position[4] * (1 - clip_time) + target_state[4] * clip_time,
          initial_position[5] * (1 - clip_time) + target_state[5] * clip_time,
          initial_position[6] * (1 - clip_time) + target_state[6] * clip_time,
      }};

      std::cout << "output " << output.q[0] << " " << output.q[1] << " " << output.q[2] << " "
                << output.q[3] << " " << output.q[4] << " " << output.q[5] << " " << output.q[6]
                << std::endl;

      if (time >= tot_time + .2) {
        std::cout << std::endl << "Finished sleep, shutting down" << std::endl;
        std::cout << "move_to_init end" << " " << robot_state.q[0] << " " << robot_state.q[1] << " "
                  << robot_state.q[2] << " " << robot_state.q[3] << " " << robot_state.q[4] << " "
                  << robot_state.q[5] << " " << robot_state.q[6] << std::endl;
        return franka::MotionFinished(output);
      }

      return output;
    });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

std::vector<double> fk_current(
        std::string robot_ip
        ) {
  try{
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    franka::RobotState state = robot.readOnce();
    std::array<double, 7> initial_position = state.q;
    std::cout << "run start " << initial_position[0] << " " << initial_position[1] << " "
              << initial_position[2] << " " << initial_position[3] << " "
              << initial_position[4] << " " << initial_position[5] << " "
              << initial_position[6] << std::endl;
    franka::Model model(robot.loadModel());

    // for (franka::Frame frame = franka::Frame::kEndEffector; frame <= franka::Frame::kEndEffector;
    //      frame++) {
    //   auto endpose = model.pose(frame, state);
      // std::cout << endpose << std::endl;
      // std::vector<double> fk_vec;
      // // for (int i = 0; i < 16; i++) {
      // //   fk_vec[i] = endpose[i];
      // //   std::cout << fk_vec[i] << std::endl;
      // // }
      // return fk_vec;
    // }
        // std::cout << model.pose(frame, state) << std::endl;}

    franka::Frame frame = franka::Frame::kEndEffector;
    std::array<double, 16> endpose = model.pose(frame, state);
    std::vector<double> fk_vec;
    std::cout << endpose[0] << std::endl;
    for (int i = 0; i < 16; i++) {
      fk_vec.push_back(endpose[i]);
    }
    return fk_vec;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::vector<double> m;
    return m;
  } catch (...) {
    std::cout << "unknown error" << std::endl;
    std::vector<double> m;
    return m;
  }
}

std::vector<double> fk(const std::vector<double>& traj,
        std::string robot_ip
        ) {

    std::vector<double> new_traj;
    for (size_t i = 0; i < traj.size(); i++) {
      // new_traj.push_back(traj[traj.size() - 1 - i]);
      new_traj.push_back(traj[i]);
    }

    // franka::Robot robot(argv[1]);
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;
    std::cout << "traj 0 ";
    for (int i = 0; i < 7; i++) {
      q_goal[i] = new_traj[i];
      std::cout << q_goal[i] << " ";
    }
    std::cout << std::endl;

    franka::RobotState state = robot.readOnce();
    std::array<double, 7> initial_position = state.q;
    std::cout << "run start " << initial_position[0] << " " << initial_position[1] << " "
              << initial_position[2] << " " << initial_position[3] << " "
              << initial_position[4] << " " << initial_position[5] << " "
              << initial_position[6] << std::endl;


    franka::Model model(robot.loadModel());
    // for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
         // frame++) {
        // std::cout << model.pose(frame, state) << std::endl;}

    std::array<double, 16> fk = model.pose(franka::Frame::kEndEffector, q_goal, state.F_T_EE,state.EE_T_K);

    std::vector<double> fk_vec;
    for (int i = 0; i < 16; i++) {
      fk_vec.push_back(fk[i]);
    }
  return fk_vec;
}

int print_ee(
        std::string robot_ip
        ) {

  // franka::Robot robot(argv[1]);
  franka::Robot robot(robot_ip);
  setDefaultBehavior(robot);

  franka::RobotState state = robot.readOnce();
  std::array<double, 7> initial_position = state.q;
  std::cout << "run start " << initial_position[0] << " " << initial_position[1] << " "
            << initial_position[2] << " " << initial_position[3] << " "
            << initial_position[4] << " " << initial_position[5] << " "
            << initial_position[6] << std::endl;


  franka::Model model(robot.loadModel());

  std::cout << state.F_T_EE << std::endl;
  std::cout << state.EE_T_K << std::endl;
  return 0;
}

NB_MODULE(frankz, m) {
  m.doc() = "Frankz plugin";
  m.def("run", &run, nb::call_guard<nb::gil_scoped_release>());
  m.def("move_to_init", &move_to_init);
  m.def("fk_current", &fk_current);
  m.def("fk", &fk);
  m.def("print_ee", &print_ee);
}