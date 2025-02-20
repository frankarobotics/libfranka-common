// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace research_interface {
namespace robot {

#pragma pack(push, 1)

enum class MotionGeneratorMode : uint8_t {
  kIdle,
  kJointPosition,
  kJointVelocity,
  kCartesianPosition,
  kCartesianVelocity,
  kNone
};

enum class ControllerMode : uint8_t {
  kJointImpedance,
  kCartesianImpedance,
  kExternalController,
  kOther
};

enum class RobotMode : uint8_t {
  kOther,
  kIdle,
  kMove,
  kGuiding,
  kReflex,
  kUserStopped,
  kAutomaticErrorRecovery
};

struct RobotState {
  // Templated floatarray (Implicit Conversion Array) class
  // allows for the conversion of the array type float to double
  // without the need for a separate class or rediculous amount of re-factoring
  template <std::size_t N>
  class floatarray : public std::array<float, N> {
   public:
    floatarray() = default;
    // Copy Constructor that allows initialization from std::array<float, N>
    floatarray(const std::array<float, N> &arr) {
      std::copy(arr.begin(), arr.end(), this->begin());
    }

    // Constructor from std::array<double, N>
    floatarray(const std::array<double, N> &arr) {
      std::transform(arr.begin(), arr.end(), this->begin(), [](double val) {
        return static_cast<float>(val);
      });
    }

    // Assignment operator from std::array<double, N>
    floatarray &operator=(const std::array<double, N> &arr) {
      std::transform(arr.begin(), arr.end(), this->begin(), [](double val) {
        return static_cast<float>(val);
      });
      return *this;
    }

    // Conversion to std::array<double, N>
    operator std::array<double, N>() const {
      std::array<double, N> result;
      std::transform(this->begin(), this->end(), result.begin(), [](float val) {
        return static_cast<double>(val);  // Convert float to double
      });
      return result;
    }

    // required for google test, which apparantly can't handle
    // template-based derived types due to type resolution.
    bool operator==(const floatarray<N> &other) const {
      return std::equal(this->begin(), this->end(), other.begin());
    }

    // Index operator (accessor) for float
    float &operator[](std::size_t i) { return std::array<float, N>::operator[](i); }
    const float &operator[](std::size_t i) const { return std::array<float, N>::operator[](i); }
  };

  uint64_t message_id;
  floatarray<16> O_T_EE;
  floatarray<16> O_T_EE_d;
  floatarray<16> F_T_EE;
  floatarray<16> EE_T_K;
  floatarray<16> F_T_NE;
  floatarray<16> NE_T_EE;
  float m_ee;
  floatarray<9> I_ee;
  floatarray<3> F_x_Cee;
  float m_load;
  floatarray<9> I_load;
  floatarray<3> F_x_Cload;
  floatarray<2> elbow;
  floatarray<2> elbow_d;
  floatarray<7> tau_J;
  floatarray<7> tau_J_d;
  floatarray<7> dtau_J;
  floatarray<7> q;
  floatarray<7> q_d;
  floatarray<7> dq;
  floatarray<7> dq_d;
  floatarray<7> ddq_d;
  floatarray<7> joint_contact;
  floatarray<6> cartesian_contact;
  floatarray<7> joint_collision;
  floatarray<6> cartesian_collision;
  floatarray<7> tau_ext_hat_filtered;
  floatarray<6> O_F_ext_hat_K;
  floatarray<6> K_F_ext_hat_K;
  floatarray<6> O_dP_EE_d;
  floatarray<3> O_ddP_O;
  floatarray<2> elbow_c;
  floatarray<2> delbow_c;
  floatarray<2> ddelbow_c;
  floatarray<16> O_T_EE_c;
  floatarray<6> O_dP_EE_c;
  floatarray<6> O_ddP_EE_c;
  floatarray<7> theta;
  floatarray<7> dtheta;
  MotionGeneratorMode motion_generator_mode;
  ControllerMode controller_mode;
  std::array<bool, 41> errors;
  std::array<bool, 41> reflex_reason;
  RobotMode robot_mode;
  float control_command_success_rate;
};
;

struct MotionGeneratorCommand {
  std::array<double, 7> q_c;
  std::array<double, 7> dq_c;
  std::array<double, 16> O_T_EE_c;
  std::array<double, 6> O_dP_EE_c;
  std::array<double, 2> elbow_c;
  bool valid_elbow;
  bool motion_generation_finished;
};

struct ControllerCommand {
  std::array<double, 7> tau_J_d;
  bool torque_command_finished;
};

struct RobotCommand {
  uint64_t message_id;
  MotionGeneratorCommand motion;
  ControllerCommand control;
};

#pragma pack(pop)

}  // namespace robot
}  // namespace research_interface
