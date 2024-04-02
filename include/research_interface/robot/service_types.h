// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace research_interface {
namespace robot {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 8;
constexpr uint16_t kCommandPort = 1337;

enum class Command : uint32_t {
  kConnect,
  kMove,
  kStopMove,
  kSetCollisionBehavior,
  kSetJointImpedance,
  kSetCartesianImpedance,
  kSetGuidingMode,
  kSetEEToK,
  kSetNEToEE,
  kSetLoad,
  kAutomaticErrorRecovery,
  kLoadModelLibrary,
  kGetRobotModel
};

struct CommandHeader {
  CommandHeader() = default;
  CommandHeader(Command command, uint32_t command_id, uint32_t size)
      : command(command), command_id(command_id), size(size) {}

  Command command;
  uint32_t command_id;
  uint32_t size;
};

template <typename T>
struct RequestBase {};

template <typename T>
struct ResponseBase {
  ResponseBase(typename T::Status status) : status(status) {}

  const typename T::Status status;

  static_assert(std::is_enum<decltype(status)>::value, "Status must be an enum.");
  static_assert(std::is_same<typename std::underlying_type<decltype(status)>::type, uint8_t>::value,
                "Status must be of type uint8_t.");
  static_assert(static_cast<uint32_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T>
struct CommandMessage {
  CommandMessage() = default;
  CommandMessage(const CommandHeader &header, const T &instance) : header(header) {
    std::memcpy(payload.data(), &instance, payload.size());
  }

  T getInstance() const noexcept { return *reinterpret_cast<const T *>(payload.data()); }

  CommandHeader header;
  std::array<uint8_t, sizeof(T)> payload;
};

template <typename T>
struct CommandMessage<RequestBase<T>> {
  CommandMessage() = default;
  CommandMessage(const CommandHeader &header, const RequestBase<T> &) : header(header) {}

  RequestBase<T> getInstance() const noexcept { return RequestBase<T>(); }

  CommandHeader header;
};

/**
 * A CommandMessage which can contain dynamically sized data
 *
 * @tparam T The type of the carried message
 */
template <typename T>
struct DynamicSizedCommandMessage {
 public:
  DynamicSizedCommandMessage() = default;

  /**
   * Constructs the message with a header containing the message information (type, size, ...) and
   * the carried data
   *
   * @param header The header of the message. Important: The size will be derived within the
   *   constructor!
   * @param instance The to-be carried data
   */
  DynamicSizedCommandMessage(const CommandHeader &header, const T &instance) : header(header) {
    payload = instance.serialize();
    this->header.size = sizeof(header) + payload.size();
  }

  /**
   * @return T Returns the carried data as its original type
   */
  T getInstance() const noexcept { return T::deserialize(payload); }

  /**
   * @return std::vector<uint8_t> Serializes the contained message data into a byte vector. This
   *   includes header + payload.
   */
  auto serialize() -> std::vector<uint8_t> {
    std::vector<uint8_t> buffer(header.size);
    memcpy(buffer.data(), &header, sizeof(header));

    std::copy(payload.cbegin(), payload.cend(), buffer.data() + sizeof(header));

    return buffer;
  }

  /**
   * Constructs the Message from a buffer containing all necessary information
   *
   * @param buffer The byte vector containing all data
   * @return DynamicSizedCommandMessage<T> The constructed message carrying the received data
   */
  static auto deserialize(const std::vector<uint8_t> &buffer) -> DynamicSizedCommandMessage<T> {
    DynamicSizedCommandMessage<T> command_message;
    memcpy(&command_message.header, buffer.data(), sizeof(header));

    command_message.payload = std::vector<uint8_t>(buffer.cbegin() + sizeof(header), buffer.cend());

    return command_message;
  }

 public:
  CommandHeader header;
  std::vector<uint8_t> payload;
};

template <typename T, Command C>
struct CommandBase {
  CommandBase() = delete;

  static constexpr Command kCommand = C;

  enum class Status : uint8_t {
    kSuccess,
    kCommandNotPossibleRejected,
    kCommandRejectedDueToActivatedSafetyFunctions
  };

  using Header = CommandHeader;
  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
  template <typename P>
  using Message = CommandMessage<P>;
};

template <typename T, Command C>
struct GetterSetterCommandBase : CommandBase<T, C> {
  enum class Status : uint8_t {
    kSuccess,
    kCommandNotPossibleRejected,
    kInvalidArgumentRejected,
    kCommandRejectedDueToActivatedSafetyFunctions
  };
};

struct Connect : CommandBase<Connect, Command::kConnect> {
  enum class Status : uint8_t { kSuccess, kIncompatibleLibraryVersion };

  struct Request : public RequestBase<Connect> {
    Request(uint16_t udp_port) : version(kVersion), udp_port(udp_port) {}

    const Version version;
    const uint16_t udp_port;
  };

  struct Response : public ResponseBase<Connect> {
    Response(Status status) : ResponseBase(status), version(kVersion) {}

    const Version version;
  };
};

struct Move : public CommandBase<Move, Command::kMove> {
  enum class ControllerMode : uint32_t {
    kJointImpedance,
    kCartesianImpedance,
    kExternalController
  };

  enum class MotionGeneratorMode : uint32_t {
    kJointPosition,
    kJointVelocity,
    kCartesianPosition,
    kCartesianVelocity,
    kNone
  };

  enum class Status : uint8_t {
    kSuccess,
    kMotionStarted,
    kPreempted,
    kPreemptedDueToActivatedSafetyFunctions,
    kCommandRejectedDueToActivatedSafetyFunctions,
    kCommandNotPossibleRejected,
    kStartAtSingularPoseRejected,
    kInvalidArgumentRejected,
    kReflexAborted,
    kEmergencyAborted,
    kInputErrorAborted,
    kAborted
  };

  struct Deviation {
    constexpr Deviation(double translation, double rotation, double elbow)
        : translation(translation), rotation(rotation), elbow(elbow) {}
    const double translation;
    const double rotation;
    const double elbow;
  };

  struct Request : public RequestBase<Move> {
    Request(ControllerMode controller_mode,
            MotionGeneratorMode motion_generator_mode,
            const Deviation &maximum_path_deviation,
            const Deviation &maximum_goal_pose_deviation)
        : controller_mode(controller_mode),
          motion_generator_mode(motion_generator_mode),
          maximum_path_deviation(maximum_path_deviation),
          maximum_goal_pose_deviation(maximum_goal_pose_deviation) {}

    const ControllerMode controller_mode;
    const MotionGeneratorMode motion_generator_mode;
    const Deviation maximum_path_deviation;
    const Deviation maximum_goal_pose_deviation;
  };
};

/**
 * The GetRobotModel contains the request and response to receive the robot model
 */
struct GetRobotModel : public CommandBase<GetRobotModel, Command::kGetRobotModel> {
  /**
   * The empty request sent for receiving back the robot model.
   */
  struct Request : public RequestBase<GetRobotModel> {
    auto serialize() const -> std::vector<uint8_t> { return std::vector<uint8_t>(); }
    static auto deserialize(const std::vector<uint8_t> & /*buffer */) -> Request {
      return Request();
    }
  };

  /**
   * The response carries the robot model
   */
  struct Response : public ResponseBase<GetRobotModel> {
   public:
    /**
     * Default constructor for an empty returned model
     *
     * @param status The status of the request
     */
    Response(Status status) : ResponseBase<GetRobotModel>(status), robot_model() {}

    /**
     * Constructs the response with the given status and robot model
     *
     * @param status The status of the request
     * @param robot_model The returned robot model
     */
    Response(Status status, const std::string &robot_model)
        : ResponseBase<GetRobotModel>(status), robot_model(robot_model) {}

    /**
     * @return std::vector<uint8_t> Returns the serialized data as byte vector
     */
    auto serialize() const -> std::vector<uint8_t> {
      std::vector<uint8_t> buffer(sizeof(status) + robot_model.size());
      memcpy(buffer.data(), &status, sizeof(status));

      std::copy(robot_model.cbegin(), robot_model.cend(), buffer.begin() + sizeof(status));

      return buffer;
    }

    /**
     * Constructs a GetRobotModel from the received buffer
     *
     * @param buffer The buffer containing the data
     * @return Response The constructed GetRobotModel
     */
    static auto deserialize(const std::vector<uint8_t> &buffer) -> Response {
      Status status;
      memcpy(&status, buffer.data(), sizeof(status));

      return Response(status, std::string(buffer.cbegin() + sizeof(Status), buffer.cend()));
    }

   public:
    std::string robot_model;
  };

  template <typename P>
  using Message = DynamicSizedCommandMessage<P>;
};

struct StopMove : public CommandBase<StopMove, Command::kStopMove> {
  enum class Status : uint8_t {
    kSuccess,
    kCommandNotPossibleRejected,
    kCommandRejectedDueToActivatedSafetyFunctions,
    kEmergencyAborted,
    kReflexAborted,
    kAborted
  };
};

struct SetCollisionBehavior
    : public GetterSetterCommandBase<SetCollisionBehavior, Command::kSetCollisionBehavior> {
  struct Request : public RequestBase<SetCollisionBehavior> {
    Request(const std::array<double, 7> &lower_torque_thresholds_acceleration,
            const std::array<double, 7> &upper_torque_thresholds_acceleration,
            const std::array<double, 7> &lower_torque_thresholds_nominal,
            const std::array<double, 7> &upper_torque_thresholds_nominal,
            const std::array<double, 6> &lower_force_thresholds_acceleration,
            const std::array<double, 6> &upper_force_thresholds_acceleration,
            const std::array<double, 6> &lower_force_thresholds_nominal,
            const std::array<double, 6> &upper_force_thresholds_nominal)
        : lower_torque_thresholds_acceleration(lower_torque_thresholds_acceleration),
          upper_torque_thresholds_acceleration(upper_torque_thresholds_acceleration),
          lower_torque_thresholds_nominal(lower_torque_thresholds_nominal),
          upper_torque_thresholds_nominal(upper_torque_thresholds_nominal),
          lower_force_thresholds_acceleration(lower_force_thresholds_acceleration),
          upper_force_thresholds_acceleration(upper_force_thresholds_acceleration),
          lower_force_thresholds_nominal(lower_force_thresholds_nominal),
          upper_force_thresholds_nominal(upper_force_thresholds_nominal) {}

    const std::array<double, 7> lower_torque_thresholds_acceleration;
    const std::array<double, 7> upper_torque_thresholds_acceleration;

    const std::array<double, 7> lower_torque_thresholds_nominal;
    const std::array<double, 7> upper_torque_thresholds_nominal;

    const std::array<double, 6> lower_force_thresholds_acceleration;
    const std::array<double, 6> upper_force_thresholds_acceleration;

    const std::array<double, 6> lower_force_thresholds_nominal;
    const std::array<double, 6> upper_force_thresholds_nominal;
  };
};

struct SetJointImpedance
    : public GetterSetterCommandBase<SetJointImpedance, Command::kSetJointImpedance> {
  struct Request : public RequestBase<SetJointImpedance> {
    Request(const std::array<double, 7> &K_theta) : K_theta(K_theta) {}

    const std::array<double, 7> K_theta;
  };
};

struct SetCartesianImpedance
    : public GetterSetterCommandBase<SetCartesianImpedance, Command::kSetCartesianImpedance> {
  struct Request : public RequestBase<SetCartesianImpedance> {
    Request(const std::array<double, 6> &K_x) : K_x(K_x) {}

    const std::array<double, 6> K_x;
  };
};

struct SetGuidingMode : public GetterSetterCommandBase<SetGuidingMode, Command::kSetGuidingMode> {
  struct Request : public RequestBase<SetGuidingMode> {
    Request(const std::array<bool, 6> &guiding_mode, bool nullspace)
        : guiding_mode(guiding_mode), nullspace(nullspace) {}

    const std::array<bool, 6> guiding_mode;
    const bool nullspace;
  };
};

struct SetEEToK : public GetterSetterCommandBase<SetEEToK, Command::kSetEEToK> {
  struct Request : public RequestBase<SetEEToK> {
    Request(const std::array<double, 16> &EE_T_K) : EE_T_K(EE_T_K) {}

    const std::array<double, 16> EE_T_K;
  };
};

struct SetNEToEE : public GetterSetterCommandBase<SetNEToEE, Command::kSetNEToEE> {
  struct Request : public RequestBase<SetNEToEE> {
    Request(const std::array<double, 16> &NE_T_EE) : NE_T_EE(NE_T_EE) {}

    const std::array<double, 16> NE_T_EE;
  };
};

struct SetLoad : public GetterSetterCommandBase<SetLoad, Command::kSetLoad> {
  struct Request : public RequestBase<SetLoad> {
    Request(double m_load,
            const std::array<double, 3> &F_x_Cload,
            const std::array<double, 9> &I_load)
        : m_load(m_load), F_x_Cload(F_x_Cload), I_load(I_load) {}

    const double m_load;
    const std::array<double, 3> F_x_Cload;
    const std::array<double, 9> I_load;
  };
};

struct AutomaticErrorRecovery
    : public CommandBase<AutomaticErrorRecovery, Command::kAutomaticErrorRecovery> {
  enum class Status : uint8_t {
    kSuccess,
    kCommandNotPossibleRejected,
    kCommandRejectedDueToActivatedSafetyFunctions,
    kManualErrorRecoveryRequiredRejected,
    kReflexAborted,
    kEmergencyAborted,
    kAborted
  };
};

struct LoadModelLibrary : public CommandBase<LoadModelLibrary, Command::kLoadModelLibrary> {
  enum class Status : uint8_t { kSuccess, kError };

  enum class Architecture : uint8_t { kX64, kX86, kARM, kARM64 };

  enum class System : uint8_t { kLinux, kWindows };

  struct Request : public RequestBase<LoadModelLibrary> {
    Request(Architecture architecture, System system)
        : architecture(architecture), system(system) {}

    const Architecture architecture;
    const System system;
  };
};

#pragma pack(pop)

}  // namespace robot
}  // namespace research_interface
