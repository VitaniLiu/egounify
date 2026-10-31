#include "litchi_xynova_arm_bridge.h"

#include "xynova/robot.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
thread_local std::string g_last_error;

void SetError(const std::string& value)
{
    g_last_error = value;
}

void SetError(const char* operation, const std::error_code& error)
{
    SetError(std::string(operation) + ": " + error.message());
}

template <size_t N>
void CopyText(const std::string& source, char (&target)[N])
{
    const size_t count = std::min(source.size(), N - 1);
    std::memcpy(target, source.data(), count);
    target[count] = '\0';
}

class RobotHandleBase
{
public:
    virtual ~RobotHandleBase() = default;
    virtual xynova::BaseRobot& robot() = 0;
    virtual uint32_t jointCount() const = 0;
    virtual bool readState(LitchiXynovaArmState& state) = 0;
    virtual bool readWrench(double* wrench) = 0;
};

template <typename Robot, size_t DoF, bool HasForce>
class RobotHandle final : public RobotHandleBase
{
public:
    RobotHandle(const std::string& remote_ip, const std::string& local_ip)
        : robot_(Create(remote_ip, local_ip))
    {
    }

    xynova::BaseRobot& robot() override
    {
        return robot_;
    }

    uint32_t jointCount() const override
    {
        return static_cast<uint32_t>(DoF);
    }

    bool readState(LitchiXynovaArmState& state) override
    {
        std::error_code error;
        const auto position = robot_.jointPos(error);
        if (error)
        {
            SetError("jointPos", error);
            return false;
        }
        const auto velocity = robot_.jointVel(error);
        if (error)
        {
            SetError("jointVel", error);
            return false;
        }
        const auto effort = robot_.jointTorque(error);
        if (error)
        {
            SetError("jointTorque", error);
            return false;
        }
        state = {};
        state.joint_count = jointCount();
        std::copy(position.begin(), position.end(), state.position);
        std::copy(velocity.begin(), velocity.end(), state.velocity);
        std::copy(effort.begin(), effort.end(), state.effort);
        return true;
    }

    bool readWrench(double* wrench) override
    {
        if constexpr (!HasForce)
        {
            SetError("Cartesian wrench feedback is unavailable for this industrial model");
            return false;
        }
        else
        {
            std::error_code error;
            std::array<double, DoF> joint_torque{};
            std::array<double, DoF> external_torque{};
            std::array<double, 3> cart_torque{};
            std::array<double, 3> cart_force{};
            robot_.forceControl().getEndTorque(
                xynova::FrameType::tool,
                joint_torque,
                external_torque,
                cart_torque,
                cart_force,
                error);
            if (error)
            {
                SetError("getEndTorque", error);
                return false;
            }
            std::copy(cart_force.begin(), cart_force.end(), wrench);
            std::copy(cart_torque.begin(), cart_torque.end(), wrench + 3);
            return true;
        }
    }

private:
    static Robot Create(const std::string& remote_ip, const std::string& local_ip)
    {
        if constexpr (std::is_constructible_v<Robot, const std::string&, const std::string&>)
        {
            return Robot(remote_ip, local_ip);
        }
        else
        {
            return Robot(remote_ip);
        }
    }

    Robot robot_;
};

std::unique_ptr<RobotHandleBase> CreateRobot(
    int32_t model,
    const std::string& remote_ip,
    const std::string& local_ip)
{
    switch (model)
    {
    case 0:
        return std::make_unique<RobotHandle<xynova::XynovaRobot, 6, true>>(
            remote_ip, local_ip);
    case 1:
        return std::make_unique<RobotHandle<xynova::XynovaCr5Robot, 5, true>>(
            remote_ip, local_ip);
    case 2:
        return std::make_unique<RobotHandle<xynova::XynovaErRobot, 7, true>>(
            remote_ip, local_ip);
    case 3:
        return std::make_unique<RobotHandle<xynova::XynovaArRobot, 7, true>>(
            remote_ip, local_ip);
    case 4:
        return std::make_unique<RobotHandle<xynova::StandardRobot, 6, false>>(
            remote_ip, local_ip);
    case 5:
        return std::make_unique<RobotHandle<xynova::PCB3Robot, 3, false>>(
            remote_ip, "");
    case 6:
        return std::make_unique<RobotHandle<xynova::PCB4Robot, 4, false>>(
            remote_ip, "");
    default:
        SetError("unsupported Xynova arm model code");
        return nullptr;
    }
}

bool RequireHandle(LitchiXynovaArmHandle* handle);

} // namespace

struct LitchiXynovaArmHandle
{
    std::unique_ptr<RobotHandleBase> value;
};

namespace
{
bool RequireHandle(LitchiXynovaArmHandle* handle)
{
    if (handle != nullptr && handle->value != nullptr)
    {
        return true;
    }
    SetError("Xynova arm handle is null");
    return false;
}
} // namespace

extern "C"
{
uint32_t litchi_xynova_arm_bridge_abi_version(void)
{
    return LITCHI_XYNOVA_ARM_BRIDGE_ABI_VERSION;
}

uint32_t litchi_xynova_arm_state_size(void)
{
    return sizeof(LitchiXynovaArmState);
}

uint32_t litchi_xynova_arm_info_size(void)
{
    return sizeof(LitchiXynovaArmInfo);
}

LitchiXynovaArmHandle* litchi_xynova_arm_connect(
    int32_t model,
    const char* remote_ip,
    const char* local_ip)
{
    g_last_error.clear();
    if (remote_ip == nullptr || remote_ip[0] == '\0')
    {
        SetError("remote_ip must not be empty");
        return nullptr;
    }
    try
    {
        auto result = std::make_unique<LitchiXynovaArmHandle>();
        result->value = CreateRobot(
            model,
            remote_ip,
            local_ip == nullptr ? "" : local_ip);
        return result->value == nullptr ? nullptr : result.release();
    }
    catch (const std::exception& error)
    {
        SetError(std::string("connect: ") + error.what());
        return nullptr;
    }
}

int32_t litchi_xynova_arm_disconnect(LitchiXynovaArmHandle* handle)
{
    if (!RequireHandle(handle))
    {
        return -1;
    }
    std::error_code error;
    handle->value->robot().disconnectFromRobot(error);
    if (error)
    {
        SetError("disconnectFromRobot", error);
        return -1;
    }
    return 0;
}

int32_t litchi_xynova_arm_destroy(LitchiXynovaArmHandle* handle)
{
    delete handle;
    return 0;
}

int32_t litchi_xynova_arm_get_info(
    LitchiXynovaArmHandle* handle,
    LitchiXynovaArmInfo* info)
{
    if (!RequireHandle(handle) || info == nullptr)
    {
        SetError("get_info requires a non-null output");
        return -1;
    }
    std::error_code error;
    const auto native_info = handle->value->robot().robotInfo(error);
    if (error)
    {
        SetError("robotInfo", error);
        return -1;
    }
    *info = {};
    info->joint_count = handle->value->jointCount();
    CopyText(native_info.id, info->id);
    CopyText(native_info.mac, info->mac);
    CopyText(native_info.version, info->controller_version);
    CopyText(native_info.type, info->model);
    CopyText(xynova::BaseRobot::sdkVersion(), info->sdk_version);
    return 0;
}

int32_t litchi_xynova_arm_read_state(
    LitchiXynovaArmHandle* handle,
    LitchiXynovaArmState* state)
{
    if (!RequireHandle(handle) || state == nullptr)
    {
        SetError("read_state requires a non-null output");
        return -1;
    }
    return handle->value->readState(*state) ? 0 : -1;
}

int32_t litchi_xynova_arm_read_tcp_pose(
    LitchiXynovaArmHandle* handle,
    double pose_xyzrpy[6])
{
    if (!RequireHandle(handle) || pose_xyzrpy == nullptr)
    {
        SetError("read_tcp_pose requires a non-null output");
        return -1;
    }
    std::error_code error;
    const auto pose = handle->value->robot().posture(
        xynova::CoordinateType::endInRef, error);
    if (error)
    {
        SetError("posture", error);
        return -1;
    }
    std::copy(pose.begin(), pose.end(), pose_xyzrpy);
    return 0;
}

int32_t litchi_xynova_arm_read_wrench(
    LitchiXynovaArmHandle* handle,
    double wrench_xyzabc[6])
{
    if (!RequireHandle(handle) || wrench_xyzabc == nullptr)
    {
        SetError("read_wrench requires a non-null output");
        return -1;
    }
    return handle->value->readWrench(wrench_xyzabc) ? 0 : -1;
}

int32_t litchi_xynova_arm_set_power(
    LitchiXynovaArmHandle* handle,
    int32_t enabled)
{
    if (!RequireHandle(handle))
    {
        return -1;
    }
    std::error_code error;
    if (enabled != 0)
    {
        handle->value->robot().setOperateMode(xynova::OperateMode::automatic, error);
        if (error)
        {
            SetError("setOperateMode", error);
            return -1;
        }
    }
    handle->value->robot().setPowerState(enabled != 0, error);
    if (error)
    {
        SetError("setPowerState", error);
        return -1;
    }
    return 0;
}

int32_t litchi_xynova_arm_command_joint_position(
    LitchiXynovaArmHandle* handle,
    const double* position,
    uint32_t joint_count,
    double speed,
    double zone)
{
    if (!RequireHandle(handle) || position == nullptr)
    {
        SetError("command_joint_position requires non-null positions");
        return -1;
    }
    if (joint_count != handle->value->jointCount())
    {
        SetError("joint position count does not match the configured model");
        return -1;
    }
    std::error_code error;
    auto& robot = handle->value->robot();
    robot.setMotionControlMode(xynova::MotionControlMode::NrtCommand, error);
    if (error)
    {
        SetError("setMotionControlMode", error);
        return -1;
    }
    std::string command_id;
    const std::vector<double> values(position, position + joint_count);
    robot.moveAppend(xynova::MoveAbsJCommand(values, speed, zone), command_id, error);
    if (error)
    {
        SetError("moveAppend", error);
        return -1;
    }
    robot.moveStart(error);
    if (error)
    {
        SetError("moveStart", error);
        return -1;
    }
    return 0;
}

int32_t litchi_xynova_arm_stop(LitchiXynovaArmHandle* handle)
{
    if (!RequireHandle(handle))
    {
        return -1;
    }
    std::error_code error;
    handle->value->robot().stop(error);
    if (error)
    {
        SetError("stop", error);
        return -1;
    }
    return 0;
}

const char* litchi_xynova_arm_last_error(void)
{
    return g_last_error.c_str();
}
}
