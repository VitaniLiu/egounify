/**
 * @file types.h
 * @brief C++ type definitions for the Xynova Dexterous Hand.
 *
 * This file provides strongly typed C++ data types corresponding to
 * Xynova Dexterous Hand C API structures, including joint identifiers,
 * control modes, joint commands, joint states, and tactile sensing data.
 *
 * @version 1.0.0
 * @date 2026-07-17
 * @author Xynova
 *
 * @see xynova/c/dexterous_hand_types.h
 */

#ifndef XYNOVA_CPP_DEXTEROUS_HAND_TYPES_HPP
#define XYNOVA_CPP_DEXTEROUS_HAND_TYPES_HPP

#include <array>
#include <vector>

#include <cstddef>
#include <cstdint>

#include "xynova/c/dexterous_hand_types.h"
#include "xynova/cpp/infra/tools/value_types.hpp"

namespace xynova {
namespace hand {
namespace dexterous_hand {

/**
 * @brief Number of joints supported by the dexterous hand.
 */
static constexpr size_t JOINT_NUM = XYNOVA_DEXTEROUS_HAND_JOINT_NUM;

/**
 * @brief Number of tactile regions supported by the dexterous hand.
 */
static constexpr size_t TACTILE_NUM = XYNOVA_DEXTEROUS_HAND_TACTILE_NUM;

/**
 * @brief Number of tactile taxels supported by the dexterous hand.
 */
static constexpr size_t TACTILE_TOTAL_TAXEL_NUM = XYNOVA_DEXTEROUS_HAND_TACTILE_TOTAL_TAXEL_NUM;

/**
 * @brief Number of range points supported by the default range sensor.
 */
static constexpr size_t RANGE_POINT_NUM = XYNOVA_DEXTEROUS_HAND_RANGE_POINT_NUM;

/**
 * @brief Joint identifier type.
 */
using JointID = XynovaDexterousHandJointID;

/**
 * @brief Tactile region identifier type.
 */
using TactileID = XynovaDexterousHandTactileID;

/**
 * @brief Joint error code type.
 */
using ErrorCode = XynovaDexterousHandErrorCode;

/**
 * @brief Global warning flags type.
 */
using WarningFlags = XynovaDexterousHandWarningFlags;

/**
 * @brief Global fault flags type.
 */
using FaultFlags = XynovaDexterousHandFaultFlags;

/**
 * @brief Joint error flags.
 */
enum class JointError : uint16_t
{
    None                = XYNOVA_DEXTEROUS_HAND_ERROR_NONE,
    StallProtection     = XYNOVA_DEXTEROUS_HAND_ERROR_STALL_PROTECTION,
    OverTemperature     = XYNOVA_DEXTEROUS_HAND_ERROR_OVER_TEMPERATURE,
    OverCurrent         = XYNOVA_DEXTEROUS_HAND_ERROR_OVER_CURRENT,
    MotorAbnormal       = XYNOVA_DEXTEROUS_HAND_ERROR_MOTOR_ABNORMAL,
    Communication       = XYNOVA_DEXTEROUS_HAND_ERROR_COMMUNICATION,
    EncoderAbnormal     = XYNOVA_DEXTEROUS_HAND_ERROR_ENCODER_ABNORMAL
};

/**
 * @brief Joint error code wrapper.
 *
 * Provides type-safe access to joint error flags.
 */
class JointErrorCode
{
public:
    constexpr JointErrorCode(ErrorCode code = 0) noexcept
        : i_code(code)
    {
    }

    void fromC(const XynovaTypedValue &value)
    {
        i_code = value.value.u16;
    }

    XynovaTypedValue toC() const
    {
        XynovaTypedValue value{};
        value.type = XYNOVA_DATA_TYPE_U16;
        value.value.u16 = i_code;
        return value;
    }

    constexpr bool has(JointError error) const noexcept
    {
        return (i_code & static_cast<ErrorCode>(error)) != 0;
    }

    constexpr bool ok() const noexcept
    {
        return i_code == 0;
    }

    constexpr ErrorCode value() const noexcept
    {
        return i_code;
    }

private:
    ErrorCode i_code{0};
};

/**
 * @brief Joint error state.
 *
 * Contains joint identifier and corresponding error code.
 */
struct JointErrorState
{
    JointID id{0}; /**< Joint identifier */

    JointErrorCode error_code{}; /**< Joint error code */
};

/**
 * @brief Fixed-size joint value array.
 *
 * Each element contains:
 * - Joint identifier.
 * - Strongly typed joint value.
 *
 * @tparam T Joint value type.
 */
template<typename T>
using JointValueArray = std::array<infra::tools::IdValue<T>, JOINT_NUM>;

/**
 * @brief Dexterous hand error state.
 *
 * Contains global warning, global fault and joint error information.
 */
struct ErrorState
{
    uint64_t timestamp_us{0}; /**< Error timestamp (us): system monotonic clock */

    WarningFlags warning_flags{0}; /**< Global warning flags */

    FaultFlags fault_flags{0}; /**< Global fault flags */

    std::array<JointErrorState, JOINT_NUM> joint_error_codes{}; /**< Joint error states */

public:
    /**
     * @brief Check whether any global fault exists.
     */
    bool hasFault() const noexcept
    {
        return fault_flags != 0;
    }

    /**
     * @brief Check whether any global warning exists.
     */
    bool hasWarning() const noexcept
    {
        return warning_flags != 0;
    }

    /**
     * @brief Get joint error by joint identifier.
     */
    JointErrorState &joint(JointID id) noexcept
    {
        for (auto &error : joint_error_codes)
        {
            if (error.id == id)
                return error;
        }

        static JointErrorState empty{};
        return empty;
    }

    /**
     * @brief Get joint error by joint identifier.
     */
    const JointErrorState &joint(JointID id) const noexcept
    {
        for (const auto &error : joint_error_codes)
        {
            if (error.id == id)
                return error;
        }

        static const JointErrorState empty{};
        return empty;
    }
};

/**
 * @brief Hand type.
 */
using HandType = XynovaDexterousHandType;

/**
 * @brief Joint control mode type.
 */
using ControlMode = XynovaDexterousHandControlMode;

/**
 * @brief Range point measurement status type.
 */
using RangePointStatus = XynovaDexterousHandRangePointStatus;

/**
 * @brief Tactile wrench dimension type.
 */
using WrenchDimension = XynovaDexterousHandWrenchDimension;

/**
 * @brief Joint control mode command.
 *
 * Each element stores:
 * - Joint identifier.
 * - Joint control mode.
 */
using ControlModeCommand = JointValueArray<ControlMode>;

/**
 * @brief Joint angle command.
 *
 * Each element stores:
 * - Joint identifier.
 * - Target joint angle.
 *
 * Unit: rad.
 */
using AngleControl = JointValueArray<float>;

/**
 * @brief Joint angle feedback state.
 *
 * Each element stores:
 * - Joint identifier.
 * - Measured joint angle.
 *
 * Unit: rad.
 */
using AngleStates = JointValueArray<float>;

/**
 * @brief Tactile resultant wrench data.
 *
 * Represents force/torque measurement of a tactile region.
 */
struct TactileWrench
{
    uint64_t timestamp_us{0};             /**< Sampling timestamp (us): system monotonic clock */
    TactileID region{0};                  /**< Tactile region identifier */
    WrenchDimension dimension{0};         /**< Wrench dimension */
    infra::tools::Value<float, 6> wrench; /**< Force/torque values: force unit: N, torque unit: Nm */
};

/**
 * @brief Tactile resultant wrench states.
 *
 * Each element represents one tactile region.
 */
using TactileWrenchStates = std::array<TactileWrench, TACTILE_NUM>;

/**
 * @brief Tactile taxel measurement.
 *
 * Represents force/torque data from a single tactile sensing element.
 */
struct TactileTaxel
{
    uint64_t timestamp_us{0};             /**< Sampling timestamp (us): system monotonic clock */
    TactileID region{0};                  /**< Tactile region identifier */
    uint16_t taxel_id{0};                 /**< Taxel identifier */
    WrenchDimension dimension{0};         /**< Wrench dimension */
    uint8_t reserved{0};                  /**< Reserved */
    infra::tools::Value<float, 6> wrench; /**< Force/torque values: force unit: N, torque unit: Nm */
};

/**
 * @brief Dynamic tactile taxel state array.
 *
 * The container owns tactile taxel data.
 */
using TactileTaxelStates = std::vector<TactileTaxel>;

/**
 * @brief Range point measurement.
 *
 * Represents distance measurement from a single range sensing point.
 */
using RangePoint = XynovaDexterousHandRangePoint;

/**
 * @brief Dynamic range point states.
 *
 * Represents a two-dimensional range map stored in row-major order.
 *
 * The point corresponding to coordinate (x, y) is stored at:
 * index = y * width + x
 */
struct RangeStates
{
    uint16_t width{0};              /**< Range map width */
    uint16_t height{0};             /**< Range map height */
    std::vector<RangePoint> points; /**< Range points */

    explicit RangeStates(size_t capacity = 0)
        : points(capacity)
    {
    }
};

} // namespace dexterous_hand
} // namespace hand
} // namespace xynova

#endif /* XYNOVA_CPP_DEXTEROUS_HAND_TYPES_HPP */