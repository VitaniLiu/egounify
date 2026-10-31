/**
 * @file Device.hpp
 * @brief C++ wrapper for Xynova Dexterous Hand device.
 *
 * This header provides a high-level C++ interface for controlling and
 * monitoring Xynova Dexterous Hand devices. It wraps the C API and provides
 * RAII-based resource management together with strongly typed interfaces.
 *
 * ## Features
 * - Automatic device handle lifetime management.
 * - Start and stop device communication.
 * - Joint control mode configuration.
 * - Synchronous and asynchronous joint angle commands.
 * - Joint state acquisition.
 *
 * @version 1.0.0
 * @date 2026-7-17
 * @author Xynova
 *
 * @see ::XynovaDeviceHandle
 * @see ::XynovaDexterousHand_run
 * @see xynova::hand::dexterous_hand::AngleControl
 */

#ifndef XYNOVA_CPP_DEXTEROUS_HAND_DEVICE_HPP
#define XYNOVA_CPP_DEXTEROUS_HAND_DEVICE_HPP

#include <string>
#include <utility>

#include "xynova/c/error_code.h"
#include "xynova/c/dexterous_hand_api.h"

#include "xynova/cpp/communication/Manager.hpp"
#include "xynova/cpp/hand/dexterous_hand/types.h"

namespace xynova {
namespace hand {
namespace dexterous_hand {

/**
 * @defgroup xynova_dexterous_hand_cpp C++ Dexterous Hand API
 * @brief High-level C++ interfaces for Xynova Dexterous Hand.
 * @{
 */

/**
 * @class Device
 * @brief RAII wrapper for Xynova Dexterous Hand device.
 *
 * This class provides a type-safe C++ interface over the C API.
 * The underlying device handle is automatically released when the object
 * is destroyed.
 */
class Device
{
public:
    using DeviceHandle = communication::DeviceHandle;

public:
    /**
     * @brief Construct a device wrapper.
     *
     * @param handle Device handle.
     */
    explicit Device(DeviceHandle &&handle) noexcept
        : i_dev_handle(std::move(handle))
    {}

    Device(const Device &) = delete;
    Device &operator=(const Device &) = delete;
    Device(Device &&) noexcept = delete;
    Device &operator=(Device &&) noexcept = delete;

    /**
     * @brief Check whether the device handle is valid.
     *
     * @return true if valid, otherwise false.
     */
    [[nodiscard]] bool valid() const noexcept
    {
        return static_cast<bool>(i_dev_handle);
    }

private:
    /**
     * @brief Get underlying C API handle.
     */
    XynovaDeviceHandle *c_handle() const noexcept
    {
        return reinterpret_cast<XynovaDeviceHandle *>(i_dev_handle.get());
    }

public:
    /**
     * @brief Start dexterous hand device.
     *
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode run()
    {
        return XynovaDexterousHand_run(c_handle());
    }

    /**
     * @brief Stop dexterous hand device.
     *
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode stop()
    {
        return XynovaDexterousHand_stop(c_handle());
    }

    /**
     * @brief Get product model.
     *
     * @param product_model Output product model string.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getProductModel(std::string &product_model)
    {
        char buf[64]{};

        auto ec = XynovaDexterousHand_getProductModel(
            c_handle(),
            buf,
            sizeof(buf));

        if (ec == XYNOVA_ERROR_CODE_OK)
            product_model = buf;

        return ec;
    }

    /**
     * @brief Get product serial number.
     *
     * @param product_sn Output serial number string.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getProductSN(std::string &product_sn)
    {
        char buf[64]{};

        auto ec = XynovaDexterousHand_getProductSN(
            c_handle(),
            buf,
            sizeof(buf));

        if (ec == XYNOVA_ERROR_CODE_OK)
            product_sn = buf;

        return ec;
    }

    /**
     * @brief Get hardware version.
     *
     * @param version Output hardware version.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getHardwareVersion(std::string &version)
    {
        char buf[64]{};

        auto ec = XynovaDexterousHand_getHardwareVersion(
            c_handle(),
            buf,
            sizeof(buf));

        if (ec == XYNOVA_ERROR_CODE_OK)
            version = buf;

        return ec;
    }

    /**
     * @brief Get software version.
     *
     * @param version Output software version.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getSoftwareVersion(std::string &version)
    {
        char buf[64]{};

        auto ec = XynovaDexterousHand_getSoftwareVersion(
            c_handle(),
            buf,
            sizeof(buf));

        if (ec == XYNOVA_ERROR_CODE_OK)
            version = buf;

        return ec;
    }

    /**
     * @brief Get hand type.
     *
     * @param type Output hand type.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getHandType(HandType &type)
    {
        return XynovaDexterousHand_getHandType(
            c_handle(),
            &type);
    }

    /**
     * @brief Get current device error state.
     *
     * Retrieves the latest global warning, global fault and
     * joint error information from the dexterous hand.
     *
     * @param state Output error state.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getErrorState(ErrorState &state)
    {
        XynovaDexterousHandErrorState c_state{};

        auto ret = XynovaDexterousHand_getErrorState(
            c_handle(),
            &c_state);

        if (ret != XYNOVA_ERROR_CODE_OK)
            return ret;

        state.timestamp_us = c_state.timestamp_us;
        state.warning_flags = c_state.warning_flags;
        state.fault_flags = c_state.fault_flags;

        for (size_t i = 0; i < JOINT_NUM; ++i)
        {
            auto &joint_error = state.joint_error_codes[i];

            joint_error.id = c_state.joint_error_code[i].id;
            joint_error.error_code.fromC(c_state.joint_error_code[i].data);
        }

        return ret;
    }

    /**
     * @brief Set control mode for all joints.
     *
     * @param mode Joint control mode.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode setJointControlMode(ControlMode mode)
    {
        return XynovaDexterousHand_setJointControlAllMode(c_handle(), mode);
    }

    /**
     * @brief Enable or disable tactile wrench reporting.
     *
     * Controls whether tactile wrench feedback is reported by the device.
     *
     * @param enable
     *        - true: Enable tactile wrench reporting.
     *        - false: Disable tactile wrench reporting.
     *
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode enableTactileWrench(bool enable)
    {
        return XynovaDexterousHand_enableTactileWrench(
            c_handle(),
            enable);
    }

    /**
     * @brief Enable or disable tactile taxel reporting.
     *
     * Controls whether tactile taxel feedback is reported by the device.
     *
     * @param enable
     *        - true: Enable tactile taxel reporting.
     *        - false: Disable tactile taxel reporting.
     *
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode enableTactileTaxel(bool enable)
    {
        return XynovaDexterousHand_enableTactileTaxel(
            c_handle(),
            enable);
    }

    /**
     * @brief Enable or disable range reporting.
     *
     * Controls whether range feedback is reported by the device.
     *
     * @param enable
     *        - true: Enable range reporting.
     *        - false: Disable range reporting.
     *
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode enableRange(bool enable)
    {
        return XynovaDexterousHand_enableRange(
            c_handle(),
            enable);
    }

    /**
     * @brief Set joint angle command.
     *
     * @param angle Joint angle command.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode setJointAngle(const AngleControl &angle)
    {
        XynovaDexterousHandAngleControl command{};

        for (size_t i = 0; i < JOINT_NUM; ++i)
            command.values[i] = angle[i].toC();

        return XynovaDexterousHand_setJointAngle(c_handle(), &command);
    }

    /**
     * @brief Asynchronously set joint angle command.
     *
     * The command is queued and returns immediately.
     *
     * @param angle Joint angle command.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode setJointAngleAsync(const AngleControl &angle)
    {
        XynovaDexterousHandAngleControl command{};

        for (size_t i = 0; i < JOINT_NUM; ++i)
            command.values[i] = angle[i].toC();

        return XynovaDexterousHand_setJointAngle_async(c_handle(), &command);
    }

    /**
     * @brief Get current joint angle states.
     *
     * @param states Output joint angle states.
     * @param timestamp_us Output timestamp in microseconds.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getJointAngle(AngleStates &states, uint64_t &timestamp_us)
    {
        XynovaDexterousHandAngleStates c_states{};

        auto ret = XynovaDexterousHand_getJointAngle(
            c_handle(),
            &c_states,
            &timestamp_us);

        if (ret != XYNOVA_ERROR_CODE_OK)
            return ret;

        for (size_t i = 0; i < JOINT_NUM; ++i)
            states[i].fromC(c_states.values[i]);

        return ret;
    }

    /**
     * @brief Get tactile wrench states.
     *
     * @param states Output tactile wrench states.
     * @param timestamp_us Output timestamp in microseconds.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getTactileWrench(TactileWrenchStates &states, uint64_t &timestamp_us)
    {
        XynovaDexterousHandTactileWrenchStates c_states{};

        auto ret = XynovaDexterousHand_getTactileWrench(
            c_handle(),
            &c_states,
            &timestamp_us);

        if (ret != XYNOVA_ERROR_CODE_OK)
            return ret;

        for (size_t i = 0; i < TACTILE_NUM; ++i)
        {
            states[i].timestamp_us = c_states.values[i].timestamp_us;
            states[i].region = c_states.values[i].region;
            states[i].dimension = c_states.values[i].dimension;
            states[i].wrench.fromC(c_states.values[i].wrench);
        }

        return ret;
    }

    /**
     * @brief Get tactile taxel states.
     *
     * @param states Output tactile taxel states.
     * @param timestamp_us Output timestamp in microseconds.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getTactileTaxel(TactileTaxelStates &states, uint64_t &timestamp_us)
    {
        constexpr uint32_t MAX_TAXEL_NUM = TACTILE_TOTAL_TAXEL_NUM;

        XynovaDexterousHandTactileTaxel buffer[MAX_TAXEL_NUM];

        XynovaDexterousHandTactileTaxelStates c_states{};
        c_states.count = MAX_TAXEL_NUM;
        c_states.taxels = buffer;

        auto ret = XynovaDexterousHand_getTactileTaxel(
            c_handle(),
            &c_states,
            &timestamp_us);

        if (ret != XYNOVA_ERROR_CODE_OK)
            return ret;

        states.resize(c_states.count);

        for (uint32_t i = 0; i < c_states.count; ++i)
        {
            states[i].timestamp_us = buffer[i].timestamp_us;
            states[i].region = buffer[i].region;
            states[i].taxel_id = buffer[i].taxel_id;
            states[i].dimension = buffer[i].dimension;
            states[i].reserved = buffer[i].reserved;
            states[i].wrench.fromC(buffer[i].wrench);
        }

        return ret;
    }

    /**
     * @brief Get range measurement states.
     *
     * Retrieves two-dimensional range measurement data from the dexterous hand.
     *
     * The caller provides the range point buffer.
     * The device fills:
     * - width
     * - height
     * - range point data
     *
     * Points are stored in row-major order:
     *
     *     index = y * width + x
     *
     * @param states Output range measurement states.
     * @param timestamp_us Output timestamp in microseconds.
     * @return ::XynovaErrorCode
     */
    XynovaErrorCode getRange(RangeStates &states, uint64_t &timestamp_us)
    {
        XynovaDexterousHandRangeStates c_states{};

        c_states.count = static_cast<uint32_t>(states.points.size());
        c_states.points = states.points.data();

        auto ret = XynovaDexterousHand_getRange(
            c_handle(),
            &c_states,
            &timestamp_us);

        if (ret != XYNOVA_ERROR_CODE_OK)
            return ret;

        states.width = c_states.width;
        states.height = c_states.height;
        states.points.resize(c_states.count);

        return ret;
    }

private:
    DeviceHandle i_dev_handle; /**< Managed device handle */
};

/**
 * @}
 */

} // namespace dexterous_hand
} // namespace hand
} // namespace xynova

#endif /* XYNOVA_CPP_DEXTEROUS_HAND_DEVICE_HPP */
