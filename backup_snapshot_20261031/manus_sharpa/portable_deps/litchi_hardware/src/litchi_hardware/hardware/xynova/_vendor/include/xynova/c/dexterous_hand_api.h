/**
 * @file dexterous_hand_api.h
 * @brief C API for Xynova Dexterous Hand control and monitoring.
 *
 * This header provides the public C interface for controlling and monitoring
 * Xynova Dexterous Hand devices, including device lifecycle management,
 * joint control, and joint state acquisition.
 *
 * ## Features
 * - Start and stop dexterous hand devices.
 * - Configure joint control modes.
 * - Send synchronous and asynchronous joint commands.
 * - Read current joint states with timestamps.
 *
 * @version 1.0.0
 * @date 2026-7-17
 * @author Xynova
 *
 * @see ::XynovaDexterousHandAngleControl
 * @see ::XynovaDexterousHandAngleStates
 * @see ::XynovaDeviceHandle
 */

#ifndef XYNOVA_C_DEXTEROUS_HAND_API_H
#define XYNOVA_C_DEXTEROUS_HAND_API_H

#include "xynova/c/abi.h"
#include "xynova/c/error_code.h"
#include "xynova/c/comm_api.h"
#include "xynova/c/dexterous_hand_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup xynova_dexterous_hand_api Dexterous Hand API
 * @brief Public C API for Xynova Dexterous Hand devices.
 * @{
 */

/**
 * @brief Start the dexterous hand device.
 *
 * Initializes and starts device communication.
 *
 * @param device Device handle obtained from ::XynovaCommunicationManager_getDevice.
 * @return ::XynovaErrorCode
 *         - XYNOVA_ERROR_CODE_OK on success
 *         - Other codes indicate startup failure.
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_run(XynovaDeviceHandle *device);

/**
 * @brief Stop the dexterous hand device.
 *
 * @param device Device handle.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_stop(XynovaDeviceHandle *device);

/**
 * @brief Get product model.
 *
 * Reads the product model string from the device.
 *
 * @param device Device handle.
 * @param product_model Output buffer for product model string.
 * @param size Size of @p product_model in bytes.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getProductModel(
    XynovaDeviceHandle *device,
    char *product_model,
    uint32_t size);

/**
 * @brief Get product serial number.
 *
 * Reads the product serial number from the device.
 *
 * @param device Device handle.
 * @param product_sn Output buffer for serial number string.
 * @param size Size of @p product_sn in bytes.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getProductSN(
    XynovaDeviceHandle *device,
    char *product_sn,
    uint32_t size);

/**
 * @brief Get hardware version.
 *
 * Reads the hardware version string.
 *
 * @param device Device handle.
 * @param version Output buffer for hardware version.
 * @param size Size of @p version in bytes.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getHardwareVersion(
    XynovaDeviceHandle *device,
    char *version,
    uint32_t size);

/**
 * @brief Get software version.
 *
 * Reads the software version string.
 *
 * @param device Device handle.
 * @param version Output buffer for software version.
 * @param size Size of @p version in bytes.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getSoftwareVersion(
    XynovaDeviceHandle *device,
    char *version,
    uint32_t size);

/**
 * @brief Get hand type.
 *
 * Reads the hand type from the device.
 *
 * @param device Device handle.
 * @param type Output hand type.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getHandType(
    XynovaDeviceHandle *device,
    XynovaDexterousHandType *type);

/**
 * @brief Get current device error state.
 *
 * Retrieves the latest global warning, global fault and joint error
 * information from the dexterous hand.
 *
 * The returned data contains:
 * - Global warning flags.
 * - Global fault flags.
 * - Joint error codes for all joints.
 *
 * @param device Device handle.
 * @param state Output error state.
 *
 * @return ::XynovaErrorCode
 *         - XYNOVA_ERROR_CODE_OK on success.
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getErrorState(
    XynovaDeviceHandle *device,
    XynovaDexterousHandErrorState *state);

/**
 * @brief Set control mode for all joints.
 *
 * @param device Device handle.
 * @param mode Joint control mode.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_setJointControlAllMode(
    XynovaDeviceHandle *device,
    XynovaDexterousHandControlMode mode);

/**
 * @brief Enable or disable tactile wrench reporting.
 *
 * Controls whether the device publishes tactile wrench data.
 * When disabled, tactile wrench feedback is not transmitted.
 *
 * @param device Device handle.
 * @param enable
 *        - true: Enable tactile wrench reporting.
 *        - false: Disable tactile wrench reporting.
 *
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_enableTactileWrench(
    XynovaDeviceHandle *device,
    bool enable);

/**
 * @brief Enable or disable tactile taxel reporting.
 *
 * Controls whether the device publishes detailed tactile taxel data.
 * When disabled, tactile taxel feedback is not transmitted.
 *
 * @param device Device handle.
 * @param enable
 *        - true: Enable tactile taxel reporting.
 *        - false: Disable tactile taxel reporting.
 *
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_enableTactileTaxel(
    XynovaDeviceHandle *device,
    bool enable);

/**
 * @brief Enable or disable range reporting.
 *
 * Controls whether the device publishes range measurement data.
 * When disabled, range feedback is not transmitted.
 *
 * @param device Device handle.
 * @param enable
 *        - true: Enable range reporting.
 *        - false: Disable range reporting.
 *
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_enableRange(
    XynovaDeviceHandle *device,
    bool enable);

/**
 * @brief Set joint angle control command.
 *
 * Sends joint angle commands and waits until execution is completed.
 *
 * @param device Device handle.
 * @param angle Joint angle command.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_setJointAngle(
    XynovaDeviceHandle *device,
    const XynovaDexterousHandAngleControl *angle);

/**
 * @brief Asynchronously set joint angle control command.
 *
 * Queues the joint command and returns immediately.
 * The execution is handled internally by the communication backend.
 *
 * @param device Device handle.
 * @param angle Joint angle command.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_setJointAngle_async(
    XynovaDeviceHandle *device,
    const XynovaDexterousHandAngleControl *angle);

/**
 * @brief Get current joint angle states.
 *
 * Retrieves the latest joint state data from the device.
 *
 * @param device Device handle.
 * @param states Output joint angle states.
 * @param timestamp_us Output timestamp in microseconds.
 *        Can be NULL when timestamp is not required.
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getJointAngle(
    XynovaDeviceHandle *device,
    XynovaDexterousHandAngleStates *states,
    uint64_t *timestamp_us);

/**
 * @brief Get tactile wrench states.
 *
 * Retrieves the latest tactile wrench data from the dexterous hand.
 *
 * Each tactile region contains:
 * - timestamp_us: Sampling timestamp in microseconds.
 * - region: Tactile region identifier.
 * - dimension: Valid wrench dimension.
 * - wrench: Force/torque values.
 *
 * The wrench vector is defined as:
 *
 *     [Fx, Fy, Fz, Tx, Ty, Tz]
 *
 * where:
 * - F*: Force components. Unit: N (Newtons)
 * - T*: Torque components. Unit: Nm (Newton-meters)
 *
 * @param device Device handle.
 * @param states Output tactile wrench states.
 * @param timestamp_us Output timestamp in microseconds.
 *        Can be NULL when timestamp is not required.
 *
 * @return ::XynovaErrorCode
 *         - XYNOVA_ERROR_CODE_OK on success.
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getTactileWrench(
    XynovaDeviceHandle *device,
    XynovaDexterousHandTactileWrenchStates *states,
    uint64_t *timestamp_us);

/**
 * @brief Get tactile taxel states.
 *
 * Retrieves detailed tactile taxel data from the dexterous hand.
 *
 * The caller provides the taxel buffer.
 * The buffer size must be sufficient for the requested data.
 *
 * @param device Device handle.
 * @param states Output tactile taxel states.
 * @param timestamp_us Output timestamp in microseconds.
 *        Can be NULL when timestamp is not required.
 *
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getTactileTaxel(
    XynovaDeviceHandle *device,
    XynovaDexterousHandTactileTaxelStates *states,
    uint64_t *timestamp_us);

/**
 * @brief Get range measurement states.
 *
 * Retrieves the latest range measurement data from the dexterous hand.
 *
 * The caller provides the range point buffer.
 * The buffer size must be sufficient for the requested data.
 *
 * Points are stored in row-major order:
 *
 *     index = y * width + x
 *
 * Each point contains:
 * - timestamp_us: Sampling timestamp.
 * - id: Point identifier.
 * - status: Measurement status.
 * - distance: Measured distance in meters.
 *
 * @param device Device handle.
 * @param states Output range measurement states.
 * @param timestamp_us Output timestamp in microseconds.
 *        Represents the latest update time of the returned range states.
 *        Can be NULL when timestamp is not required.
 *
 * @return ::XynovaErrorCode
 */
XYNOVA_API XynovaErrorCode XynovaDexterousHand_getRange(
    XynovaDeviceHandle *device,
    XynovaDexterousHandRangeStates *states,
    uint64_t *timestamp_us);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* XYNOVA_C_DEXTEROUS_HAND_API_H */