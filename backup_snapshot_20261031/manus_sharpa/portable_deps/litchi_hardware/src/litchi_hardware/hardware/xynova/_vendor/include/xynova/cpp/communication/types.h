/**
 * @file types.h
 * @brief C++ wrapper definitions for Xynova communication system configuration.
 *
 * This header provides modern C++ representations of Xynova communication
 * configuration structures, bridging the low-level C API
 * (`xynova/c/comm_types.h`) with STL-based and type-safe C++ structures.
 *
 * ## Features
 * - Defines communication types and configuration structures.
 * - Supports master/slave communication topology.
 * - Provides unified manager configuration.
 * - Uses modern C++ containers and strong typed enums.
 *
 * These types are located in the `xynova::communication` namespace and are
 * designed to support future communication transports such as CAN, EtherCAT,
 * TCP, and serial communication.
 *
 * @version 1.0.0
 * @date 2025-10-29
 * @author Xynova
 *
 * @see ::XynovaCommunicationManagerConfig
 * @see xynova::communication::Manager
 */

#ifndef XYNOVA_CPP_COMMUNICATION_TYPES_H
#define XYNOVA_CPP_COMMUNICATION_TYPES_H

#include <string>
#include <vector>

#include <cstdint>

#include "xynova/c/comm_types.h"
#include "xynova/cpp/communication/can/types.h"

namespace xynova {
namespace communication {

/**
 * @defgroup xynova_cpp_communication_types Communication C++ Types
 * @brief High-level C++ communication configuration types.
 * @{
 */

/**
 * @enum CommType
 * @brief Supported communication transport types.
 */
enum class CommType : uint32_t
{
    Can = XYNOVA_COMMUNICATION_COMM_TYPE_CAN /**< Controller Area Network */
};

/**
 * @struct CommConfig
 * @brief Communication interface configuration.
 *
 * Provides a common configuration layer for different communication backends.
 */
struct CommConfig
{
    CommType type;           /**< Communication type */
    std::string sub_type;    /**< Communication driver subtype, e.g. "SockCAN" */
    can::CanConfig can;      /**< CAN specific configuration */
    std::string logger_name; /**< Logger name */
};

/**
 * @struct MasterConfig
 * @brief Configuration of a communication master.
 *
 * A master represents a communication controller managing one or more devices.
 */
struct MasterConfig
{
    std::string name;   /**< Master name */
    CommConfig comm;    /**< Communication configuration */
    uint64_t period_us; /**< Control loop period in microseconds */
    int32_t cpu_index;  /**< CPU affinity index, -1 disables binding */
};

/**
 * @struct ResponseTimeSetting
 * @brief Communication response timeout and retry configuration.
 *
 * This structure corresponds to ::XynovaCommunicationResponseTimeSetting.
 */
struct ResponseTimeSetting
{
    uint32_t user_max_timeout_us;      /**< Maximum timeout for user API calls */
    uint32_t user_poll_interval_us;    /**< User polling interval */
    uint32_t execute_timeout_us;       /**< Backend request timeout */
    uint32_t execute_max_retry;        /**< Backend retry count */
    uint32_t execute_poll_interval_us; /**< Backend polling interval */
};

/**
 * @struct SlaveConfig
 * @brief Configuration of a communication slave device.
 */
struct SlaveConfig
{
    std::string type; /**< Slave type or model name */
    uint32_t id;      /**< Slave identifier */

    ResponseTimeSetting general_time_setting; /**< General response timing */
    ResponseTimeSetting fast_time_setting;    /**< Fast response timing */

    std::string logger_name; /**< Logger name */
};

/**
 * @struct SchedulingCommand
 * @brief Periodic command scheduling configuration.
 *
 * Defines commands that are periodically sent to or reported from devices.
 */
struct SchedulingCommand
{
    std::string command;                       /**< Command name */
    ResponseTimeSetting response_time_setting; /**< Response timing configuration */
};

/**
 * @struct DeviceConfig
 * @brief Logical device configuration.
 *
 * A device may contain multiple communication slaves and scheduled commands.
 */
struct DeviceConfig
{
    std::string type; /**< Device type */
    std::string name; /**< Device name */

    std::string master_name; /**< Associated master name */

    std::vector<SlaveConfig> slaves; /**< Slave configuration list */
    std::vector<SchedulingCommand> commands; /**< Scheduled command list */
};

/**
 * @struct ManagerConfig
 * @brief Complete communication manager configuration.
 *
 * Contains all communication masters and logical devices required to
 * initialize the communication subsystem.
 *
 * @see xynova::communication::Manager
 */
struct ManagerConfig
{
    std::vector<MasterConfig> masters; /**< Master configuration list */
    std::vector<DeviceConfig> devices;  /**< Device configuration list */
};

/**
 * @}
 */

} // namespace communication
} // namespace xynova

#endif // XYNOVA_CPP_COMMUNICATION_TYPES_H