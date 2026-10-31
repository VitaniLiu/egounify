/**
 * @file can_types.hpp
 * @brief C++ CAN communication type definitions for the Xynova SDK.
 *
 * Provides type-safe C++ wrappers for CAN communication configuration,
 * frame types and receive filters.
 *
 * This header bridges the C API definitions from
 * ::xynova/c/can_types.h with modern C++ interfaces.
 *
 * @version 1.0.0
 * @date 2025-10-29
 * @author Xynova
 *
 * @see ::XynovaCommunicationCanConfig
 */

#ifndef XYNOVA_CPP_CAN_TYPES_HPP
#define XYNOVA_CPP_CAN_TYPES_HPP

#include <string>

#include <cstdint>

#include "xynova/c/can_types.h"

namespace xynova {
namespace communication {
namespace can {

/**
 * @defgroup xynova_cpp_can C++ CAN Types
 * @brief C++ CAN communication wrappers.
 * @{
 */

/**
 * @brief CAN interface configuration.
 *
 * Supports both classical CAN and CAN-FD configuration.
 */
struct CanConfig {
    std::string iface; /**< CAN interface name, e.g. "can0" */
    bool manager_can = false; /**< Whether SDK manages CAN interface setup */
    uint32_t bitrate = 0; /**< Arbitration bitrate in bits per second */
    uint32_t dbitrate = 0; /**< CAN-FD data bitrate in bits per second */
    float sample_point = 0.75f; /**< Arbitration sample point ratio */
    float dsample_point = 0.75f; /**< CAN-FD data sample point ratio */
};

/**
 * @brief CAN frame identifier type.
 */
enum class FrameType : uint32_t {
    Standard = XYNOVA_COMMUNICATION_CAN_FRAME_TYPE_STANDARD, /**< Standard 11-bit CAN identifier */
    Extended = XYNOVA_COMMUNICATION_CAN_FRAME_TYPE_EXTENDED /**< Extended 29-bit CAN identifier */
};

/**
 * @brief CAN identifier mask.
 */
enum class CanMask : uint32_t {
    Standard = XYNOVA_COMMUNICATION_CAN_MASK_STANDARD, /**< Standard 11-bit identifier mask */
    Extended = XYNOVA_COMMUNICATION_CAN_MASK_EXTENDED /**< Extended 29-bit identifier mask */
};

/**
 * @brief CAN receive filter.
 *
 * Filters CAN frames using identifier and mask:
 *
 * ```
 * (received_id & mask) == (id & mask)
 * ```
 */
struct RecvFilter {
    FrameType type; /**< CAN frame type */
    uint32_t id; /**< CAN identifier */
    uint32_t mask; /**< CAN identifier mask */
};

/**
 * @}
 */

} // namespace can
} // namespace communication
} // namespace xynova

#endif /* XYNOVA_CPP_CAN_TYPES_HPP */