/**
 * @file dexterous_hand_types.h
 * @brief Xynova Dexterous Hand type definitions.
 *
 * This header defines fundamental data types used by the Xynova Dexterous Hand
 * API, including joint identifiers, control modes, joint commands, and feedback
 * state structures.
 *
 * ## Features
 * - Defines supported joint identifiers.
 * - Defines joint control modes.
 * - Provides joint command and state data structures.
 * - Maintains C ABI compatibility.
 *
 * @version 1.0.0
 * @date 2026-7-17
 * @author Xynova
 *
 * @see ::XynovaDexterousHandAngleControl
 * @see ::XynovaDexterousHandAngleStates
 */

#ifndef XYNOVA_C_DEXTEROUS_HAND_TYPES_H
#define XYNOVA_C_DEXTEROUS_HAND_TYPES_H

#include "xynova/c/value_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup xynova_dexterous_hand_types Dexterous Hand Types
 * @brief Data types for Xynova Dexterous Hand control and state.
 * @{
 */

/**
 * @brief Number of joints supported by Xynova Dexterous Hand.
 */
#define XYNOVA_DEXTEROUS_HAND_JOINT_NUM 23

/**
 * @brief Number of tactile regions supported by Xynova Dexterous Hand.
 */
#define XYNOVA_DEXTEROUS_HAND_TACTILE_NUM 15

/**
 * @brief Total number of tactile taxels supported by Xynova Dexterous Hand.
 */
#define XYNOVA_DEXTEROUS_HAND_TACTILE_TOTAL_TAXEL_NUM 312

/**
 * @brief Number of range points supported by the default range sensor.
 */
#define XYNOVA_DEXTEROUS_HAND_RANGE_POINT_NUM 16

/**
 * @brief Dexterous hand error code type.
 *
 * Stored as uint16_t to guarantee ABI compatibility.
 *
 * Each bit represents a joint error condition.
 */
typedef uint16_t XynovaDexterousHandErrorCode;

/**
 * @brief Dexterous hand joint error flags.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_ERROR_NONE                 = 0x0000, /**< No error */
    XYNOVA_DEXTEROUS_HAND_ERROR_STALL_PROTECTION     = 0x0001, /**< Motor stall protection */
    XYNOVA_DEXTEROUS_HAND_ERROR_OVER_TEMPERATURE     = 0x0002, /**< Over temperature protection */
    XYNOVA_DEXTEROUS_HAND_ERROR_OVER_CURRENT         = 0x0004, /**< Over current protection */
    XYNOVA_DEXTEROUS_HAND_ERROR_MOTOR_ABNORMAL       = 0x0008, /**< Motor abnormal */
    XYNOVA_DEXTEROUS_HAND_ERROR_COMMUNICATION        = 0x0010, /**< Communication error */
    XYNOVA_DEXTEROUS_HAND_ERROR_ENCODER_ABNORMAL     = 0x0020  /**< Encoder abnormal */
};

/**
 * @brief Dexterous hand global warning flags.
 *
 * Stored as uint32_t to guarantee ABI compatibility.
 *
 * Each bit represents a global warning condition.
 */
typedef uint32_t XynovaDexterousHandWarningFlags;

/**
 * @brief Dexterous hand global warning flags.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_WARNING_NONE                           = 0x00000000U, /**< No warning */

    XYNOVA_DEXTEROUS_HAND_WARNING_ANGLE_SENSOR_COMMUNICATION     = 1U << 0,  /**< Angle sensor communication abnormal */
    XYNOVA_DEXTEROUS_HAND_WARNING_TACTILE_SENSOR_COMMUNICATION   = 1U << 1,  /**< Tactile sensor communication abnormal */
    XYNOVA_DEXTEROUS_HAND_WARNING_RANGE_SENSOR_COMMUNICATION     = 1U << 2,  /**< Range (dToF) sensor communication abnormal */

    XYNOVA_DEXTEROUS_HAND_WARNING_MAINBOARD_OVER_TEMPERATURE     = 1U << 3,  /**< Main control board over temperature */
    XYNOVA_DEXTEROUS_HAND_WARNING_MAINBOARD_UNDER_TEMPERATURE    = 1U << 4,  /**< Main control board under temperature */

    XYNOVA_DEXTEROUS_HAND_WARNING_SUBBOARD_OVER_TEMPERATURE      = 1U << 5,  /**< Sub board over temperature */
    XYNOVA_DEXTEROUS_HAND_WARNING_SUBBOARD_UNDER_TEMPERATURE     = 1U << 6,  /**< Sub board under temperature */

    XYNOVA_DEXTEROUS_HAND_WARNING_CANFD1_STATUS                  = 1U << 7,  /**< CAN FD1 status warning */
    XYNOVA_DEXTEROUS_HAND_WARNING_CANFD2_STATUS                  = 1U << 8,  /**< CAN FD2 status warning */

    XYNOVA_DEXTEROUS_HAND_WARNING_FIRMWARE_UPGRADE               = 1U << 9,  /**< Firmware upgrade warning */
    XYNOVA_DEXTEROUS_HAND_WARNING_PDM_FRAME_LOSS                 = 1U << 10, /**< PDM frame loss */

    /* Bit 11 reserved */

    XYNOVA_DEXTEROUS_HAND_WARNING_CONTROL_LOOP_TIMEOUT           = 1U << 11  /**< Control loop timeout */
};

/**
 * @brief Dexterous hand global fault flags.
 *
 * Stored as uint32_t to guarantee ABI compatibility.
 *
 * Each bit represents a global fault condition.
 */
typedef uint32_t XynovaDexterousHandFaultFlags;

/**
 * @brief Dexterous hand global fault flags.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_FAULT_NONE                           = 0x00000000U, /**< No fault */

    XYNOVA_DEXTEROUS_HAND_FAULT_SUBBOARD_COMMUNICATION         = 1U << 0,  /**< Sub board communication abnormal */
    XYNOVA_DEXTEROUS_HAND_FAULT_ACTUATOR_COMMUNICATION         = 1U << 1,  /**< Actuator communication abnormal */
    XYNOVA_DEXTEROUS_HAND_FAULT_ANGLE_SENSOR_COMMUNICATION     = 1U << 2,  /**< Angle sensor communication abnormal */
    XYNOVA_DEXTEROUS_HAND_FAULT_TACTILE_SENSOR_COMMUNICATION   = 1U << 3,  /**< Tactile sensor communication abnormal */
    XYNOVA_DEXTEROUS_HAND_FAULT_RANGE_SENSOR_COMMUNICATION     = 1U << 4,  /**< Range (dToF) sensor communication abnormal */

    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_OVER_TEMPERATURE     = 1U << 5,  /**< Main control board over temperature */
    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_UNDER_TEMPERATURE    = 1U << 6,  /**< Main control board under temperature */
    XYNOVA_DEXTEROUS_HAND_FAULT_SUBBOARD_OVER_TEMPERATURE      = 1U << 7,  /**< Sub board over temperature */
    XYNOVA_DEXTEROUS_HAND_FAULT_SUBBOARD_UNDER_TEMPERATURE     = 1U << 8,  /**< Sub board under temperature */

    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_48V_OVER_VOLTAGE     = 1U << 9,  /**< Main board 48V over voltage */
    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_24V_OVER_VOLTAGE     = 1U << 10, /**< Main board 24V over voltage */
    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_48V_UNDER_VOLTAGE    = 1U << 11, /**< Main board 48V under voltage */
    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_24V_UNDER_VOLTAGE    = 1U << 12, /**< Main board 24V under voltage */

    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_OVER_CURRENT1        = 1U << 13, /**< Main board over current channel 1 */
    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_OVER_CURRENT2        = 1U << 14, /**< Main board over current channel 2 */
    XYNOVA_DEXTEROUS_HAND_FAULT_MAINBOARD_OVER_CURRENT3        = 1U << 15, /**< Main board over current channel 3 */

    XYNOVA_DEXTEROUS_HAND_FAULT_CANFD1_STATUS                  = 1U << 16, /**< CAN FD1 status fault */
    XYNOVA_DEXTEROUS_HAND_FAULT_CANFD2_STATUS                  = 1U << 17, /**< CAN FD2 status fault */

    XYNOVA_DEXTEROUS_HAND_FAULT_PARAMETER_STORAGE              = 1U << 18, /**< Parameter storage fault */

    XYNOVA_DEXTEROUS_HAND_FAULT_ACTUATOR_OVER_TEMPERATURE      = 1U << 19, /**< Actuator over temperature */
    XYNOVA_DEXTEROUS_HAND_FAULT_ACTUATOR_OVER_CURRENT          = 1U << 20, /**< Actuator over current */
    XYNOVA_DEXTEROUS_HAND_FAULT_ACTUATOR_UNDER_VOLTAGE         = 1U << 21, /**< Actuator under voltage */
    XYNOVA_DEXTEROUS_HAND_FAULT_ACTUATOR_OVER_VOLTAGE          = 1U << 22, /**< Actuator over voltage */

    XYNOVA_DEXTEROUS_HAND_FAULT_PDM_FRAME_LOSS                = 1U << 23, /**< PDM frame loss */
    XYNOVA_DEXTEROUS_HAND_FAULT_CONTROL_LOOP_TIMEOUT          = 1U << 24  /**< Control loop timeout */
};

/**
 * @brief Dexterous hand error state.
 *
 * Provides global warning, global fault and joint error information.
 */
typedef struct XynovaDexterousHandErrorState
{
    uint64_t timestamp_us; /**< Error timestamp (us): system monotonic clock */

    XynovaDexterousHandWarningFlags warning_flags; /**< Global warning flags */

    XynovaDexterousHandFaultFlags fault_flags; /**< Global fault flags */

    XynovaIdValue joint_error_code[XYNOVA_DEXTEROUS_HAND_JOINT_NUM]; /**< Joint error codes */
} XynovaDexterousHandErrorState;

/**
 * @brief Dexterous hand type.
 *
 * Stored as uint8_t to guarantee ABI compatibility.
 */
typedef uint8_t XynovaDexterousHandType;

/**
 * @brief Dexterous hand types.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_TYPE_UNKNOWN = 0, /**< Unknown hand type */
    XYNOVA_DEXTEROUS_HAND_TYPE_LEFT    = 1, /**< Left hand */
    XYNOVA_DEXTEROUS_HAND_TYPE_RIGHT   = 2, /**< Right hand */
};

/**
 * @brief Dexterous hand joint identifier type.
 *
 * Stored as uint8_t to guarantee ABI compatibility.
 */
typedef uint8_t XynovaDexterousHandJointID;

/**
 * @brief Dexterous hand joint identifiers.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_FF1 = 1,  /**< Index fingertip flex (Passive) */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_FF2 = 2,  /**< Index middle flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_FF3 = 3,  /**< Index base flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_FF4 = 4,  /**< Index abduction */

    XYNOVA_DEXTEROUS_HAND_JOINT_ID_MF1 = 5,  /**< Middle fingertip flex (Passive) */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_MF2 = 6,  /**< Middle middle flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_MF3 = 7,  /**< Middle base flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_MF4 = 8,  /**< Middle abduction */

    XYNOVA_DEXTEROUS_HAND_JOINT_ID_RF1 = 9,  /**< Ring fingertip flex (Passive) */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_RF2 = 10, /**< Ring middle flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_RF3 = 11, /**< Ring base flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_RF4 = 12, /**< Ring abduction */

    XYNOVA_DEXTEROUS_HAND_JOINT_ID_LF1 = 13, /**< Little fingertip flex (Passive) */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_LF2 = 14, /**< Little middle flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_LF3 = 15, /**< Little base flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_LF4 = 16, /**< Little abduction */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_LF5 = 17, /**< Little rotation */

    XYNOVA_DEXTEROUS_HAND_JOINT_ID_TH1 = 18, /**< Thumb tip flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_TH2 = 19, /**< Thumb middle flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_TH3 = 20, /**< Thumb base flex */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_TH4 = 21, /**< Thumb abduction */

    XYNOVA_DEXTEROUS_HAND_JOINT_ID_WR1 = 22, /**< Wrist pitch */
    XYNOVA_DEXTEROUS_HAND_JOINT_ID_WR2 = 23  /**< Wrist yaw */
};

/**
 * @brief Dexterous hand tactile region identifier type.
 *
 * Stored as uint8_t to guarantee ABI compatibility.
 */
typedef uint8_t XynovaDexterousHandTactileID;

/**
 * @brief Dexterous hand tactile region identifiers.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_TT1  = 0x01, /**< Thumb fingertip */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_TT2  = 0x02, /**< Thumb pad */

    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_FT1  = 0x03, /**< Index fingertip */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_FT2  = 0x04, /**< Index middle phalanx */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_FT3  = 0x05, /**< Index proximal phalanx */

    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_MT1  = 0x06, /**< Middle fingertip */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_MT2  = 0x07, /**< Middle middle phalanx */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_MT3  = 0x08, /**< Middle proximal phalanx */

    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_RT1  = 0x09, /**< Ring fingertip */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_RT2  = 0x0A, /**< Ring middle phalanx */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_RT3  = 0x0B, /**< Ring proximal phalanx */

    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_LT1  = 0x0C, /**< Little fingertip */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_LT2  = 0x0D, /**< Little middle phalanx */
    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_LT3  = 0x0E, /**< Little proximal phalanx */

    XYNOVA_DEXTEROUS_HAND_TACTILE_ID_PT1  = 0x0F  /**< Palm */
};

/**
 * @brief Tactile force dimension type.
 *
 * Stored as uint8_t to guarantee ABI compatibility.
 */
typedef uint8_t XynovaDexterousHandWrenchDimension;

/**
 * @brief Tactile wrench dimensions.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_WRENCH_DIMENSION_1D = 1, /**< One-dimensional wrench */
    XYNOVA_DEXTEROUS_HAND_WRENCH_DIMENSION_3D = 3, /**< Three-dimensional wrench */
    XYNOVA_DEXTEROUS_HAND_WRENCH_DIMENSION_6D = 6  /**< Six-dimensional wrench */
};

/**
 * @brief Dexterous hand control mode type.
 *
 * Stored as uint8_t to guarantee ABI compatibility.
 */
typedef uint8_t XynovaDexterousHandControlMode;

/**
 * @brief Dexterous hand control modes.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_CONTROL_MODE_DISABLE              = 0x00, /**< Control disabled */
    XYNOVA_DEXTEROUS_HAND_CONTROL_MODE_ANGLE                = 0x02, /**< Angle control */
};

/**
 * @brief Range point status type.
 *
 * Stored as uint8_t to guarantee ABI compatibility.
 */
typedef uint8_t XynovaDexterousHandRangePointStatus;

/**
 * @brief Range point measurement status.
 */
enum {
    XYNOVA_DEXTEROUS_HAND_RANGE_POINT_STATUS_NO_TARGET    = 0, /**< No target detected */
    XYNOVA_DEXTEROUS_HAND_RANGE_POINT_STATUS_WEAK_SIGNAL  = 1, /**< Weak signal */
    XYNOVA_DEXTEROUS_HAND_RANGE_POINT_STATUS_VALID        = 2, /**< Valid measurement */
    XYNOVA_DEXTEROUS_HAND_RANGE_POINT_STATUS_INVALID      = 3  /**< Invalid measurement */
};

/**
 * @brief Joint control mode command.
 *
 * Each element contains:
 * - id: Joint identifier
 * - type: ::XYNOVA_DATA_TYPE_U8
 * - value.u8: ::XynovaDexterousHandControlMode
 */
typedef struct XynovaDexterousHandControlModeCommand {
    XynovaIdValue values[XYNOVA_DEXTEROUS_HAND_JOINT_NUM]; /**< Joint control mode values */
} XynovaDexterousHandControlModeCommand;

/**
 * @brief Joint angle control command.
 *
 * Each element contains:
 * - id: Joint identifier
 * - type: ::XYNOVA_DATA_TYPE_FLOAT
 * - value.f32: Joint angle in radians
 */
typedef struct XynovaDexterousHandAngleControl {
    XynovaIdValue values[XYNOVA_DEXTEROUS_HAND_JOINT_NUM]; /**< Joint angle commands */
} XynovaDexterousHandAngleControl;

/**
 * @brief Joint angle feedback states.
 *
 * Each element contains:
 * - id: Joint identifier
 * - type: ::XYNOVA_DATA_TYPE_FLOAT
 * - value.f32: Joint angle in radians
 */
typedef struct XynovaDexterousHandAngleStates {
    XynovaIdValue values[XYNOVA_DEXTEROUS_HAND_JOINT_NUM]; /**< Joint angle states */
} XynovaDexterousHandAngleStates;

/**
 * @brief Tactile resultant force.
 */
typedef struct XynovaDexterousHandTactileWrench
{
    uint64_t timestamp_us;                       /**< Sampling timestamp (us): system monotonic clock */
    XynovaDexterousHandTactileID region;         /**< Tactile region */
    XynovaDexterousHandWrenchDimension dimension; /**< Force dimension */
    XynovaTypedValueArray6 wrench;               /**< Force/torque values: force unit: N, torque unit: Nm */
} XynovaDexterousHandTactileWrench;

/**
 * @brief Tactile resultant force states.
 */
typedef struct XynovaDexterousHandTactileWrenchStates
{
    XynovaDexterousHandTactileWrench
        values[XYNOVA_DEXTEROUS_HAND_TACTILE_NUM];
} XynovaDexterousHandTactileWrenchStates;

/**
 * @brief Tactile taxel force.
 */
typedef struct XynovaDexterousHandTactileTaxel
{
    uint64_t timestamp_us;                      /**< Sampling timestamp (us): system monotonic clock */
    XynovaDexterousHandTactileID region;        /**< Tactile region */
    uint16_t taxel_id;                          /**< Taxel identifier */
    XynovaDexterousHandWrenchDimension dimension; /**< Force dimension */
    uint8_t reserved;                            /**< Reserved */
    XynovaTypedValueArray6 wrench;              /**< Force/torque values: force unit: N, torque unit: Nm */
} XynovaDexterousHandTactileTaxel;

/**
 * @brief Tactile taxel states.
 *
 * The taxels buffer is managed by the caller.
 */
typedef struct XynovaDexterousHandTactileTaxelStates
{
    uint32_t count;                                   /**< Number of taxels */
    XynovaDexterousHandTactileTaxel *taxels;          /**< Taxel array */
} XynovaDexterousHandTactileTaxelStates;

/**
 * @brief Range point measurement.
 *
 * Distance unit: meter.
 */
typedef struct XynovaDexterousHandRangePoint
{
    uint64_t timestamp_us; /**< Measurement timestamp (us): system monotonic clock */
    uint16_t id;           /**< Point identifier */
    XynovaDexterousHandRangePointStatus status; /**< Measurement status */
    uint8_t reserved;      /**< Reserved */
    float distance;        /**< Distance in meter */
} XynovaDexterousHandRangePoint;

/**
 * @brief Range measurement states.
 *
 * The points buffer is managed by the caller.
 *
 * Points are stored in row-major order:
 * index = y * width + x
 */
typedef struct XynovaDexterousHandRangeStates
{
    uint16_t width;  /**< Range map width */
    uint16_t height; /**< Range map height */
    uint32_t count;  /**< Number of range points */
    XynovaDexterousHandRangePoint *points; /**< Range point buffer */
} XynovaDexterousHandRangeStates;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* XYNOVA_C_DEXTEROUS_HAND_TYPES_H */