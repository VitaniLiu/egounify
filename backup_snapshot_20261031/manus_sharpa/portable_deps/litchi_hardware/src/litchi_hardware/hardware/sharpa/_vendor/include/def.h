
/**
 * @file def.h
 * @brief class definitions used by other headers
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "tensor.h"

namespace sharpa {
namespace tactile {

/**
 * @brief base class of Logger, for user to implement custom Loggers 
 */
class LoggerBase {
public:
    /**
     * @brief for logging debug messages
     * @param msg message
     */
    virtual void debug(const std::string &msg) = 0;
    /**
     * @brief for logging info messages
     * @param msg message
     */
    virtual void info(const std::string &msg) = 0;
    /**
     * @brief for logging warn messages
     * @param msg message
     */
    virtual void warn(const std::string &msg) = 0;
    /**
     * @brief for logging error messages
     * @param msg message
     */
    virtual void error(const std::string &msg) = 0;

    /**
     * @brief for destructing LoggerBase
     */
    virtual ~LoggerBase() = default;
};

/**
 * @brief tactile data frame
 */
struct Frame {
    /** pointer of data frame */
    using Ptr = std::shared_ptr<Frame>;
    /** frame id, unique and increasing for each tactile sensor
     * (reset by next hardware restart)
     * (as 32-bit integer, it may overflow)
     */
    int frame_id;
    /** channel, see Touch constructor */
    int channel;
    /** time stamp */
    double ts;
    /**
     * contents
     * key can be ["RAW", "DEFORM", "F6", ...]
     */
    std::map<std::string, DataBlock::Ptr> content;
    bool is_to_infer() const;
    /**
     * copy a frame
     */
    Frame::Ptr copy() const;
};

enum class TACTILE_TAG{
    WAVE_HA4=0,
    WAVE_HB1=1,
};

// Base coordinate frame each eskin point is expressed in. Names match the URDF
// link names (see right_sharpa_wave.urdf). The right- and left-hand blocks share
// the same order; the left value is always the right value plus kEskinBaseHandStride.
enum class ESKIN_BASE_COORDINATE {
    RIGHT_PINKY_PP=0,
    RIGHT_PINKY_MP=1,
    RIGHT_RING_PP=2,
    RIGHT_RING_MP=3,
    RIGHT_MIDDLE_PP=4,
    RIGHT_MIDDLE_MP=5,
    RIGHT_INDEX_PP=6,
    RIGHT_INDEX_MP=7,
    RIGHT_THUMB_PP=8,
    RIGHT_THUMB_MC=9,
    RIGHT_HAND_C_MC=10,
    RIGHT_PINKY_MC=11,

    LEFT_PINKY_PP=12,
    LEFT_PINKY_MP=13,
    LEFT_RING_PP=14,
    LEFT_RING_MP=15,
    LEFT_MIDDLE_PP=16,
    LEFT_MIDDLE_MP=17,
    LEFT_INDEX_PP=18,
    LEFT_INDEX_MP=19,
    LEFT_THUMB_PP=20,
    LEFT_THUMB_MC=21,
    LEFT_HAND_C_MC=22,
    LEFT_PINKY_MC=23,
};
// Offset from a right-hand base value to its left-hand counterpart.
constexpr int kEskinBaseHandStride = 12;
struct EskinCoordinate {
    ESKIN_BASE_COORDINATE base;
    float x;
    float y;
    float z;
};

enum class TACTILE_ERROR_CODE {
    NO_ERROR = 0,
    HOST_IP_INVALID = 1,
    HOST_PORT_OCCUPIED = 2,
    HOST_LISTEN_FAULT = 3,
    TACCTILE_SDK_ALREADY_RUN = 4,
    TCP_CONNECT_FAILED = 6,
    ERROR_CODE_RESERVED = 5,
    TCP_SYSTEM_ERROR = 7,
    TCP_RECV_ERROR = 8,
    TCP_SEND_ERROR = 9,
    TCP_MSG_PARSE_FALSE = 10,
    CMD_EXEC_ERROR = 11,
    BOARD_CFG_KEY_NOT_EXIST = 12,
    BOARD_IP_NOT_CONFIGURED = 13,
    BOARD_CHANNEL_CONFLICT = 14,
    UNKNOWN_ERROR = 100,
};

}
}
