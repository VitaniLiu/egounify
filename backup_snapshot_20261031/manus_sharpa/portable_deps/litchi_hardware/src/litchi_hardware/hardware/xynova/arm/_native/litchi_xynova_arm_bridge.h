#ifndef LITCHI_XYNOVA_ARM_BRIDGE_H
#define LITCHI_XYNOVA_ARM_BRIDGE_H

#include <stdint.h>

#if defined(__GNUC__)
#define LITCHI_XYNOVA_ARM_API __attribute__((visibility("default")))
#else
#define LITCHI_XYNOVA_ARM_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define LITCHI_XYNOVA_ARM_BRIDGE_ABI_VERSION 1U
#define LITCHI_XYNOVA_ARM_MAX_JOINTS 7U

typedef struct LitchiXynovaArmState
{
    uint32_t joint_count;
    double position[LITCHI_XYNOVA_ARM_MAX_JOINTS];
    double velocity[LITCHI_XYNOVA_ARM_MAX_JOINTS];
    double effort[LITCHI_XYNOVA_ARM_MAX_JOINTS];
} LitchiXynovaArmState;

typedef struct LitchiXynovaArmInfo
{
    uint32_t joint_count;
    char id[128];
    char mac[64];
    char controller_version[64];
    char model[128];
    char sdk_version[64];
} LitchiXynovaArmInfo;

typedef struct LitchiXynovaArmHandle LitchiXynovaArmHandle;

LITCHI_XYNOVA_ARM_API uint32_t litchi_xynova_arm_bridge_abi_version(void);
LITCHI_XYNOVA_ARM_API uint32_t litchi_xynova_arm_state_size(void);
LITCHI_XYNOVA_ARM_API uint32_t litchi_xynova_arm_info_size(void);

/*
 * model codes:
 *   0 XynovaRobot (6 DoF collaborative)
 *   1 XynovaCr5Robot (5 DoF collaborative)
 *   2 XynovaErRobot (7 DoF collaborative)
 *   3 XynovaArRobot (7 DoF collaborative)
 *   4 StandardRobot (6 DoF industrial)
 *   5 PCB3Robot (3 DoF industrial)
 *   6 PCB4Robot (4 DoF industrial)
 */
LITCHI_XYNOVA_ARM_API LitchiXynovaArmHandle* litchi_xynova_arm_connect(
    int32_t model,
    const char* remote_ip,
    const char* local_ip);

LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_disconnect(
    LitchiXynovaArmHandle* handle);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_destroy(
    LitchiXynovaArmHandle* handle);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_get_info(
    LitchiXynovaArmHandle* handle,
    LitchiXynovaArmInfo* info);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_read_state(
    LitchiXynovaArmHandle* handle,
    LitchiXynovaArmState* state);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_read_tcp_pose(
    LitchiXynovaArmHandle* handle,
    double pose_xyzrpy[6]);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_read_wrench(
    LitchiXynovaArmHandle* handle,
    double wrench_xyzabc[6]);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_set_power(
    LitchiXynovaArmHandle* handle,
    int32_t enabled);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_command_joint_position(
    LitchiXynovaArmHandle* handle,
    const double* position,
    uint32_t joint_count,
    double speed,
    double zone);
LITCHI_XYNOVA_ARM_API int32_t litchi_xynova_arm_stop(
    LitchiXynovaArmHandle* handle);
LITCHI_XYNOVA_ARM_API const char* litchi_xynova_arm_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
