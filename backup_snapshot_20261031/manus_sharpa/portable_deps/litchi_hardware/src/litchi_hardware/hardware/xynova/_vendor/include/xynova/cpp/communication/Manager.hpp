/**
 * @file Manager.hpp
 * @brief C++ wrapper for Xynova Communication Manager.
 *
 * Provides RAII resource management and type-safe C++ interfaces
 * for the Xynova communication manager C API.
 *
 * The module supports:
 * - Communication manager initialization
 * - Master runtime control
 * - Device handle management
 * - Firmware upgrade
 *
 * @version 1.0.0
 * @date 2025-10-29
 * @author Xynova
 *
 * @see ::XynovaCommunicationManagerConfig
 * @see ::XynovaCommunicationManager_init
 */

#ifndef XYNOVA_CPP_COMMUNICATION_MANAGER_HPP
#define XYNOVA_CPP_COMMUNICATION_MANAGER_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xynova/c/comm_api.h"
#include "xynova/cpp/communication/types.h"

namespace xynova {
namespace communication {

/**
 * @defgroup xynova_cpp_communication C++ Communication Module
 * @brief High-level communication manager and device wrappers.
 * @{
 */

/**
 * @brief RAII wrapper for C device handle.
 *
 * Automatically releases the underlying C API handle
 * when the object is destroyed.
 */
class DeviceHandle {
public:
    explicit DeviceHandle(XynovaDeviceHandle *handle = nullptr)
        : i_handle(handle) {}

    ~DeviceHandle()
    {
        if (i_handle)
            XynovaCommunicationManager_freeDevice(i_handle);
    }

    DeviceHandle(const DeviceHandle &) = delete;
    DeviceHandle &operator=(const DeviceHandle &) = delete;

    DeviceHandle(DeviceHandle &&other) noexcept
        : i_handle(other.i_handle)
    {
        other.i_handle = nullptr;
    }

    DeviceHandle &operator=(DeviceHandle &&other) noexcept
    {
        if (this != &other)
        {
            if (i_handle)
                XynovaCommunicationManager_freeDevice(i_handle);

            i_handle = other.i_handle;
            other.i_handle = nullptr;
        }

        return *this;
    }

    [[nodiscard]] XynovaDeviceHandle *get() const noexcept
    {
        return i_handle;
    }

    explicit operator bool() const noexcept
    {
        return i_handle != nullptr;
    }

private:
    XynovaDeviceHandle *i_handle{nullptr};
};

/**
 * @brief Communication manager singleton.
 *
 * Provides high-level C++ interfaces for communication lifecycle,
 * device discovery and firmware operations.
 */
class Manager {
public:
    /**
     * @brief Get singleton instance.
     *
     * @return Manager instance.
     */
    static Manager &instance()
    {
        static Manager instance;
        return instance;
    }

    Manager(const Manager &) = delete;
    Manager &operator=(const Manager &) = delete;
    /**
     * @brief Initialize communication manager.
     *
     * Converts C++ configuration objects into C-compatible structures
     * and initializes communication masters and devices.
     *
     * @param config C++ manager configuration.
     *
     * @return ::XynovaErrorCode.
     *
     * @note Temporary storage is maintained during initialization to
     *       guarantee C API pointer validity.
     */
    XynovaErrorCode init(const ManagerConfig &config)
    {
        std::vector<XynovaCommunicationMasterConfig> c_masters;
        std::vector<XynovaCommunicationDeviceConfig> c_devices;
        std::vector<std::vector<XynovaCommunicationSlaveConfig>> c_slave_arrays;
        std::vector<std::vector<XynovaCommunicationSchedulingCommand>> c_command_arrays;

        c_masters.reserve(config.masters.size());
        c_devices.reserve(config.devices.size());
        c_slave_arrays.reserve(config.devices.size());
        c_command_arrays.reserve(config.devices.size());

        for (const auto &master : config.masters)
        {
            XynovaCommunicationMasterConfig c_master{};

            c_master.name = master.name.c_str();
            c_master.period_us = master.period_us;
            c_master.cpu_index = master.cpu_index;

            c_master.comm.type = static_cast<XynovaCommunicationCommType>(master.comm.type);
            c_master.comm.sub_type = master.comm.sub_type.c_str();
            c_master.comm.logger_name = master.comm.logger_name.c_str();

            switch (master.comm.type)
            {
            case CommType::Can:
                c_master.comm.config.can.iface = master.comm.can.iface.c_str();
                c_master.comm.config.can.manager_can = master.comm.can.manager_can;
                c_master.comm.config.can.bitrate = master.comm.can.bitrate;
                c_master.comm.config.can.dbitrate = master.comm.can.dbitrate;
                c_master.comm.config.can.sample_point = master.comm.can.sample_point;
                c_master.comm.config.can.dsample_point = master.comm.can.dsample_point;
                break;

            default:
                return XYNOVA_ERROR_CODE_COMM_INVALID_CONFIG;
            }

            c_masters.push_back(c_master);
        }

        for (const auto &device : config.devices)
        {
            XynovaCommunicationDeviceConfig c_device{};

            c_device.type = device.type.c_str();
            c_device.name = device.name.c_str();
            c_device.master_name = device.master_name.c_str();

            std::vector<XynovaCommunicationSlaveConfig> slaves;
            slaves.reserve(device.slaves.size());

            for (const auto &slave : device.slaves)
            {
                XynovaCommunicationSlaveConfig c_slave{};

                c_slave.type = slave.type.c_str();
                c_slave.id = slave.id;
                c_slave.logger_name = slave.logger_name.c_str();

                c_slave.general_time_setting.user_max_timeout_us        = slave.general_time_setting.user_max_timeout_us;
                c_slave.general_time_setting.user_poll_interval_us      = slave.general_time_setting.user_poll_interval_us;
                c_slave.general_time_setting.execute_timeout_us         = slave.general_time_setting.execute_timeout_us;
                c_slave.general_time_setting.execute_max_retry          = slave.general_time_setting.execute_max_retry;
                c_slave.general_time_setting.execute_poll_interval_us   = slave.general_time_setting.execute_poll_interval_us;

                c_slave.fast_time_setting.user_max_timeout_us           = slave.fast_time_setting.user_max_timeout_us;
                c_slave.fast_time_setting.user_poll_interval_us         = slave.fast_time_setting.user_poll_interval_us;
                c_slave.fast_time_setting.execute_timeout_us            = slave.fast_time_setting.execute_timeout_us;
                c_slave.fast_time_setting.execute_max_retry             = slave.fast_time_setting.execute_max_retry;
                c_slave.fast_time_setting.execute_poll_interval_us      = slave.fast_time_setting.execute_poll_interval_us;

                slaves.push_back(c_slave);
            }

            c_slave_arrays.push_back(std::move(slaves));

            c_device.slaves = c_slave_arrays.back().data();
            c_device.slave_count = c_slave_arrays.back().size();

            std::vector<XynovaCommunicationSchedulingCommand> commands;
            commands.reserve(device.commands.size());

            for (const auto &command : device.commands)
            {
                XynovaCommunicationSchedulingCommand c_command{};

                c_command.command = command.command.c_str();

                c_command.response_time_setting.user_max_timeout_us         = command.response_time_setting.user_max_timeout_us;
                c_command.response_time_setting.user_poll_interval_us       = command.response_time_setting.user_poll_interval_us;
                c_command.response_time_setting.execute_timeout_us          = command.response_time_setting.execute_timeout_us;
                c_command.response_time_setting.execute_max_retry           = command.response_time_setting.execute_max_retry;
                c_command.response_time_setting.execute_poll_interval_us    = command.response_time_setting.execute_poll_interval_us;

                commands.push_back(c_command);
            }

            c_command_arrays.push_back(std::move(commands));

            c_device.commands = c_command_arrays.back().data();
            c_device.command_count = c_command_arrays.back().size();

            c_devices.push_back(c_device);
        }

        XynovaCommunicationManagerConfig c_manager{};

        c_manager.masters = c_masters.data();
        c_manager.master_count = c_masters.size();

        c_manager.devices = c_devices.data();
        c_manager.device_count = c_devices.size();

        return XynovaCommunicationManager_init(&c_manager);
    }
    /**
     * @brief Start communication loop.
     *
     * @param master_name Master name.
     *
     * @return ::XynovaErrorCode.
     */
    XynovaErrorCode run(const std::string &master_name)
    {
        return XynovaCommunicationManager_run(master_name.c_str());
    }

    /**
     * @brief Stop communication loop.
     *
     * @param master_name Master name.
     *
     * @return ::XynovaErrorCode.
     */
    XynovaErrorCode stop(const std::string &master_name)
    {
        return XynovaCommunicationManager_stop(master_name.c_str());
    }

    /**
     * @brief Get device instance.
     *
     * Wraps the C API device handle into a C++ RAII object.
     *
     * @tparam T Device wrapper type.
     *
     * @param master_name Master name.
     * @param type Device type.
     * @param device_name Device name.
     *
     * @return Shared pointer to device instance.
     *         Returns nullptr when device is unavailable.
     */
    template <typename T>
    std::shared_ptr<T> getDevice(const std::string &master_name,
                                 const std::string &type,
                                 const std::string &device_name)
    {
        XynovaDeviceHandle *handle =
            XynovaCommunicationManager_getDevice(
                master_name.c_str(),
                type.c_str(),
                device_name.c_str());

        if (!handle)
            return nullptr;

        return std::make_shared<T>(DeviceHandle(handle));
    }

    /**
     * @brief Upgrade slave firmware.
     *
     * @param master_name Master name.
     * @param slave_id Slave identifier.
     * @param file Firmware file path.
     *
     * @return ::XynovaErrorCode.
     */
    XynovaErrorCode upgradeFirmware(const std::string &master_name,
                                    uint32_t slave_id,
                                    const std::string &file)
    {
        return XynovaCommunicationManager_upgradeFirmware(
            master_name.c_str(),
            slave_id,
            file.c_str());
    }

private:
    Manager() = default;
    ~Manager() = default;
};

} // namespace communication
} // namespace xynova

/**
 * @}
 */

#endif /* XYNOVA_CPP_COMMUNICATION_MANAGER_HPP */