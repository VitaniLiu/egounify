/**
 * @file upgrade.h
 * @brief 控制器升级与备份
 */

#ifndef XYNOVASDK_INCLUDE_XYNOVA_UPGRADE_H_
#define XYNOVASDK_INCLUDE_XYNOVA_UPGRADE_H_

#include "base.h"

#if defined(_MSC_VER)
#  if defined(XYNOVAUPGRADE_DLL_BUILD)
#    define XYNOVAUPGRADE_API __declspec(dllexport)
#  elif defined(XYNOVAUPGRADE_DLL)
#    define XYNOVAUPGRADE_API __declspec(dllimport)
#  else
#    define XYNOVAUPGRADE_API
#  endif
#else
#  define XYNOVAUPGRADE_API __attribute__((visibility("default")))
#endif

namespace xynova {

 /**
  * @brief 控制器备份项
  */
 enum class BackupItem {
    customConfig,                 ///< 自定义配置
    robotConfig,                  ///< 机器人配置
    robotConfigWithoutBodyParams, ///< 机器人配置, 不含本体参数
    controllerLog,                ///< 控制器日志
    systemLog,                    ///< 系统日志
    rlProgram,                    ///< RL程序
    backupRlProgram,              ///< 备份RL工程
    sdkRecordPath                 ///< 通过SDK录制的拖动示教路径
 };

 /**
  * @brief 升级/备份控制器
  */
 class XYNOVAUPGRADE_API BaseUpgrade {
   public:

   /**
    * @brief 构造函数, 不连接
    */
   BaseUpgrade() noexcept;

   /**
    * @brief 构造函数，并连接机器人升级程序
    * @param remote_ip 机器人地址
    * @throw NetworkException 1)无法连接升级程序; 2)连接被拒绝, 可能有其它客户端或RobotAssist连接, 不允许重复连接
    */
   explicit BaseUpgrade(const std::string &remote_ip);

   /**
    * @brief 析构
    */
   ~BaseUpgrade() noexcept;

   /**
    * @brief 连接到机器人升级程序
    * @param ec 错误码, 无法连接到升级程序，或重复链接被拒绝
    */
   void connect(error_code &ec) noexcept;

   /**
    * @brief 连接到机器人升级程序
    * @param remote_ip 机器人地址
    * @throw NetworkException 1)无法连接升级程序; 2)连接被拒绝, 可能有其它客户端或RobotAssist连接, 不允许重复连接
    */
   void connect(const std::string &remote_ip);

   /**
    * @brief 升级控制器各类文件, 包括: 控制器固件版本rpa文件, 控制器备份rpa文件。
    * 阻塞等待升级完成或出错，或等待超时，超时时间1分钟。
    * @param[in] local_file_path 文件本地路径
    * @param[out] ec 错误码 本地文件打开失败/网络连接问题/升级文件格式错误/升级失败/有升级备份操作未完成
    */
   void upgrade(const std::string &local_file_path, error_code &ec) noexcept;

   /**
    * @brief 导出控制器备份，阻塞等待升级完成或出错，或等待超时，超时时间2分钟。
    * @param[in] local_file_path 导出文件到该本地文件
    * @param[in] items 备份项
    * @param[out] ec 错误码，网络连接问题/文件传输过程储存/有升级备份操作未完成/保存失败
    */
   void exportBackup(const std::string &local_file_path, const std::set<BackupItem> &items, error_code &ec) noexcept;

   XYNOVASDK_DECLARE_IMPL
 };
}
#endif //XYNOVASDK_INCLUDE_XYNOVA_UPGRADE_H_
