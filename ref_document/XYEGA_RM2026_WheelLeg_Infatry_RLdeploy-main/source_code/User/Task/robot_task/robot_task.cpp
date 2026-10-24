//
// Created by An on 2025/10/24.
//
#include "robot_task.h"
#include "cmsis_os.h"
#include "logger.h"
#include "remote.h"
#include "remote_DT7.h"
#include "remote_FS_I6X.h"
#include "robot_manager.h"
#include "user_configs.h"
#include "chassis.h"
#include "iwdg.h"

[[noreturn]] void RobotTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(2);//周期500hz

    //初始化机器人管理器
    RobotManager::init(configs::robot_manager_config);
    Chassis::init();
    MX_IWDG1_Init();
    TickType_t last_wake_time = xTaskGetTickCount();
    auto &hub = RobotManager::getDataHubConst();
    for (;;)
    {
        if (!hub.gimbal.board_need_reset)
        {
            HAL_IWDG_Refresh(&hiwdg1);
        }
        RobotManager::update();
        Chassis::controlLoop();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
