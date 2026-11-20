/**
 * @file crt_arm.cpp
 * @author hsl
 * @brief 机械臂
 * @version V1.1
 * @date 2027-11-17 27赛季定稿
 *
 * @copyright ZLLC 2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_arm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_RoboticArm::Init()
{
    // Init 的量程参数顺序为 VMAX、TMAX、PMAX。FDCAN1 的反馈 ID 为 0x11～0x16。
    J1_Yaw_4340.Init(&hfdcan1, DM_Motor_ID_0x01, DM_Motor_Control_Method_POSITION_OMEGA, 10.0f, 9.0f, 12.5f);
    J2_Pitch_8009P.Init(&hfdcan1, DM_Motor_ID_0x02, DM_Motor_Control_Method_POSITION_OMEGA, 20.0f, 20.0f, 12.5f);
    J3_Pitch_4340.Init(&hfdcan1, DM_Motor_ID_0x03, DM_Motor_Control_Method_POSITION_OMEGA, 20.0f, 25.0f, 12.5f);
    J4_Roll_2325.Init(&hfdcan1, DM_Motor_ID_0x04, DM_Motor_Control_Method_POSITION_OMEGA, 200.0f, 10.0f, 310.0f, 25.0f);
    J5_Pitch_2325.Init(&hfdcan1, DM_Motor_ID_0x05, DM_Motor_Control_Method_POSITION_OMEGA, 200.0f, 10.0f, 158.0f, 19.2032f);
    J6_Roll_3507.Init(&hfdcan1, DM_Motor_ID_0x06, DM_Motor_Control_Method_POSITION_OMEGA, 50.0f, 5.0f, 3.1415f);
}

void Class_RoboticArm::TIM_Calculate_PeriodElapsedCallback()
{
    Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM] = {
        &J1_Yaw_4340,
        &J2_Pitch_8009P,
        &J3_Pitch_4340,
        &J4_Roll_2325,
        &J5_Pitch_2325,
        &J6_Roll_3507,
    };

    for (uint8_t joint = 0; joint < RoboticArm_Joint_NUM; ++joint)
    {
        Now_Joint_Radian[joint] = Motor_To_Joint_Scale[joint] * joint_motors[joint]->Get_Now_Angle();
        Now_Joint_Omega[joint] = Motor_To_Joint_Scale[joint] * joint_motors[joint]->Get_Now_Omega();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
