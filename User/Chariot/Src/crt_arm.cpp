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
#include "drv_math.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace
{
constexpr uint32_t Control_Period_Ms = 2U;                             // 交互层每 2ms 调用一次

constexpr float Default_Normal_Joint_Omega = 1.0f;                     // 正常模式默认关节最大速度，rad/s
constexpr float Cali_Prepare_J2_Radian = 30.0f * DEG_TO_RAD;           // +30°
constexpr float Cali_Prepare_J3_Radian = -30.0f * DEG_TO_RAD;          // -30°
constexpr float Cali_Prepare_Omega = 0.5f;                             // 前置移位速度上限，rad/s
constexpr float Cali_Prepare_Tolerance_Radian = 5.0f * DEG_TO_RAD;     // 前置到位阈值 5°
constexpr float Cali_Return_Zero_Tolerance_Radian = 2.0f * DEG_TO_RAD; // 回零到位阈值 2°
constexpr float Cali_Search_Extra_Radian = 0.5f;                       // 搜索目标超出满行程余量，rad
constexpr uint32_t Cali_Return_Zero_Timeout_Ms = 10000U;               // 回零阶段超时保护，ms
constexpr uint32_t Cali_J4_Timeout_Ms = 13000U;                        // J4 满行程 5.24 rad / 0.5 rad/s + 余量
constexpr uint32_t Cali_J5_Timeout_Ms = 8000U;                         // J5 满行程 3.14 rad / 0.5 rad/s + 余量

constexpr Enum_RoboticArm_Joint Cali_Axis_Joint_Map[RoboticArm_Cali_Axis_NUM] = {
    RoboticArm_Joint_J4,
    RoboticArm_Joint_J5,
};
} // namespace

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_RoboticArm::Init()
{
    // Init 的量程参数顺序为 VMAX、TMAX、PMAX。FDCAN1 的反馈 ID 为 0x11～0x16。
    J1_Yaw_4340.Init(&hfdcan1, DM_Motor_ID_0x01, DM_Motor_Control_Method_POSITION_OMEGA, 10.0f, 9.0f, 12.5f);
    J2_Pitch_8009P.Init(&hfdcan1, DM_Motor_ID_0x02, DM_Motor_Control_Method_POSITION_OMEGA, 20.0f, 20.0f, 12.5f);
    J3_Pitch_4340.Init(&hfdcan1, DM_Motor_ID_0x03, DM_Motor_Control_Method_POSITION_OMEGA, 20.0f, 25.0f, 12.5f);
    J4_Roll_2325.Init(&hfdcan1, DM_Motor_ID_0x04, DM_Motor_Control_Method_POSITION_OMEGA, 200.0f, 10.0f, 310.0f, 25.00f);
    J5_Pitch_2325.Init(&hfdcan1, DM_Motor_ID_0x05, DM_Motor_Control_Method_POSITION_OMEGA, 200.0f, 10.0f, 158.0f, 25.00f);
    J6_Roll_3507.Init(&hfdcan1, DM_Motor_ID_0x06, DM_Motor_Control_Method_POSITION_OMEGA, 50.0f, 5.0f, 3.1415f);

    for (uint8_t joint = 0; joint < RoboticArm_Joint_NUM; ++joint)
    {
        Target_Joint_Omega[joint] = Default_Normal_Joint_Omega;
    }

    Joint_Cali[RoboticArm_Cali_Axis_J4] = Struct_Incremental_Joint_Cali{};
    Joint_Cali[RoboticArm_Cali_Axis_J4].Timeout_Ms = Cali_J4_Timeout_Ms;

    Joint_Cali[RoboticArm_Cali_Axis_J5] = Struct_Incremental_Joint_Cali{};
    Joint_Cali[RoboticArm_Cali_Axis_J5].Timeout_Ms = Cali_J5_Timeout_Ms;

    RoboticArm_Initialized = false;
    Calibration_Switch_Enable = false;
    First_Three_Hold_Active = false;
    Calibration_FSM.Init(RoboticArm_Cali_Status_NUM, RoboticArm_Cali_Status_WAIT_START);
}

void Class_RoboticArm::Set_RoboticArm_Control_Type(Enum_RoboticArm_Control_Type __RoboticArm_Control_Type)
{
    RoboticArm_Control_Type = __RoboticArm_Control_Type;
}

void Class_RoboticArm::Set_RoboticArm_Online_Status(Enum_RoboticArm_Online_Status __RoboticArm_Online_Status)
{
    RoboticArm_Online_Status = __RoboticArm_Online_Status;
}

void Class_RoboticArm::Set_RoboticArm_Initialized(bool __RoboticArm_Initialized)
{
    RoboticArm_Initialized = __RoboticArm_Initialized;
}

void Class_RoboticArm::Set_Calibration_Switch(bool __Enable)
{
    Calibration_Switch_Enable = __Enable;
}

void Class_RoboticArm::Set_Target_Joint_Radian(Enum_RoboticArm_Joint joint, float __Target_Radian)
{
    if (joint < RoboticArm_Joint_NUM)
    {
        Math_Constrain(&__Target_Radian, Joint_Min_Radian[joint], Joint_Max_Radian[joint]);
        Target_Joint_Radian[joint] = __Target_Radian;
    }
}

void Class_RoboticArm::Set_Target_Joint_Omega(Enum_RoboticArm_Joint joint, float __Target_Omega)
{
    if (joint < RoboticArm_Joint_NUM)
    {
        Target_Joint_Omega[joint] = __Target_Omega;
    }
}

void Class_RoboticArm::Request_Recalibration()
{
    for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
    {
        Joint_Cali[axis].Calibrated = false;
        Joint_Cali[axis].Timed_Out = false;
        Joint_Cali[axis].Returned_Zero = false;
        Joint_Cali[axis].Search_Started = false;
        Joint_Cali[axis].Zero_Offset = 0.0f;
        Joint_Cali[axis].Locked_Cnt = 0;
        Joint_Cali[axis].Elapsed_Ms = 0;
        Joint_Cali[axis].Peak_Torque = 0.0f;
    }
    RoboticArm_Initialized = false;
    Calibration_FSM.Set_Status(RoboticArm_Cali_Status_WAIT_START);
}

void Class_RoboticArm::Update_Incremental_Joint_Disconnect(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM])
{
    for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
    {
        const Enum_RoboticArm_Joint joint = Cali_Axis_Joint_Map[axis];
        Class_DM_Motor &motor = *joint_motors[joint];
        Struct_Incremental_Joint_Cali &cali = Joint_Cali[axis];

        cali.Now_Torque = motor.Get_Now_Torque();

        if (motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
        {
            // 增量式编码器通信断开即视为可能下电：立即失能并清除该轴校准状态，不自动重校。
            motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            cali.Calibrated = false;
            cali.Returned_Zero = false;
            cali.Search_Started = false;
            cali.Locked_Cnt = 0;
            cali.Elapsed_Ms = 0;
            RoboticArm_Initialized = false;
        }
    }
}

void Class_RoboticArm::Disable_Controlled_Joints(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM])
{
    for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J5; ++j)
    {
        joint_motors[j]->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
}

void Class_RoboticArm::Hold_First_Three_Joints(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM], float max_joint_omega)
{
    for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J3; ++j)
    {
        Class_DM_Motor &motor = *joint_motors[j];
        if (motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
        {
            motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            continue;
        }

        if (motor.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
        {
            motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        }

        Math_Constrain(&Target_Joint_Radian[j], Joint_Min_Radian[j], Joint_Max_Radian[j]);
        motor.Set_Target_Omega(max_joint_omega / fabsf(Motor_To_Joint_Scale[j]));
        motor.Set_Target_Angle(Target_Joint_Radian[j] / Motor_To_Joint_Scale[j]);
        motor.TIM_Process_PeriodElapsedCallback();
    }
}

void Class_RoboticArm::Command_Calibrated_Incremental_Joint(Enum_RoboticArm_Cali_Axis axis, Class_DM_Motor &motor, float max_joint_omega)
{
    const Enum_RoboticArm_Joint joint = Cali_Axis_Joint_Map[axis];
    Struct_Incremental_Joint_Cali &cali = Joint_Cali[axis];

    if (!cali.Calibrated || cali.Timed_Out || motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
    {
        motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        return;
    }

    Math_Constrain(&Target_Joint_Radian[joint], Joint_Min_Radian[joint], Joint_Max_Radian[joint]);
    motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    motor.Set_Target_Omega(max_joint_omega / fabsf(Motor_To_Joint_Scale[joint]));
    motor.Set_Target_Angle(Target_Joint_Radian[joint] / Motor_To_Joint_Scale[joint] + cali.Zero_Offset);
    motor.TIM_Process_PeriodElapsedCallback();
}

void Class_RoboticArm::Step_Incremental_Joint_Calibration(Enum_RoboticArm_Cali_Axis axis, Class_DM_Motor &motor)
{
    const Enum_RoboticArm_Joint joint = Cali_Axis_Joint_Map[axis];
    Struct_Incremental_Joint_Cali &cali = Joint_Cali[axis];

    if (cali.Timed_Out || motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
    {
        motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        return;
    }

    // 已完成堵转校准的轴直接回 0.0f，并检测是否到达 2° 以内。
    if (cali.Calibrated)
    {
        Target_Joint_Radian[joint] = 0.0f;
        Command_Calibrated_Incremental_Joint(axis, motor, cali.Cali_Omega);
        if (motor.Get_ErrorCode() == ERROR_Enabled &&
            fabsf(Now_Joint_Radian[joint]) <= Cali_Return_Zero_Tolerance_Radian)
        {
            cali.Returned_Zero = true;
        }
        return;
    }

    if (!cali.Search_Started)
    {
        const float full_stroke = Joint_Max_Radian[joint] - Joint_Min_Radian[joint];
        cali.Search_Target_Angle = motor.Get_Now_Angle() +
                                   (full_stroke + Cali_Search_Extra_Radian) / Motor_To_Joint_Scale[joint];
        cali.Locked_Cnt = 0;
        cali.Elapsed_Ms = 0;
        cali.Peak_Torque = 0.0f;
        cali.Search_Started = true;
    }

    cali.Elapsed_Ms += Control_Period_Ms;
    if (cali.Elapsed_Ms >= cali.Timeout_Ms)
    {
        // 单轴校准超时：仅失能超时轴。
        cali.Timed_Out = true;
        cali.Calibrated = false;
        cali.Search_Started = false;
        cali.Locked_Cnt = 0;
        motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        return;
    }

    motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    motor.Set_Target_Omega(cali.Cali_Omega / fabsf(Motor_To_Joint_Scale[joint]));
    motor.Set_Target_Angle(cali.Search_Target_Angle);
    motor.TIM_Process_PeriodElapsedCallback();

    const float abs_torque = fabsf(motor.Get_Now_Torque());
    const float abs_omega = fabsf(motor.Get_Now_Omega());
    if (abs_torque > cali.Peak_Torque)
    {
        cali.Peak_Torque = abs_torque;
    }

    if (motor.Get_ErrorCode() != ERROR_Enabled)
    {
        return;
    }

    if (abs_torque >= cali.Locked_Torque && abs_omega <= cali.Locked_Omega)
    {
        cali.Locked_Cnt++;
        if (cali.Locked_Cnt >= cali.Confirm_Count)
        {
            cali.Locked_Cnt = 0;
            cali.Zero_Offset = motor.Get_Now_Angle() - Joint_Max_Radian[joint] / Motor_To_Joint_Scale[joint];
            cali.Calibrated = true;
            cali.Returned_Zero = false;
            cali.Search_Started = false;
            Now_Joint_Radian[joint] = Joint_Max_Radian[joint];
            Target_Joint_Radian[joint] = 0.0f;
            Command_Calibrated_Incremental_Joint(axis, motor, cali.Cali_Omega);
        }
    }
    else
    {
        cali.Locked_Cnt = (cali.Locked_Cnt > cali.Decay_Step) ? (cali.Locked_Cnt - cali.Decay_Step) : 0U;
    }
}

void Class_RoboticArm::Update_Calibration_FSM(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM])
{
    const uint8_t status = Calibration_FSM.Get_Now_Status_Serial();
    Calibration_FSM.Status[status].Time += Control_Period_Ms;

    switch (static_cast<Enum_RoboticArm_Cali_Status>(status))
    {
        case RoboticArm_Cali_Status_WAIT_START:
        {
            // 等待 J1~J5 全部在线后，触发全臂上电首次校准。
            bool all_online = true;
            for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J5; ++j)
            {
                if (joint_motors[j]->Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
                {
                    all_online = false;
                    break;
                }
            }
            if (!all_online)
            {
                Disable_Controlled_Joints(joint_motors);
                break;
            }

            // 如果两轴均已校准并回中（例如切回 DISABLE 再进 NORMAL），直接进入 DONE。
            if (Joint_Cali[RoboticArm_Cali_Axis_J4].Calibrated &&
                Joint_Cali[RoboticArm_Cali_Axis_J4].Returned_Zero &&
                Joint_Cali[RoboticArm_Cali_Axis_J5].Calibrated &&
                Joint_Cali[RoboticArm_Cali_Axis_J5].Returned_Zero)
            {
                RoboticArm_Initialized = true;
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_DONE);
                break;
            }

            // 锁定 J1 当前角度，设定 J2(+30°) 和 J3(-30°) 的校准前置目标。
            Target_Joint_Radian[RoboticArm_Joint_J1] = Now_Joint_Radian[RoboticArm_Joint_J1];
            Target_Joint_Radian[RoboticArm_Joint_J2] = Cali_Prepare_J2_Radian;
            Target_Joint_Radian[RoboticArm_Joint_J3] = Cali_Prepare_J3_Radian;
            Calibration_FSM.Set_Status(RoboticArm_Cali_Status_PREPARE_J2_J3);
            break;
        }

        case RoboticArm_Cali_Status_PREPARE_J2_J3:
        {
            // 前置移位期间，J4/J5 保持失能；J1 锁当前角，J2/J3 以 0.5 rad/s 移动到 ±30°。
            J4_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            J5_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            Hold_First_Three_Joints(joint_motors, Cali_Prepare_Omega);

            const bool j2_ready = (J2_Pitch_8009P.Get_ErrorCode() == ERROR_Enabled) &&
                                  (fabsf(Now_Joint_Radian[RoboticArm_Joint_J2] - Cali_Prepare_J2_Radian) <= Cali_Prepare_Tolerance_Radian);
            const bool j3_ready = (J3_Pitch_4340.Get_ErrorCode() == ERROR_Enabled) &&
                                  (fabsf(Now_Joint_Radian[RoboticArm_Joint_J3] - Cali_Prepare_J3_Radian) <= Cali_Prepare_Tolerance_Radian);
            if (j2_ready && j3_ready)
            {
                for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
                {
                    if (!Joint_Cali[axis].Calibrated)
                    {
                        Joint_Cali[axis].Timed_Out = false;
                        Joint_Cali[axis].Search_Started = false;
                        Joint_Cali[axis].Locked_Cnt = 0;
                        Joint_Cali[axis].Elapsed_Ms = 0;
                        Joint_Cali[axis].Peak_Torque = 0.0f;
                    }
                }
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_CALIBRATING);
            }
            break;
        }

        case RoboticArm_Cali_Status_CALIBRATING:
        {
            Hold_First_Three_Joints(joint_motors, Cali_Prepare_Omega);

            Step_Incremental_Joint_Calibration(RoboticArm_Cali_Axis_J4, J4_Roll_2325);
            Step_Incremental_Joint_Calibration(RoboticArm_Cali_Axis_J5, J5_Pitch_2325);

            const bool j4_finished = Joint_Cali[RoboticArm_Cali_Axis_J4].Calibrated ||
                                     Joint_Cali[RoboticArm_Cali_Axis_J4].Timed_Out ||
                                     (J4_Roll_2325.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE);
            const bool j5_finished = Joint_Cali[RoboticArm_Cali_Axis_J5].Calibrated ||
                                     Joint_Cali[RoboticArm_Cali_Axis_J5].Timed_Out ||
                                     (J5_Pitch_2325.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE);
            if (j4_finished && j5_finished)
            {
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_RETURN_ZERO);
            }
            break;
        }

        case RoboticArm_Cali_Status_RETURN_ZERO:
        {
            Hold_First_Three_Joints(joint_motors, Cali_Prepare_Omega);

            bool all_calibrated_returned = true;
            for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
            {
                const Enum_RoboticArm_Joint joint = Cali_Axis_Joint_Map[axis];
                Class_DM_Motor &motor = *joint_motors[joint];
                Struct_Incremental_Joint_Cali &cali = Joint_Cali[axis];

                if (!cali.Calibrated || cali.Timed_Out || motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
                {
                    motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                    continue;
                }

                Target_Joint_Radian[joint] = 0.0f;
                Command_Calibrated_Incremental_Joint(static_cast<Enum_RoboticArm_Cali_Axis>(axis), motor, cali.Cali_Omega);

                if (motor.Get_ErrorCode() == ERROR_Enabled &&
                    fabsf(Now_Joint_Radian[joint]) <= Cali_Return_Zero_Tolerance_Radian)
                {
                    cali.Returned_Zero = true;
                }

                if (!cali.Returned_Zero)
                {
                    if (Calibration_FSM.Status[status].Time >= Cali_Return_Zero_Timeout_Ms)
                    {
                        cali.Timed_Out = true;
                        motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                    }
                    else
                    {
                        all_calibrated_returned = false;
                    }
                }
            }

            if (all_calibrated_returned)
            {
                RoboticArm_Initialized = Joint_Cali[RoboticArm_Cali_Axis_J4].Calibrated &&
                                         Joint_Cali[RoboticArm_Cali_Axis_J4].Returned_Zero &&
                                         Joint_Cali[RoboticArm_Cali_Axis_J5].Calibrated &&
                                         Joint_Cali[RoboticArm_Cali_Axis_J5].Returned_Zero;
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_DONE);
            }
            break;
        }

        case RoboticArm_Cali_Status_DONE:
        default:
            break;
    }
}

void Class_RoboticArm::Run_Normal_Control(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM])
{
    for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J3; ++j)
    {
        Class_DM_Motor &motor = *joint_motors[j];
        if (motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
        {
            motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            continue;
        }

        if (motor.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
        {
            Target_Joint_Radian[j] = Now_Joint_Radian[j];
            motor.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        }

        Math_Constrain(&Target_Joint_Radian[j], Joint_Min_Radian[j], Joint_Max_Radian[j]);
        const float max_omega = (Target_Joint_Omega[j] > 0.0f) ? Target_Joint_Omega[j] : Default_Normal_Joint_Omega;
        motor.Set_Target_Omega(max_omega / fabsf(Motor_To_Joint_Scale[j]));
        motor.Set_Target_Angle(Target_Joint_Radian[j] / Motor_To_Joint_Scale[j]);
        motor.TIM_Process_PeriodElapsedCallback();
    }

    // J4/J5 暂不绑定摇杆：已校准且在线则锁在目标角（默认 0.0f），掉线或超时轴保持失能。
    for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
    {
        const Enum_RoboticArm_Joint joint = Cali_Axis_Joint_Map[axis];
        const float max_omega = (Target_Joint_Omega[joint] > 0.0f) ? Target_Joint_Omega[joint] : Default_Normal_Joint_Omega;
        Command_Calibrated_Incremental_Joint(static_cast<Enum_RoboticArm_Cali_Axis>(axis), *joint_motors[joint], max_omega);
    }
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

    // 先监测增量式关节是否通信断开（断开则失能并清校准标志，不自动重校）。
    Update_Incremental_Joint_Disconnect(joint_motors);

    for (uint8_t joint = 0; joint < RoboticArm_Joint_NUM; ++joint)
    {
        float motor_angle = joint_motors[joint]->Get_Now_Angle();
        if (joint == RoboticArm_Joint_J4 && Joint_Cali[RoboticArm_Cali_Axis_J4].Calibrated)
        {
            motor_angle -= Joint_Cali[RoboticArm_Cali_Axis_J4].Zero_Offset;
        }
        else if (joint == RoboticArm_Joint_J5 && Joint_Cali[RoboticArm_Cali_Axis_J5].Calibrated)
        {
            motor_angle -= Joint_Cali[RoboticArm_Cali_Axis_J5].Zero_Offset;
        }
        Now_Joint_Radian[joint] = Motor_To_Joint_Scale[joint] * motor_angle;
        Now_Joint_Omega[joint] = Motor_To_Joint_Scale[joint] * joint_motors[joint]->Get_Now_Omega();
    }

    if (RoboticArm_Control_Type != RoboticArm_Control_Type_NORMAL)
    {
        First_Three_Hold_Active = false;
        Disable_Controlled_Joints(joint_motors);
        // 校准尚未完成时切回 DISABLE，中止本次校准并回到等待状态（保留已校完且未断电轴的偏移）。
        if (Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_DONE &&
            Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_WAIT_START)
        {
            for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
            {
                if (!Joint_Cali[axis].Calibrated)
                {
                    Joint_Cali[axis].Timed_Out = false;
                    Joint_Cali[axis].Search_Started = false;
                    Joint_Cali[axis].Locked_Cnt = 0;
                    Joint_Cali[axis].Elapsed_Ms = 0;
                }
            }
            Calibration_FSM.Set_Status(RoboticArm_Cali_Status_WAIT_START);
        }
        return;
    }

    if (!Calibration_Switch_Enable)
    {
        // 关掉 SWA：校准的两轴（J4/J5）失能；若前三轴已使能则保持当前位置并将目标角速度置 0。
        J4_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J5_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

        if (Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_DONE &&
            Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_WAIT_START)
        {
            for (uint8_t axis = 0; axis < RoboticArm_Cali_Axis_NUM; ++axis)
            {
                if (!Joint_Cali[axis].Calibrated)
                {
                    Joint_Cali[axis].Timed_Out = false;
                    Joint_Cali[axis].Search_Started = false;
                    Joint_Cali[axis].Locked_Cnt = 0;
                    Joint_Cali[axis].Elapsed_Ms = 0;
                }
            }
            Calibration_FSM.Set_Status(RoboticArm_Cali_Status_WAIT_START);
        }

        if (!First_Three_Hold_Active)
        {
            for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J3; ++j)
            {
                if (joint_motors[j]->Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_ENABLE)
                {
                    Target_Joint_Radian[j] = Now_Joint_Radian[j];
                    Target_Joint_Omega[j] = 0.0f;
                    First_Three_Hold_Active = true;
                }
            }
        }

        if (First_Three_Hold_Active)
        {
            for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J3; ++j)
            {
                Class_DM_Motor &motor = *joint_motors[j];
                if (motor.Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
                {
                    motor.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                    continue;
                }
                if (motor.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_ENABLE)
                {
                    Target_Joint_Omega[j] = 0.0f;
                    motor.Set_Target_Angle(Target_Joint_Radian[j] / Motor_To_Joint_Scale[j]);
                    motor.Set_Target_Omega(0.0f);
                    motor.TIM_Process_PeriodElapsedCallback();
                }
            }
        }
        else
        {
            Disable_Controlled_Joints(joint_motors);
        }
        return;
    }

    First_Three_Hold_Active = false;

    if (Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_DONE)
    {
        Update_Calibration_FSM(joint_motors);
        return;
    }

    Run_Normal_Control(joint_motors);
}

void Class_Joint_DM_J4310::TIM_Process_PeriodElapsedCallback()
{
    Pack_Control_Frame(Target_Angle, Target_Omega, Target_Torque, MIT_K_P, MIT_K_D);
}

void Class_Joint_DM_S2325::TIM_Process_PeriodElapsedCallback()
{
    Pack_Control_Frame(Target_Angle, Target_Omega, Target_Torque, MIT_K_P, MIT_K_D);
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
