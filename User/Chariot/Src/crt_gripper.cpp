/**
 * @file crt_gripper.cpp
 * @author hsl by suzumiaoya
 * @brief 旋转夹爪机构类实现
 * @version V1.1
 * @date 2026-11-22
 *
 * @copyright ZLLC 2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gripper.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static constexpr uint32_t Gripper_Control_Period_Ms = 2U;        // 交互层每 2ms 调用一次
static constexpr float Gripper_Control_Period_S = 0.002f;        // M2006 速度环离散周期，s

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_Gripper_DM_Motor::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
                                  float __Omega_Max, float __Torque_Max, float __Angle_Max)
{
    Class_DM_Motor::Init(hcan, __CAN_ID, __Control_Method, __Omega_Max, __Torque_Max, __Angle_Max);
    Reset_Multi_Turn_Radian();
}

void Class_Gripper_DM_Motor::Data_Process(uint8_t *Rx_Data)
{
    Class_DM_Motor::Data_Process(Rx_Data);

    const float feedback_radian = Data.Now_Angle;
    if (!Radian_Initialized)
    {
        // 首帧可能已包含电机内部记录的角度，直接作为多圈初值
        Pre_Feedback_Radian = feedback_radian;
        Now_Multi_Turn_Radian = feedback_radian;
        Radian_Initialized = true;
    }
    else
    {
        float delta_radian = feedback_radian - Pre_Feedback_Radian;
        // 相邻反馈的实际转角应小于 Angle_Max；跨越 [-Angle_Max, +Angle_Max] 边界时按 2*Angle_Max 解卷
        if (delta_radian > Angle_Max)
        {
            delta_radian -= 2.0f * Angle_Max;
        }
        else if (delta_radian < -Angle_Max)
        {
            delta_radian += 2.0f * Angle_Max;
        }
        Now_Multi_Turn_Radian += delta_radian;
        Pre_Feedback_Radian = feedback_radian;
    }
}

void Class_Gripper_DM_Motor::TIM_Alive_PeriodElapsedCallback()
{
    Class_DM_Motor::TIM_Alive_PeriodElapsedCallback();
    if (Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
    {
        // DM3507 自身掉线视为可能重启：清零多圈初始化标志，避免重连后按旧圈数跳变
        Reset_Multi_Turn_Radian();
    }
}

void Class_Gripper_DM_Motor::TIM_Process_PeriodElapsedCallback()
{
    Pack_Control_Frame(Target_Angle, Target_Omega, Target_Torque, MIT_K_P, MIT_K_D);
}

void Class_Gripper_DM_Motor::Reset_Multi_Turn_Radian()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Radian_Initialized = false;
    Pre_Feedback_Radian = 0.0f;
    Now_Multi_Turn_Radian = 0.0f;
    __set_PRIMASK(primask);
}

void Class_Gripper_DJI_Motor::Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __Control_Method,
                                   float __Gearbox_Rate, float __Output_Max)
{
    // Torque_Max 与基类 Output_Max 均为 10000，使目标值与 CAN 电流映射值一一对应
    Class_DJI_Motor_C610::Init(hcan, __CAN_ID, __Control_Method, __Gearbox_Rate, 10000.0f);
    Clamp_Output_Max = __Output_Max;
}

void Class_Gripper_DJI_Motor::Set_Current_Control(float __Target_Current)
{
    Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_CURRENT;
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
    Target_Current = __Target_Current;
    Set_Target_Torque(__Target_Current);
    Set_Target_Omega_Radian(0.0f);
    Target_Clamp_Current = 0.0f;
}

void Class_Gripper_DJI_Motor::Set_Omega_Control(float __Target_Omega_Radian)
{
    Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_OMEGA;
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    Set_Target_Omega_Radian(__Target_Omega_Radian);
    Target_Clamp_Current = 0.0f;
}

void Class_Gripper_DJI_Motor::Set_Clamp_Follow_Control(float __Target_Clamp_Current, float __Target_Omega_Radian)
{
    Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_CLAMP_FOLLOW;
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
    Target_Clamp_Current = __Target_Clamp_Current;
    Set_Target_Omega_Radian(__Target_Omega_Radian);
}

void Class_Gripper_DJI_Motor::Reset_Omega_PID()
{
    PID_Omega.Set_Integral_Error(0.0f);
}

void Class_Gripper_DJI_Motor::TIM_Control_PeriodElapsedCallback()
{
    switch (Gripper_DJI_Control_Method)
    {
        case Gripper_DJI_Control_Method_CURRENT:
        {
            Out = Target_Current;
            Math_Constrain(&Out, -Clamp_Output_Max, Clamp_Output_Max);
            break;
        }

        case Gripper_DJI_Control_Method_OMEGA:
        case Gripper_DJI_Control_Method_CLAMP_FOLLOW:
        {
            PID_Omega.Set_Target(Get_Target_Omega_Radian());
            PID_Omega.Set_Now(Get_Now_Omega_Radian());
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();
            Out = PID_Omega.Get_Out();
            if (Gripper_DJI_Control_Method == Gripper_DJI_Control_Method_CLAMP_FOLLOW)
            {
                Out = Target_Clamp_Current + PID_Omega.Get_Out();
                Math_Constrain(&Out, -Clamp_Output_Max, Clamp_Output_Max);
                Set_Target_Torque(Out);
            }
            break;
        }

        default:
        {
            Out = 0.0f;
            break;
        }
    }

    // 基类 Output 仅将 Out 写入 CAN1_0x200_Tx_Data 对应槽位
    Output();
}

void Class_Gripper::Init()
{
    // DM3507 沿用现有已验证的 FDCAN1 配置（发送 0x1F6，反馈 0x16，VMAX=50，TMAX=5，PMAX=3.1415）
    DM_Motor_Rotary.Init(&hfdcan1, DM_Motor_ID_0xA6, DM_Motor_Control_Method_POSITION_OMEGA, 50.0f, 5.0f, 3.1415f);
    // M2006 与所有关节共线于 FDCAN1（发送 0x200 槽位 4，反馈 0x204，减速比 36）
    DJI_Motor_Clamp.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_TORQUE, 36.0f, Control_Const.Output_Max);
    DJI_Motor_Clamp.Set_Out(0.0f);

    DJI_Motor_Clamp.PID_Omega.Init(Control_Const.Omega_Kp, 0.0f, 0.0f, 0.0f, 0.0f, Control_Const.Output_Max,
                                   0.0f, 0.0f, 0.0f, Gripper_Control_Period_S);

    Target_Clamp_Current = Control_Const.Clamp_Current_Max;
    Clamp_FSM.Init(Gripper_Clamp_Status_NUM, Gripper_Clamp_Status_DISABLED);
}

void Class_Gripper::Set_Target_Clamp_Current(float __Target_Clamp_Current)
{
    Math_Constrain(&__Target_Clamp_Current, 0.0f, Control_Const.Clamp_Current_Max);
    Target_Clamp_Current = __Target_Clamp_Current;
}

void Class_Gripper::Calculate_Kinematics()
{
    // 电机轴数据先经外啮合传动换算为旋转齿轮数据，再参与机构解算
    const float now_dm_radian = DM_Motor_Rotary.Get_Now_Multi_Turn_Radian();
    const float dm_to_roll_scale = Mechanics_Const.Direction_DM2ROLL * Mechanics_Const.Gear_Ratio_DM2ROLL;
    Now_Roll_Total_Radian = dm_to_roll_scale * now_dm_radian;
    Now_Roll_Gear_Omega_Radian = dm_to_roll_scale * DM_Motor_Rotary.Get_Now_Omega();

    // 正运动学解算（DJI 轴先经 Direction_DJI2CLAMP 映射到夹持机构坐标系）
    const float now_dji_gear_omega = Mechanics_Const.Direction_DJI2CLAMP * DJI_Motor_Clamp.Get_Now_Omega_Radian();
    Gripper_Data.Now_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Now_Roll_Total_Radian);
    Gripper_Data.Now_Roll_Omega_Radian = Now_Roll_Gear_Omega_Radian;
    Gripper_Data.Now_Clamp_Omega_Radian = now_dji_gear_omega - Now_Roll_Gear_Omega_Radian;

    // 逆运动学：优弧目标先在旋转齿轮侧确定，再还原到 DM 电机轴
    const Enum_Gripper_Clamp_Status clamp_status = Get_Gripper_Clamp_Status();
    if (DM_Motor_Rotary.Get_Radian_Initialized() &&
        ((clamp_status == Gripper_Clamp_Status_OPENED) ||
         (clamp_status == Gripper_Clamp_Status_GRIPPED)))
    {
        Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian);
        const float delta_roll_radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian - Gripper_Data.Now_Roll_Radian);
        Target_DM_Radian = now_dm_radian + delta_roll_radian / dm_to_roll_scale;
        Target_DM_Omega = fabsf(Target_Roll_Omega_Radian) / fabsf(dm_to_roll_scale);
    }
}

void Class_Gripper::Step_Clamp_FSM(uint32_t period_ms)
{
    const uint8_t status_serial = Clamp_FSM.Get_Now_Status_Serial();
    Clamp_FSM.Status[status_serial].Time += period_ms;
    const uint32_t status_time_ms = Clamp_FSM.Status[status_serial].Time;
    const Enum_Gripper_Clamp_Status clamp_status = static_cast<Enum_Gripper_Clamp_Status>(status_serial);

    const bool dm_online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    const bool dji_online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);
    if (!dm_online || !dji_online)
    {
        Contact_Confirm_Time_Ms = 0U;
        if (!dm_online)
        {
            Now_Roll_Total_Radian = 0.0f;
        }
        if (clamp_status != Gripper_Clamp_Status_DISABLED)
        {
            Clamp_FSM.Set_Status(Gripper_Clamp_Status_DISABLED);
        }
        return;
    }

    if (Gripper_Control_Type == Gripper_Control_Type_DISABLE)
    {
        Contact_Confirm_Time_Ms = 0U;
        DJI_Motor_Clamp.Reset_Omega_PID();
        if (clamp_status != Gripper_Clamp_Status_DISABLED)
        {
            Clamp_FSM.Set_Status(Gripper_Clamp_Status_DISABLED);
        }
        return;
    }

    if (!DM_Motor_Rotary.Get_Radian_Initialized())
    {
        return;
    }

    switch (clamp_status)
    {
        case Gripper_Clamp_Status_DISABLED:
        {
            Start_Clamp_Motion(Gripper_Clamp_Command);
            break;
        }

        case Gripper_Clamp_Status_OPENING:
        {
            if (Gripper_Clamp_Command == Gripper_Clamp_Command_CLOSE)
            {
                Start_Clamp_Motion(Gripper_Clamp_Command_CLOSE);
                break;
            }

            if (status_time_ms >= static_cast<uint32_t>(FSM_Const.Motion_Timeout_Ms + FSM_Const.Motion_Stop_Time_Ms))
            {
                Timeout_Clamp_Command = Gripper_Clamp_Command;
                Clamp_FSM.Set_Status(Gripper_Clamp_Status_TIMEOUT);
                break;
            }

            // 进入状态的前 Ignore_Time_Ms 不进行接触判定，待电流稳定后再检测堵转
            const float now_dji_gear_current = Mechanics_Const.Direction_DJI2CLAMP * DJI_Motor_Clamp.Get_Now_Torque();
            if ((status_time_ms >= FSM_Const.Ignore_Time_Ms) &&
                (status_time_ms < FSM_Const.Motion_Timeout_Ms) &&
                (fabsf(Gripper_Data.Now_Clamp_Omega_Radian) < FSM_Const.Stall_Omega) &&
                (fabsf(now_dji_gear_current - Control_Const.Spring_Current) >=
                 FSM_Const.Stall_Current_Ratio * Control_Const.Omega_Kp * fabsf(Control_Const.Open_Omega)))
            {
                Contact_Confirm_Time_Ms += static_cast<uint16_t>(period_ms);
            }
            else
            {
                Contact_Confirm_Time_Ms = 0U;
            }

            if (Contact_Confirm_Time_Ms >= FSM_Const.Confirm_Time_Ms)
            {
                Enter_Roll_Enabled_Status(Gripper_Clamp_Status_OPENED);
            }
            break;
        }

        case Gripper_Clamp_Status_OPENED:
        {
            if (Gripper_Clamp_Command == Gripper_Clamp_Command_CLOSE)
            {
                Start_Clamp_Motion(Gripper_Clamp_Command_CLOSE);
            }
            break;
        }

        case Gripper_Clamp_Status_CLOSING:
        {
            if (Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
            {
                Start_Clamp_Motion(Gripper_Clamp_Command_OPEN);
                break;
            }

            if (status_time_ms >= static_cast<uint32_t>(FSM_Const.Motion_Timeout_Ms + FSM_Const.Motion_Stop_Time_Ms))
            {
                Timeout_Clamp_Command = Gripper_Clamp_Command;
                Clamp_FSM.Set_Status(Gripper_Clamp_Status_TIMEOUT);
                break;
            }

            if ((status_time_ms >= FSM_Const.Ignore_Time_Ms) &&
                (status_time_ms < FSM_Const.Motion_Timeout_Ms) &&
                (fabsf(Gripper_Data.Now_Clamp_Omega_Radian) < FSM_Const.Stall_Omega))
            {
                Contact_Confirm_Time_Ms += static_cast<uint16_t>(period_ms);
            }
            else
            {
                Contact_Confirm_Time_Ms = 0U;
            }

            if (Contact_Confirm_Time_Ms >= FSM_Const.Confirm_Time_Ms)
            {
                Enter_Roll_Enabled_Status(Gripper_Clamp_Status_GRIPPED);
            }
            break;
        }

        case Gripper_Clamp_Status_GRIPPED:
        {
            if (Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
            {
                Start_Clamp_Motion(Gripper_Clamp_Command_OPEN);
            }
            break;
        }

        case Gripper_Clamp_Status_TIMEOUT:
        {
            if (Gripper_Clamp_Command != Timeout_Clamp_Command)
            {
                Start_Clamp_Motion(Gripper_Clamp_Command);
            }
            break;
        }

        default:
        {
            Clamp_FSM.Set_Status(Gripper_Clamp_Status_DISABLED);
            break;
        }
    }
}

void Class_Gripper::Output()
{
    const bool dm_online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    const bool dji_online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);
    const uint8_t status_serial = Clamp_FSM.Get_Now_Status_Serial();
    const uint32_t status_time_ms = Clamp_FSM.Status[status_serial].Time;
    const Enum_Gripper_Clamp_Status clamp_status = static_cast<Enum_Gripper_Clamp_Status>(status_serial);
    const float dji_dir = Mechanics_Const.Direction_DJI2CLAMP;

    if ((Gripper_Control_Type != Gripper_Control_Type_ENABLE) ||
        (clamp_status == Gripper_Clamp_Status_DISABLED) ||
        !dm_online || !dji_online || !DM_Motor_Rotary.Get_Radian_Initialized())
    {
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        DM_Motor_Rotary.Set_Target_Omega(0.0f);
        DJI_Motor_Clamp.Set_Current_Control(0.0f);
    }
    else
    {
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

        if ((clamp_status == Gripper_Clamp_Status_OPENED) ||
            (clamp_status == Gripper_Clamp_Status_GRIPPED))
        {
            DM_Motor_Rotary.Set_Target_Angle(Target_DM_Radian);
            DM_Motor_Rotary.Set_Target_Omega(Target_DM_Omega);
        }
        else
        {
            DM_Motor_Rotary.Set_Target_Angle(Clamp_Hold_DM_Radian);
            DM_Motor_Rotary.Set_Target_Omega(0.0f);
        }

        switch (clamp_status)
        {
            case Gripper_Clamp_Status_OPENING:
            {
                float motion_omega = Control_Const.Open_Omega;
                if (status_time_ms >= FSM_Const.Motion_Timeout_Ms)
                {
                    float stop_ratio = 1.0f - static_cast<float>(status_time_ms - FSM_Const.Motion_Timeout_Ms) /
                                              static_cast<float>(FSM_Const.Motion_Stop_Time_Ms);
                    Math_Constrain(&stop_ratio, 0.0f, 1.0f);
                    motion_omega *= stop_ratio;
                }

                const float target_dji_omega = dji_dir * (motion_omega + Now_Roll_Gear_Omega_Radian);
                DJI_Motor_Clamp.Set_Clamp_Follow_Control(dji_dir * Control_Const.Spring_Current, target_dji_omega);
                break;
            }

            case Gripper_Clamp_Status_CLOSING:
            {
                // 闭合时不施加电流，夹齿由弹簧拉动
                DJI_Motor_Clamp.Set_Current_Control(0.0f);
                break;
            }

            case Gripper_Clamp_Status_GRIPPED:
            {
                float ramp_ratio = static_cast<float>(status_time_ms) / static_cast<float>(FSM_Const.Clamp_Ramp_Time_Ms);
                Math_Constrain(&ramp_ratio, 0.0f, 1.0f);
                DJI_Motor_Clamp.Set_Clamp_Follow_Control(dji_dir * Target_Clamp_Current * ramp_ratio,
                                                         dji_dir * Now_Roll_Gear_Omega_Radian);
                break;
            }

            case Gripper_Clamp_Status_OPENED:
            {
                DJI_Motor_Clamp.Set_Clamp_Follow_Control(dji_dir * Control_Const.Spring_Current,
                                                         dji_dir * Now_Roll_Gear_Omega_Radian);
                break;
            }

            case Gripper_Clamp_Status_TIMEOUT:
            default:
            {
                DJI_Motor_Clamp.Set_Current_Control(0.0f);
                break;
            }
        }
    }

    DJI_Motor_Clamp.TIM_Control_PeriodElapsedCallback();
    DM_Motor_Rotary.TIM_Process_PeriodElapsedCallback();
}

void Class_Gripper::TIM_Calculate_PeriodElapsedCallback()
{
    Calculate_Kinematics();
    Step_Clamp_FSM(Gripper_Control_Period_Ms);
    Output();
}

void Class_Gripper::Start_Clamp_Motion(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command)
{
    Clamp_Hold_DM_Radian = DM_Motor_Rotary.Get_Now_Multi_Turn_Radian();
    Target_DM_Radian = Clamp_Hold_DM_Radian;
    Target_DM_Omega = 0.0f;
    Target_Roll_Radian = Gripper_Data.Now_Roll_Radian;
    Contact_Confirm_Time_Ms = 0U;
    DJI_Motor_Clamp.Reset_Omega_PID();

    if (__Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
    {
        Clamp_FSM.Set_Status(Gripper_Clamp_Status_OPENING);
    }
    else
    {
        Clamp_FSM.Set_Status(Gripper_Clamp_Status_CLOSING);
    }
}

void Class_Gripper::Enter_Roll_Enabled_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status)
{
    Target_Roll_Radian = Gripper_Data.Now_Roll_Radian;
    Target_DM_Radian = DM_Motor_Rotary.Get_Now_Multi_Turn_Radian();
    Target_DM_Omega = 0.0f;
    Contact_Confirm_Time_Ms = 0U;
    DJI_Motor_Clamp.Reset_Omega_PID();
    Clamp_FSM.Set_Status(__Gripper_Clamp_Status);
}

void Class_Gripper::DM_Motor_Rotary_CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    DM_Motor_Rotary.CAN_RxCpltCallback(Rx_Data);
}

void Class_Gripper::DJI_Motor_Clamp_CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    DJI_Motor_Clamp.CAN_RxCpltCallback(Rx_Data);
}
