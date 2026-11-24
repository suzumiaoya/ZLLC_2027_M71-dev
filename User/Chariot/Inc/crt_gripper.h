/**
 * @file crt_gripper.h
 * @author hsl by suzumiaoya
 * @brief 旋转夹爪机构类, 对外提供 Roll 角度及 OPEN/CLOSE 张合控制
 * @version V1.1
 * @date 2026-11-22
 *
 * @copyright ZLLC 2027
 *
 */

#ifndef CRT_GRIPPER_H
#define CRT_GRIPPER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"
#include <math.h>

/* Exported macros -----------------------------------------------------------*/

// 按 3A 约对应 1Nm 估算：扭矩(Nm)乘此系数得到 CAN 电流映射值，不是安培值
#define M2006_TORQUE_TO_CURRENT (3000.0f)

/* Exported types ------------------------------------------------------------*/

// 夹爪机械属性结构体
struct Struct_Gripper_Mechanics_Const
{
    float Gear_Ratio_DM2ROLL = (17.0f / 34.0f);
    float Direction_DM2ROLL = -1.0f;
    float Direction_DJI2CLAMP = 1.0f;
};

// 与张合状态机相关的常量（时间单位均为 ms）
struct Struct_Gripper_FSM_Const
{
    float Stall_Omega = 0.1f;
    float Stall_Current_Ratio = 0.9f;
    uint16_t Ignore_Time_Ms = 50U;
    uint16_t Confirm_Time_Ms = 20U;
    uint16_t Clamp_Ramp_Time_Ms = 30U;
    uint16_t Motion_Timeout_Ms = 4000U;
    uint16_t Motion_Stop_Time_Ms = 1000U;
};

// 夹爪控制常量（夹持坐标系下：负向为张开，正向为闭合）
struct Struct_Gripper_Control_Const
{
    float Omega_Kp = 1200.0f;
    float Output_Max = 9000.0f;
    float Open_Omega = -5.0f;
    float Spring_Current = -1000.0f;
    float Clamp_Current_Max = 2000.0f;
};

// 夹爪特化达妙电机子类（多圈解卷 + 仅打包控制帧）
class Class_Gripper_DM_Motor : public Class_DM_Motor
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
              float __Omega_Max = 50.0f, float __Torque_Max = 5.0f, float __Angle_Max = 3.1415f);

    void TIM_Alive_PeriodElapsedCallback() override;
    void TIM_Process_PeriodElapsedCallback() override;
    void Reset_Multi_Turn_Radian();

    inline float Get_Now_Multi_Turn_Radian() const;
    inline bool Get_Radian_Initialized() const;

protected:
    void Data_Process(uint8_t *Rx_Data) override;

private:
    volatile bool Radian_Initialized = false;
    float Pre_Feedback_Radian = 0.0f;
    volatile float Now_Multi_Turn_Radian = 0.0f;
};

// 夹爪特化 M2006 工作模式
enum Enum_Gripper_DJI_Control_Method : uint8_t
{
    Gripper_DJI_Control_Method_CURRENT = 0,
    Gripper_DJI_Control_Method_OMEGA,
    Gripper_DJI_Control_Method_CLAMP_FOLLOW,
};

// 夹爪特化 M2006 子类，实现电流开环、速度环与夹持随动速度环
class Class_Gripper_DJI_Motor : public Class_DJI_Motor_C610
{
public:
    // 固定 Torque_Max 为 10000，目标电流统一使用 CAN 映射值（1000 对应 1A）
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __Control_Method,
              float __Gearbox_Rate, float __Output_Max = 9000.0f);

    void Set_Current_Control(float __Target_Current);
    void Set_Omega_Control(float __Target_Omega_Radian);
    void Set_Clamp_Follow_Control(float __Target_Clamp_Current, float __Target_Omega_Radian);
    void Reset_Omega_PID();
    // 每控制周期调用一次，完成计算并写入 CAN 发送缓存
    void TIM_Control_PeriodElapsedCallback();

private:
    float Clamp_Output_Max = 9000.0f;
    Enum_Gripper_DJI_Control_Method Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_CURRENT;
    float Target_Current = 0.0f;
    // 仅夹持随动模式使用的目标基础电流映射值，随动分量保存在 PID_Omega 的 Out 中
    float Target_Clamp_Current = 0.0f;
};

// 夹爪控制状态枚举
enum Enum_Gripper_Control_Type : uint8_t
{
    Gripper_Control_Type_DISABLE = 0,
    Gripper_Control_Type_ENABLE,
};

// 夹爪张合指令枚举
enum Enum_Gripper_Clamp_Command : uint8_t
{
    Gripper_Clamp_Command_OPEN = 0,
    Gripper_Clamp_Command_CLOSE,
};

// 夹爪张合状态机枚举（配合 Class_FSM 使用）
enum Enum_Gripper_Clamp_Status : uint8_t
{
    Gripper_Clamp_Status_DISABLED = 0,
    Gripper_Clamp_Status_OPENING,
    Gripper_Clamp_Status_OPENED,
    Gripper_Clamp_Status_CLOSING,
    Gripper_Clamp_Status_GRIPPED,
    Gripper_Clamp_Status_TIMEOUT,
    Gripper_Clamp_Status_NUM,
};

// 夹爪运动学反馈数据结构体
struct Struct_Gripper_Data
{
    float Now_Roll_Radian;
    float Now_Roll_Omega_Radian;
    // M2006 与 DM 运动解耦后的相对张合角速度
    float Now_Clamp_Omega_Radian;
};

/**
 * @brief Specialized, 旋转夹爪机构类
 *
 */
class Class_Gripper
{
public:
    // 成员电机与张合状态机（公开便于 CAN 分发、错峰发送/存活调度及 Watch 调试）
    Class_Gripper_DM_Motor DM_Motor_Rotary;
    Class_Gripper_DJI_Motor DJI_Motor_Clamp;
    Class_FSM Clamp_FSM;

    void Init();

    // Roll 轴目标角度与最大角速度，单位分别为 rad、rad/s
    inline void Set_Target_Roll(float __Target_Roll_Radian, float __Target_Roll_Omega_Radian = 5.0f);
    inline void Set_Gripper_Clamp_Command(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command);

    // 目标夹持基础电流，使用 CAN 映射值，限于 0 ~ Clamp_Current_Max；不包含随动速度环输出
    void Set_Target_Clamp_Current(float __Target_Clamp_Current);
    inline float Get_Target_Clamp_Current() const;

    inline float Get_Now_Roll_Radian() const;
    inline float Get_Now_Roll_Omega_Radian() const;
    inline float Get_Now_Clamp_Omega_Radian() const;

    inline void Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type);
    inline Enum_Gripper_Control_Type Get_Gripper_Control_Type() const;
    inline Enum_Gripper_Clamp_Status Get_Gripper_Clamp_Status();

    inline Enum_DM_Motor_Status Get_DM_Motor_Rotary_Status();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Clamp_Status();

    // 2ms 控制周期主解算回调：运动学 -> 张合状态机 -> 电机输出打包
    void TIM_Calculate_PeriodElapsedCallback();
    // 成员电机 CAN 接收回调透传入口
    void DM_Motor_Rotary_CAN_RxCpltCallback(uint8_t *Rx_Data);
    void DJI_Motor_Clamp_CAN_RxCpltCallback(uint8_t *Rx_Data);

private:
    Struct_Gripper_Mechanics_Const Mechanics_Const;
    Struct_Gripper_FSM_Const FSM_Const;
    Struct_Gripper_Control_Const Control_Const;

    // 夹爪自身控制状态
    Enum_Gripper_Control_Type Gripper_Control_Type = Gripper_Control_Type_DISABLE;

    // Roll 控制变量
    float Target_Roll_Radian = 0.0f;
    float Target_Roll_Omega_Radian = 5.0f;
    float Target_DM_Radian = 0.0f;
    float Target_DM_Omega = 0.0f;

    // 目标夹持基础电流映射值（不包含随动速度环输出）
    float Target_Clamp_Current = 2000.0f;

    // 对外的夹爪运动学数据结构体
    Struct_Gripper_Data Gripper_Data = {};

    // 张合状态机相关变量
    Enum_Gripper_Clamp_Command Gripper_Clamp_Command = Gripper_Clamp_Command_CLOSE;
    Enum_Gripper_Clamp_Command Timeout_Clamp_Command = Gripper_Clamp_Command_CLOSE;
    float Clamp_Hold_DM_Radian = 0.0f;
    uint16_t Contact_Confirm_Time_Ms = 0U;

    // 旋转齿轮的连续角度及角速度（已包含 DM 到齿轮的传动比和方向）
    float Now_Roll_Total_Radian = 0.0f;
    float Now_Roll_Gear_Omega_Radian = 0.0f;

    void Calculate_Kinematics();
    void Step_Clamp_FSM(uint32_t period_ms);
    void Output();
    void Start_Clamp_Motion(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command);
    void Enter_Roll_Enabled_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline float Class_Gripper_DM_Motor::Get_Now_Multi_Turn_Radian() const
{
    return Now_Multi_Turn_Radian;
}

inline bool Class_Gripper_DM_Motor::Get_Radian_Initialized() const
{
    return Radian_Initialized;
}

inline void Class_Gripper::Set_Target_Roll(float __Target_Roll_Radian, float __Target_Roll_Omega_Radian)
{
    Target_Roll_Omega_Radian = fabsf(__Target_Roll_Omega_Radian);
    const Enum_Gripper_Clamp_Status status = Get_Gripper_Clamp_Status();
    if ((status == Gripper_Clamp_Status_OPENED) ||
        (status == Gripper_Clamp_Status_GRIPPED))
    {
        Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(__Target_Roll_Radian);
    }
}

inline void Class_Gripper::Set_Gripper_Clamp_Command(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command)
{
    Gripper_Clamp_Command = __Gripper_Clamp_Command;
}

inline float Class_Gripper::Get_Target_Clamp_Current() const
{
    return Target_Clamp_Current;
}

inline float Class_Gripper::Get_Now_Roll_Radian() const
{
    return Gripper_Data.Now_Roll_Radian;
}

inline float Class_Gripper::Get_Now_Roll_Omega_Radian() const
{
    return Gripper_Data.Now_Roll_Omega_Radian;
}

inline float Class_Gripper::Get_Now_Clamp_Omega_Radian() const
{
    return Gripper_Data.Now_Clamp_Omega_Radian;
}

inline void Class_Gripper::Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type)
{
    Gripper_Control_Type = __Gripper_Control_Type;
}

inline Enum_Gripper_Control_Type Class_Gripper::Get_Gripper_Control_Type() const
{
    return Gripper_Control_Type;
}

inline Enum_Gripper_Clamp_Status Class_Gripper::Get_Gripper_Clamp_Status()
{
    return static_cast<Enum_Gripper_Clamp_Status>(Clamp_FSM.Get_Now_Status_Serial());
}

inline Enum_DM_Motor_Status Class_Gripper::Get_DM_Motor_Rotary_Status()
{
    return DM_Motor_Rotary.Get_DM_Motor_Status();
}

inline Enum_DJI_Motor_Status Class_Gripper::Get_DJI_Motor_Clamp_Status()
{
    return DJI_Motor_Clamp.Get_DJI_Motor_Status();
}

#endif
