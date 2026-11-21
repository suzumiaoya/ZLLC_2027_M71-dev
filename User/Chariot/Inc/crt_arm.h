#ifndef CRT_ARM_H
#define CRT_ARM_H

#include "dvc_dmmotor.h"

/**
 * @brief 机械臂控制类型
 *
 */
enum Enum_RoboticArm_Control_Type : uint8_t {
    RoboticArm_Control_Type_DISABLE = 0, // 失能模式，所有电机均不使能
    RoboticArm_Control_Type_STANDING_BY, // 待机模式，所有电机使能但仅输出重力项，不进行位置控制
    RoboticArm_Control_Type_NORMAL,      // 正常模式，所有电机使能并进行位置控制
};

/**
 * @brief 机械臂在线情况枚举
 *
 */
enum Enum_RoboticArm_Online_Status : uint8_t {
    RoboticArm_Online_Status_OFFLINE = 0, // 全部电机离线
    RoboticArm_Online_Status_PARTLY,      // 部分电机在线/离线
    RoboticArm_Online_Status_ONLINE,      // 全部电机在线
};

/**
 * @brief 机械臂关节枚举
 *
 */
enum Enum_RoboticArm_Joint : uint8_t {
    RoboticArm_Joint_J1 = 0, // Yaw
    RoboticArm_Joint_J2,     // Pitch
    RoboticArm_Joint_J3,     // Pitch
    RoboticArm_Joint_J4,     // Roll
    RoboticArm_Joint_J5,     // Pitch
    RoboticArm_Joint_J6,      // Roll
    RoboticArm_Joint_NUM,     // 关节数量
};

/**
 * @brief Specialized, 机械臂 J4310 系列关节电机类（仅打包控制帧，由交互层 CAN 调度统一错峰发送）
 *
 */
class Class_Joint_DM_J4310 : public Class_DM_Motor_J4310
{
public:
    void TIM_Process_PeriodElapsedCallback() override;
};


/**
 * @brief Specialized, 机械臂类
 *
 */
class Class_RoboticArm
{
public:

    // 成员关节电机
    Class_Joint_DM_J4310 J1_Yaw_4340;
    Class_Joint_DM_J4310 J2_Pitch_8009P;
    Class_Joint_DM_J4310 J3_Pitch_4340;
    Class_DM_Motor_S2325 J4_Roll_2325;
    Class_DM_Motor_S2325 J5_Pitch_2325;
    Class_DM_Motor_J4310 J6_Roll_3507;

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();

    void Set_RoboticArm_Control_Type(Enum_RoboticArm_Control_Type __RoboticArm_Control_Type);
    void Set_RoboticArm_Online_Status(Enum_RoboticArm_Online_Status __RoboticArm_Online_Status);
    void Set_RoboticArm_Initialized(bool __RoboticArm_Initialized);

    void Set_Target_Joint_Radian(Enum_RoboticArm_Joint joint, float __Target_Radian);
    void Set_Target_Joint_Omega(Enum_RoboticArm_Joint joint, float __Target_Omega);

private:

    // 机械臂控制类型
    Enum_RoboticArm_Control_Type RoboticArm_Control_Type = RoboticArm_Control_Type_DISABLE;
    // 机械臂电机在线情况
    Enum_RoboticArm_Online_Status RoboticArm_Online_Status = RoboticArm_Online_Status_OFFLINE;
    // 机械臂整体初始化状态
    bool RoboticArm_Initialized = false;

    // 机械臂各关节限位
    float Joint_Max_Radian[RoboticArm_Joint_NUM] = {0};
    float Joint_Min_Radian[RoboticArm_Joint_NUM] = {0};
    // 电机输出轴到关节轴的换算系数：安装方向 / 外部减速比
    float Motor_To_Joint_Scale[RoboticArm_Joint_NUM] = {-1.0f / 2.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    // 机械臂各个关节的目标角度和角速度
    float Target_Joint_Radian[RoboticArm_Joint_NUM] = {0};
    float Target_Joint_Omega[RoboticArm_Joint_NUM] = {0};
    // 机械臂各个关节的实际角度和角速度
    float Now_Joint_Radian[RoboticArm_Joint_NUM] = {0};
    float Now_Joint_Omega[RoboticArm_Joint_NUM] = {0};
};

#endif
