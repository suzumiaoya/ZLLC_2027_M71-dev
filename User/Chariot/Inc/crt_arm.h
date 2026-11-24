#ifndef CRT_ARM_H
#define CRT_ARM_H

#include "alg_fsm.h"
#include "crt_gripper.h"
#include "dvc_dmmotor.h"

/**
 * @brief 机械臂控制类型
 *
 */
enum Enum_RoboticArm_Control_Type : uint8_t {
    RoboticArm_Control_Type_DISABLE = 0, // 失能模式，所有电机均不使能
    RoboticArm_Control_Type_STANDING_BY, // 待机模式，预留给后续重力补偿
    RoboticArm_Control_Type_NORMAL,      // 正常模式，所有电机使能并进行位置控制
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
    RoboticArm_Joint_J6,     // Roll（差动无限旋转夹爪）
    RoboticArm_Joint_NUM,    // 关节数量
};

/**
 * @brief 增量式 S2325 单轴校准状态机状态编号（配合 Class_FSM 使用）
 *
 */
enum Enum_Joint_S2325_Cali_Status : uint8_t {
    Joint_S2325_Cali_Status_IDLE = 0,     // 未校准 / 掉线清零后待机
    Joint_S2325_Cali_Status_SEARCHING,    // 正向寻找机械硬限位并检测堵转
    Joint_S2325_Cali_Status_RETURN_ZERO,  // 已标定零偏，正在回中（0.0f）
    Joint_S2325_Cali_Status_DONE,         // 已校准且回中完成
    Joint_S2325_Cali_Status_TIMEOUT,      // 搜索超时（本轴失能）
    Joint_S2325_Cali_Status_NUM,
};

/**
 * @brief 机械臂整臂校准协调状态机状态编号（配合 Class_FSM 使用）
 *
 */
enum Enum_RoboticArm_Cali_Status : uint8_t {
    RoboticArm_Cali_Status_WAIT_START = 0, // 等待 J1~J5 全部在线
    RoboticArm_Cali_Status_PREPARE_J2_J3,  // J1 锁当前角，J2 -> +30°、J3 -> -30°（5° 阈值）
    RoboticArm_Cali_Status_CALIBRATING,    // J4/J5 同时校准并回中（2° 阈值）
    RoboticArm_Cali_Status_DONE,           // 校准成功，进入正常控制
    RoboticArm_Cali_Status_FAILED,         // 校准失败（有轴超时），整臂失能，切回 DISABLE 后可重试
    RoboticArm_Cali_Status_NUM,
};

/**
 * @brief 增量式 S2325 单轴校准参数与运行观测数据（输出轴/关节侧单位）
 *
 */
struct Struct_Joint_S2325_Cali {
    // 可调标定参数（可在调试器 Watch 中直接修改标定）
    float Cali_Omega = 2.0f;                                // 校准与回中速度，rad/s
    float Locked_Torque = 2.0f;                             // 堵转扭矩阈值，Nm
    float Locked_Omega = 0.16f;                             // 堵转角速度阈值，rad/s（VMAX=200 时 1 LSB≈0.0977，静止码值±0.0488，1 LSB 抖动±0.1465）
    uint16_t Confirm_Count = 25U;                           // 堵转连续判定计数（2ms 周期下 25 次 = 50ms）
    uint16_t Decay_Step = 4U;                               // 不满足堵转条件时每次递减步长
    uint32_t Timeout_Ms = 25000U;                           // 正向找限位超时时间，ms
    float Return_Zero_Tolerance_Radian = 2.0f * DEG_TO_RAD; // 回中到位阈值，rad
    float Search_Extra_Radian = 0.25f;                       // 搜索目标超出满行程余量，rad

    // 运行态与调试观测字段
    float Zero_Offset = 0.0f;                               // 电机输出轴侧零点偏置：Joint_Rad = Scale * (Motor_Angle - Zero_Offset)
    float Search_Target_Angle = 0.0f;                       // 校准搜索目标角（输出轴侧），rad
    uint16_t Locked_Cnt = 0;                                // 堵转防抖计数器
    float Now_Torque = 0.0f;                                // 实时反馈扭矩（输出轴侧），Nm
    float Peak_Torque = 0.0f;                               // 本轮校准期间最大绝对扭矩（输出轴侧），Nm
};

/**
 * @brief Specialized, 机械臂 J4310 系列单电机关节类（关节换算/限位 + 仅打包控制帧）
 *
 */
class Class_Joint_DM_J4310 : public Class_DM_Motor
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
              float __Omega_Max, float __Torque_Max, float __Angle_Max,
              float joint_min_rad, float joint_max_rad, float motor_to_joint_scale);
    void TIM_Process_PeriodElapsedCallback() override;

    void Set_Joint_Target_Radian(float target_joint_radian);
    void Set_Joint_Target_Omega(float max_joint_omega);

    float Get_Joint_Radian() { return Motor_To_Joint_Scale * Get_Now_Angle(); }
    float Get_Joint_Omega() { return Motor_To_Joint_Scale * Get_Now_Omega(); }
    float Get_Joint_Min_Radian() const { return Joint_Min_Radian; }
    float Get_Joint_Max_Radian() const { return Joint_Max_Radian; }

private:
    float Joint_Min_Radian = 0.0f;
    float Joint_Max_Radian = 0.0f;
    float Motor_To_Joint_Scale = 1.0f;
};

/**
 * @brief Specialized, 机械臂 S2325 系列单电机增量关节类（关节换算/限位 + 单轴校准 FSM + 仅打包控制帧）
 *
 */
class Class_Joint_DM_S2325 : public Class_DM_Motor
{
public:
    // 单轴校准状态机与标定观测数据（公开便于 Watch 调试）
    Class_FSM Cali_FSM;
    Struct_Joint_S2325_Cali Cali;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
              float __Omega_Max, float __Torque_Max, float __Angle_Max,
              float joint_min_rad, float joint_max_rad, float motor_to_joint_scale, uint32_t cali_timeout_ms);
    void TIM_Process_PeriodElapsedCallback() override;

    // 供外部调用的校准入口与掉线监测
    void Update_Disconnect_Check();
    void Start_Calibration();
    void Reset_Calibration();
    void Step_Calibration(uint32_t period_ms);

    void Set_Joint_Target_Radian(float target_joint_radian);
    void Set_Joint_Target_Omega(float max_joint_omega);

    float Get_Joint_Radian();
    float Get_Joint_Omega() { return Motor_To_Joint_Scale * Get_Now_Omega(); }
    float Get_Joint_Min_Radian() const { return Joint_Min_Radian; }
    float Get_Joint_Max_Radian() const { return Joint_Max_Radian; }
    Enum_Joint_S2325_Cali_Status Get_Calibration_Status()
    {
        return static_cast<Enum_Joint_S2325_Cali_Status>(Cali_FSM.Get_Now_Status_Serial());
    }
    bool Is_Calibrated() { return Get_Calibration_Status() == Joint_S2325_Cali_Status_DONE; }

private:
    float Joint_Min_Radian = 0.0f;
    float Joint_Max_Radian = 0.0f;
    float Motor_To_Joint_Scale = 1.0f;
};


/**
 * @brief Specialized, 机械臂类
 *
 */
class Class_RoboticArm
{
public:
    // 成员关节电机与末端夹爪
    Class_Joint_DM_J4310 J1_Yaw_4340;
    Class_Joint_DM_J4310 J2_Pitch_8009P;
    Class_Joint_DM_J4310 J3_Pitch_4340;
    Class_Joint_DM_S2325 J4_Roll_2325;
    Class_Joint_DM_S2325 J5_Pitch_2325;
    Class_Gripper Gripper;

    // 整臂校准协调状态机（公开便于 Watch 调试）
    Class_FSM Calibration_FSM;

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();

    void Set_RoboticArm_Control_Type(Enum_RoboticArm_Control_Type __RoboticArm_Control_Type);
    void Set_Target_Joint_Radian(Enum_RoboticArm_Joint joint, float __Target_Radian);
    void Set_Target_Joint_Omega(Enum_RoboticArm_Joint joint, float __Target_Omega);
    float Get_Target_Joint_Radian(Enum_RoboticArm_Joint joint) const;
    float Get_Now_Joint_Radian(Enum_RoboticArm_Joint joint) const;

    bool Get_RoboticArm_Initialized() const { return RoboticArm_Initialized; }
    Enum_RoboticArm_Cali_Status Get_Calibration_Status()
    {
        return static_cast<Enum_RoboticArm_Cali_Status>(Calibration_FSM.Get_Now_Status_Serial());
    }

private:
    void Update_Feedback();
    void Disable_Controlled_Joints();
    void Update_Calibration_FSM();
    void Run_Normal_Control();
    void Output();

    // 机械臂控制类型
    Enum_RoboticArm_Control_Type RoboticArm_Control_Type = RoboticArm_Control_Type_DISABLE;
    // 机械臂整体初始化状态（J4/J5 均校准完成后置 true）
    bool RoboticArm_Initialized = false;
    // 校准前置阶段锁存的 J1 角度（不覆写外部 Target_Joint_Radian）
    float Cali_Prepare_J1_Radian = 0.0f;

    // 机械臂外部目标角度和角速度（仅 NORMAL 模式使用）
    float Target_Joint_Radian[RoboticArm_Joint_NUM] = {0};
    float Target_Joint_Omega[RoboticArm_Joint_NUM] = {0};
    // 机械臂各关节当前实际角度和角速度
    float Now_Joint_Radian[RoboticArm_Joint_NUM] = {0};
    float Now_Joint_Omega[RoboticArm_Joint_NUM] = {0};
};

#endif
