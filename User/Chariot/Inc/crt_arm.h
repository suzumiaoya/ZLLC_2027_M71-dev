#ifndef CRT_ARM_H
#define CRT_ARM_H

#include "alg_fsm.h"
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
 * @brief 机械臂增量式关节（J4/J5）校准轴索引
 *
 */
enum Enum_RoboticArm_Cali_Axis : uint8_t {
    RoboticArm_Cali_Axis_J4 = 0,
    RoboticArm_Cali_Axis_J5,
    RoboticArm_Cali_Axis_NUM,
};

/**
 * @brief 机械臂后两轴校准状态机状态编号（配合 Class_FSM 使用）
 *
 */
enum Enum_RoboticArm_Cali_Status : uint8_t {
    RoboticArm_Cali_Status_WAIT_START = 0, // 等待全臂首次进入 NORMAL 触发校准
    RoboticArm_Cali_Status_PREPARE_J2_J3,  // J1 锁当前角，J2 -> +30°、J3 -> -30°（5° 阈值）
    RoboticArm_Cali_Status_CALIBRATING,    // J4/J5 同时正向堵转校准，先校完的轴直接回 0.0f
    RoboticArm_Cali_Status_RETURN_ZERO,    // 等待已校准的 J4/J5 回到 0.0f（2° 阈值）
    RoboticArm_Cali_Status_DONE,           // 校准流程结束，进入正常控制
    RoboticArm_Cali_Status_NUM,
};

/**
 * @brief 增量式单编码器关节校准参数与运行状态（输出轴/关节侧单位）
 *
 */
struct Struct_Incremental_Joint_Cali {
    // 可调标定参数（可在调试器 Watch 中直接修改标定）
    float Cali_Omega = 0.25f;          // 校准运动速度，rad/s
    float Locked_Torque = 1.8f;       // 堵转扭矩阈值，Nm（占位初值，台架结合实测扭矩调整）
    float Locked_Omega = 0.05f;       // 堵转角速度阈值，rad/s
    uint16_t Confirm_Count = 25U;     // 堵转连续判定计数（2ms 周期下 25 次 = 50ms）
    uint16_t Decay_Step = 4U;         // 不满足堵转条件时每次递减步长
    uint32_t Timeout_Ms = 13000U;     // 单轴校准超时时间，ms

    // 运行态与调试观测字段
    bool Calibrated = false;          // 是否已完成校准（仅通信断开或手动重校时清零）
    bool Timed_Out = false;           // 本次校准是否超时（超时仅失能本轴）
    bool Returned_Zero = false;       // 校准完成后是否已回到 0.0f
    bool Search_Started = false;      // 本轮校准是否已锁定搜索目标
    float Zero_Offset = 0.0f;         // 电机输出轴侧零点偏置：Joint_Rad = Scale * (Motor_Angle - Zero_Offset)
    float Search_Target_Angle = 0.0f; // 校准搜索目标角（输出轴侧），rad
    uint16_t Locked_Cnt = 0;          // 堵转防抖计数器
    uint32_t Elapsed_Ms = 0;          // 本轮校准已用时间，ms
    float Now_Torque = 0.0f;          // 实时反馈扭矩（输出轴侧），Nm
    float Peak_Torque = 0.0f;         // 本轮校准期间最大绝对扭矩（输出轴侧），Nm
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
 * @brief Specialized, 机械臂 S2325 系列关节电机类（仅打包控制帧，由交互层 CAN 调度统一错峰发送）
 *
 */
class Class_Joint_DM_S2325 : public Class_DM_Motor_S2325
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
    Class_Joint_DM_S2325 J4_Roll_2325;
    Class_Joint_DM_S2325 J5_Pitch_2325;
    Class_DM_Motor_J4310 J6_Roll_3507;

    // 后两轴（J4/J5）校准状态机与单轴标定观测数据（便于 Watch 调试）
    Class_FSM Calibration_FSM;
    Struct_Incremental_Joint_Cali Joint_Cali[RoboticArm_Cali_Axis_NUM];

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();

    void Set_RoboticArm_Control_Type(Enum_RoboticArm_Control_Type __RoboticArm_Control_Type);
    void Set_RoboticArm_Online_Status(Enum_RoboticArm_Online_Status __RoboticArm_Online_Status);
    void Set_RoboticArm_Initialized(bool __RoboticArm_Initialized);
    void Set_Calibration_Switch(bool __Enable);

    void Set_Target_Joint_Radian(Enum_RoboticArm_Joint joint, float __Target_Radian);
    void Set_Target_Joint_Omega(Enum_RoboticArm_Joint joint, float __Target_Omega);

    // 预留手动重校准入口（供后续按键触发）
    void Request_Recalibration();
    bool Get_RoboticArm_Initialized() const { return RoboticArm_Initialized; }
    Enum_RoboticArm_Cali_Status Get_Calibration_Status()
    {
        return static_cast<Enum_RoboticArm_Cali_Status>(Calibration_FSM.Get_Now_Status_Serial());
    }

private:
    void Update_Incremental_Joint_Disconnect(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM]);
    void Disable_Controlled_Joints(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM]);
    void Hold_First_Three_Joints(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM], float max_joint_omega);
    void Step_Incremental_Joint_Calibration(Enum_RoboticArm_Cali_Axis axis, Class_DM_Motor &motor);
    void Command_Calibrated_Incremental_Joint(Enum_RoboticArm_Cali_Axis axis, Class_DM_Motor &motor, float max_joint_omega);
    void Update_Calibration_FSM(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM]);
    void Run_Normal_Control(Class_DM_Motor *const joint_motors[RoboticArm_Joint_NUM]);

    // 机械臂控制类型
    Enum_RoboticArm_Control_Type RoboticArm_Control_Type = RoboticArm_Control_Type_DISABLE;
    // 机械臂电机在线情况
    Enum_RoboticArm_Online_Status RoboticArm_Online_Status = RoboticArm_Online_Status_OFFLINE;
    // 机械臂整体初始化状态
    bool RoboticArm_Initialized = false;
    // 校准/运行开关（由遥控器 SWA 控制：true=开，false=关）
    bool Calibration_Switch_Enable = false;
    // 关掉 SWA 后，保持前三轴在当前位置锁位（目标角速度置 0），仅失能 J4/J5
    bool First_Three_Hold_Active = false;

    // 机械臂各关节限位，单位 rad（对应 M71_predev.urdf 中 J1~J6 的 limit）
    float Joint_Max_Radian[RoboticArm_Joint_NUM] = {1.5708f, 1.7279f, 0.0f, 2.618f, 1.5708f, 6.28f};
    float Joint_Min_Radian[RoboticArm_Joint_NUM] = {-0.78539f, 0.0f, -3.1416f, -2.618f, -1.5708f, -6.28f};
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
