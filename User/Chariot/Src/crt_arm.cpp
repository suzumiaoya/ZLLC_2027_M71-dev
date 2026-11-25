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

static constexpr uint32_t Control_Period_Ms = 2U;                         // 交互层每 2ms 调用一次

static constexpr float Default_Normal_Joint_Omega = 1.0f;                 // 正常模式默认关节最大速度，rad/s
static constexpr float Cali_Prepare_J2_Radian = 30.0f * DEG_TO_RAD;       // +30°
static constexpr float Cali_Prepare_J3_Radian = -30.0f * DEG_TO_RAD;      // -30°
static constexpr float Cali_Prepare_Omega = 0.5f;                         // 前置移位速度上限，rad/s
static constexpr float Cali_Prepare_Tolerance_Radian = 5.0f * DEG_TO_RAD; // 前置到位阈值 5°
static constexpr uint32_t Cali_J4_Timeout_Ms = 25000U;                    // J4 满行程 5.24 rad / 0.25 rad/s + 余量
static constexpr uint32_t Cali_J5_Timeout_Ms = 16000U;                    // J5 满行程 3.14 rad / 0.25 rad/s + 余量

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_Joint_DM_J4310::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
                                float __Omega_Max, float __Torque_Max, float __Angle_Max,
                                float joint_min_rad, float joint_max_rad, float motor_to_joint_scale)
{
    Class_DM_Motor::Init(hcan, __CAN_ID, __Control_Method, __Omega_Max, __Torque_Max, __Angle_Max);
    Joint_Min_Radian = joint_min_rad;
    Joint_Max_Radian = joint_max_rad;
    Motor_To_Joint_Scale = motor_to_joint_scale;
}

void Class_Joint_DM_J4310::TIM_Process_PeriodElapsedCallback()
{
    Pack_Control_Frame(Target_Angle, Target_Omega, Target_Torque, MIT_K_P, MIT_K_D);
}

void Class_Joint_DM_J4310::Set_Joint_Target_Radian(float target_joint_radian)
{
    Math_Constrain(&target_joint_radian, Joint_Min_Radian, Joint_Max_Radian);
    Set_Target_Angle(target_joint_radian / Motor_To_Joint_Scale);
}

void Class_Joint_DM_J4310::Set_Joint_Target_Omega(float max_joint_omega)
{
    Set_Target_Omega(max_joint_omega / fabsf(Motor_To_Joint_Scale));
}

void Class_Joint_DM_S2325::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
                                float __Omega_Max, float __Torque_Max, float __Angle_Max,
                                float joint_min_rad, float joint_max_rad, float motor_to_joint_scale, uint32_t cali_timeout_ms)
{
    Class_DM_Motor::Init(hcan, __CAN_ID, __Control_Method, __Omega_Max, __Torque_Max, __Angle_Max);
    Joint_Min_Radian = joint_min_rad;
    Joint_Max_Radian = joint_max_rad;
    Motor_To_Joint_Scale = motor_to_joint_scale;

    Cali = Struct_Joint_S2325_Cali{};
    Cali.Timeout_Ms = cali_timeout_ms;
    Cali_FSM.Init(Joint_S2325_Cali_Status_NUM, Joint_S2325_Cali_Status_IDLE);
}

void Class_Joint_DM_S2325::TIM_Process_PeriodElapsedCallback()
{
    Pack_Control_Frame(Target_Angle, Target_Omega, Target_Torque, MIT_K_P, MIT_K_D);
}

void Class_Joint_DM_S2325::Update_Disconnect_Check()
{
    Cali.Now_Torque = Get_Now_Torque();

    if (Get_DM_Motor_Status() != DM_Motor_Status_ENABLE)
    {
        // 增量式编码器通信断开视为可能下电：失能并清零校准态，由外部决定何时重校。
        Reset_Calibration();
    }
}

void Class_Joint_DM_S2325::Start_Calibration()
{
    const float full_stroke = Joint_Max_Radian - Joint_Min_Radian;
    Cali.Zero_Offset = 0.0f;
    Cali.Search_Target_Angle = Get_Now_Angle() + (full_stroke + Cali.Search_Extra_Radian) / Motor_To_Joint_Scale;
    Cali.Locked_Cnt = 0;
    Cali.Peak_Torque = 0.0f;
    Cali_FSM.Set_Status(Joint_S2325_Cali_Status_SEARCHING);
}

void Class_Joint_DM_S2325::Reset_Calibration()
{
    Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    Cali.Zero_Offset = 0.0f;
    Cali.Locked_Cnt = 0;
    Cali.Peak_Torque = 0.0f;
    if (Get_Calibration_Status() != Joint_S2325_Cali_Status_IDLE)
    {
        Cali_FSM.Set_Status(Joint_S2325_Cali_Status_IDLE);
    }
}

void Class_Joint_DM_S2325::Set_Joint_Target_Radian(float target_joint_radian)
{
    Math_Constrain(&target_joint_radian, Joint_Min_Radian, Joint_Max_Radian);
    Set_Target_Angle(target_joint_radian / Motor_To_Joint_Scale + Cali.Zero_Offset);
}

void Class_Joint_DM_S2325::Set_Joint_Target_Omega(float max_joint_omega)
{
    Set_Target_Omega(max_joint_omega / fabsf(Motor_To_Joint_Scale));
}

float Class_Joint_DM_S2325::Get_Joint_Radian()
{
    const Enum_Joint_S2325_Cali_Status status = Get_Calibration_Status();
    const bool has_offset = (status == Joint_S2325_Cali_Status_RETURN_ZERO) ||
                            (status == Joint_S2325_Cali_Status_DONE);
    return Motor_To_Joint_Scale * (Get_Now_Angle() - (has_offset ? Cali.Zero_Offset : 0.0f));
}

void Class_Joint_DM_S2325::Step_Calibration(uint32_t period_ms)
{
    const uint8_t status = Cali_FSM.Get_Now_Status_Serial();
    Cali_FSM.Status[status].Time += period_ms;

    switch (static_cast<Enum_Joint_S2325_Cali_Status>(status))
    {
        case Joint_S2325_Cali_Status_SEARCHING:
        {
            if (Cali_FSM.Status[status].Time >= Cali.Timeout_Ms)
            {
                Cali.Locked_Cnt = 0;
                Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                Cali_FSM.Set_Status(Joint_S2325_Cali_Status_TIMEOUT);
                break;
            }

            Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Set_Joint_Target_Omega(Cali.Cali_Omega);
            Set_Target_Angle(Cali.Search_Target_Angle);

            const float abs_torque = fabsf(Get_Now_Torque());
            const float abs_omega = fabsf(Get_Now_Omega());
            if (abs_torque > Cali.Peak_Torque)
            {
                Cali.Peak_Torque = abs_torque;
            }

            if (Get_ErrorCode() != ERROR_Enabled)
            {
                break;
            }

            if (abs_torque >= Cali.Locked_Torque && abs_omega <= Cali.Locked_Omega)
            {
                Cali.Locked_Cnt++;
                if (Cali.Locked_Cnt >= Cali.Confirm_Count)
                {
                    Cali.Locked_Cnt = 0;
                    Cali.Zero_Offset = Get_Now_Angle() - Joint_Max_Radian / Motor_To_Joint_Scale;
                    Cali_FSM.Set_Status(Joint_S2325_Cali_Status_RETURN_ZERO);
                    Set_Joint_Target_Radian(0.0f);
                    Set_Joint_Target_Omega(Cali.Cali_Omega);
                }
            }
            else
            {
                Cali.Locked_Cnt = (Cali.Locked_Cnt > Cali.Decay_Step) ? (Cali.Locked_Cnt - Cali.Decay_Step) : 0U;
            }
            break;
        }

        case Joint_S2325_Cali_Status_RETURN_ZERO:
        {
            Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Set_Joint_Target_Radian(0.0f);
            Set_Joint_Target_Omega(Cali.Cali_Omega);

            if (Get_ErrorCode() == ERROR_Enabled &&
                fabsf(Get_Joint_Radian()) <= Cali.Return_Zero_Tolerance_Radian)
            {
                Cali_FSM.Set_Status(Joint_S2325_Cali_Status_DONE);
            }
            break;
        }

        case Joint_S2325_Cali_Status_DONE:
        {
            Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Set_Joint_Target_Radian(0.0f);
            Set_Joint_Target_Omega(Cali.Cali_Omega);
            break;
        }

        case Joint_S2325_Cali_Status_IDLE:
        case Joint_S2325_Cali_Status_TIMEOUT:
        default:
        {
            Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            break;
        }
    }
}

void Class_Joint_Gripper::Init(float roll_sign, float zero_offset_rad)
{
    Class_Gripper::Init();
    // 保持父类 Class_Gripper 内部差动随动方向不变，仅在对外关节边界做 URDF 符号与零偏对齐
    Roll_Sign = (roll_sign < 0.0f) ? -1.0f : 1.0f;
    Joint_Zero_Offset = zero_offset_rad;
}

void Class_Joint_Gripper::Set_Joint_Target_Radian(float target_joint_radian, float max_joint_omega)
{
    const float target_roll_radian = Normalize_Angle_Radian_PI_to_PI(target_joint_radian * Roll_Sign + Joint_Zero_Offset);
    Set_Target_Roll(target_roll_radian, max_joint_omega);
}

float Class_Joint_Gripper::Get_Joint_Radian() const
{
    return Normalize_Angle_Radian_PI_to_PI(Roll_Sign * (Get_Now_Roll_Radian() - Joint_Zero_Offset));
}

float Class_Joint_Gripper::Get_Joint_Omega() const
{
    return Roll_Sign * Get_Now_Roll_Omega_Radian();
}

void Class_RoboticArm::Init()
{
    // 量程参数顺序：VMAX、TMAX、PMAX（FDCAN1 反馈 ID：0x11～0x16）；限位对应 M71_predev.urdf。
    J1_Yaw_4340.Init(&hfdcan1, DM_Motor_ID_0x01, DM_Motor_Control_Method_POSITION_OMEGA, 10.0f, 9.0f, 12.5f,
                     -0.78539f, 1.5708f, -1.0f / 2.0f);
    J2_Pitch_8009P.Init(&hfdcan1, DM_Motor_ID_0x02, DM_Motor_Control_Method_POSITION_OMEGA, 20.0f, 20.0f, 12.5f,
                        0.0f, 1.7279f, 1.0f);
    J3_Pitch_4340.Init(&hfdcan1, DM_Motor_ID_0x03, DM_Motor_Control_Method_POSITION_OMEGA, 20.0f, 25.0f, 12.5f,
                       -3.1416f, 0.0f, 1.0f);
    // J4 S2325 驱动器内置 GR 锁死为 19.2032，机械实际减速比为 25，用 Motor_To_Joint_Scale 补齐差值。
    J4_Roll_2325.Init(&hfdcan1, DM_Motor_ID_0x04, DM_Motor_Control_Method_POSITION_OMEGA, 200.0f, 10.0f, 122.0f,
                      -2.618f, 2.618f, 19.2032f / 25.0f, Cali_J4_Timeout_Ms);
    J5_Pitch_2325.Init(&hfdcan1, DM_Motor_ID_0x05, DM_Motor_Control_Method_POSITION_OMEGA, 200.0f, 10.0f, 80.0f,
                       -1.5708f, 1.5708f, 1.0f, Cali_J5_Timeout_Ms);
    Gripper.Init(1.0f, 0.0f);
    Kinematics.Init();
    Now_TCP_Pose = {};

    for (uint8_t joint = 0; joint < RoboticArm_Joint_NUM; ++joint)
    {
        Target_Joint_Omega[joint] = Default_Normal_Joint_Omega;
    }

    RoboticArm_Initialized = false;
    Cali_Prepare_J1_Radian = 0.0f;
    Calibration_FSM.Init(RoboticArm_Cali_Status_NUM, RoboticArm_Cali_Status_WAIT_START);
}

void Class_RoboticArm::Set_RoboticArm_Control_Type(Enum_RoboticArm_Control_Type __RoboticArm_Control_Type)
{
    RoboticArm_Control_Type = __RoboticArm_Control_Type;
}

void Class_RoboticArm::Set_Target_Joint_Radian(Enum_RoboticArm_Joint joint, float __Target_Radian)
{
    switch (joint)
    {
        case RoboticArm_Joint_J1:
            Math_Constrain(&__Target_Radian, J1_Yaw_4340.Get_Joint_Min_Radian(), J1_Yaw_4340.Get_Joint_Max_Radian());
            break;
        case RoboticArm_Joint_J2:
            Math_Constrain(&__Target_Radian, J2_Pitch_8009P.Get_Joint_Min_Radian(), J2_Pitch_8009P.Get_Joint_Max_Radian());
            break;
        case RoboticArm_Joint_J3:
            Math_Constrain(&__Target_Radian, J3_Pitch_4340.Get_Joint_Min_Radian(), J3_Pitch_4340.Get_Joint_Max_Radian());
            break;
        case RoboticArm_Joint_J4:
            Math_Constrain(&__Target_Radian, J4_Roll_2325.Get_Joint_Min_Radian(), J4_Roll_2325.Get_Joint_Max_Radian());
            break;
        case RoboticArm_Joint_J5:
            Math_Constrain(&__Target_Radian, J5_Pitch_2325.Get_Joint_Min_Radian(), J5_Pitch_2325.Get_Joint_Max_Radian());
            break;
        case RoboticArm_Joint_J6:
            __Target_Radian = Normalize_Angle_Radian_PI_to_PI(__Target_Radian);
            break;
        default:
            return;
    }
    Target_Joint_Radian[joint] = __Target_Radian;
}

void Class_RoboticArm::Set_Target_Joint_Omega(Enum_RoboticArm_Joint joint, float __Target_Omega)
{
    if (joint < RoboticArm_Joint_NUM)
    {
        Target_Joint_Omega[joint] = __Target_Omega;
    }
}

float Class_RoboticArm::Get_Target_Joint_Radian(Enum_RoboticArm_Joint joint) const
{
    if (joint < RoboticArm_Joint_NUM)
    {
        return Target_Joint_Radian[joint];
    }
    return 0.0f;
}

float Class_RoboticArm::Get_Now_Joint_Radian(Enum_RoboticArm_Joint joint) const
{
    if (joint < RoboticArm_Joint_NUM)
    {
        return Now_Joint_Radian[joint];
    }
    return 0.0f;
}

void Class_RoboticArm::Update_Feedback()
{
    J4_Roll_2325.Update_Disconnect_Check();
    J5_Pitch_2325.Update_Disconnect_Check();
    if (!J4_Roll_2325.Is_Calibrated() || !J5_Pitch_2325.Is_Calibrated())
    {
        RoboticArm_Initialized = false;
    }

    Now_Joint_Radian[RoboticArm_Joint_J1] = J1_Yaw_4340.Get_Joint_Radian();
    Now_Joint_Omega[RoboticArm_Joint_J1] = J1_Yaw_4340.Get_Joint_Omega();
    Now_Joint_Radian[RoboticArm_Joint_J2] = J2_Pitch_8009P.Get_Joint_Radian();
    Now_Joint_Omega[RoboticArm_Joint_J2] = J2_Pitch_8009P.Get_Joint_Omega();
    Now_Joint_Radian[RoboticArm_Joint_J3] = J3_Pitch_4340.Get_Joint_Radian();
    Now_Joint_Omega[RoboticArm_Joint_J3] = J3_Pitch_4340.Get_Joint_Omega();
    Now_Joint_Radian[RoboticArm_Joint_J4] = J4_Roll_2325.Get_Joint_Radian();
    Now_Joint_Omega[RoboticArm_Joint_J4] = J4_Roll_2325.Get_Joint_Omega();
    Now_Joint_Radian[RoboticArm_Joint_J5] = J5_Pitch_2325.Get_Joint_Radian();
    Now_Joint_Omega[RoboticArm_Joint_J5] = J5_Pitch_2325.Get_Joint_Omega();
    Now_Joint_Radian[RoboticArm_Joint_J6] = Gripper.Get_Joint_Radian();
    Now_Joint_Omega[RoboticArm_Joint_J6] = Gripper.Get_Joint_Omega();

    // 每控制周期基于实时关节反馈刷新末端 TCP 位姿（当前仅用于观测）
    Kinematics.Fkine(Now_Joint_Radian, &Now_TCP_Pose);
}

void Class_RoboticArm::Disable_Controlled_Joints()
{
    J1_Yaw_4340.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    J2_Pitch_8009P.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    J3_Pitch_4340.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    J4_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    J5_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
}

void Class_RoboticArm::Update_Calibration_FSM()
{
    const uint8_t status = Calibration_FSM.Get_Now_Status_Serial();
    Calibration_FSM.Status[status].Time += Control_Period_Ms;

    switch (static_cast<Enum_RoboticArm_Cali_Status>(status))
    {
        case RoboticArm_Cali_Status_WAIT_START:
        {
            const bool all_online = (J1_Yaw_4340.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE) &&
                                    (J2_Pitch_8009P.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE) &&
                                    (J3_Pitch_4340.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE) &&
                                    (J4_Roll_2325.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE) &&
                                    (J5_Pitch_2325.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
            if (!all_online)
            {
                Disable_Controlled_Joints();
                break;
            }

            Cali_Prepare_J1_Radian = Now_Joint_Radian[RoboticArm_Joint_J1];
            Calibration_FSM.Set_Status(RoboticArm_Cali_Status_PREPARE_J2_J3);
            break;
        }

        case RoboticArm_Cali_Status_PREPARE_J2_J3:
        {
            // 前置移位期间：J4/J5 保持失能；J1 锁当前角，J2/J3 以 0.5 rad/s 移动到 ±30°。
            J1_Yaw_4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J2_Pitch_8009P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J3_Pitch_4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J1_Yaw_4340.Set_Joint_Target_Radian(Cali_Prepare_J1_Radian);
            J2_Pitch_8009P.Set_Joint_Target_Radian(Cali_Prepare_J2_Radian);
            J3_Pitch_4340.Set_Joint_Target_Radian(Cali_Prepare_J3_Radian);
            J1_Yaw_4340.Set_Joint_Target_Omega(Cali_Prepare_Omega);
            J2_Pitch_8009P.Set_Joint_Target_Omega(Cali_Prepare_Omega);
            J3_Pitch_4340.Set_Joint_Target_Omega(Cali_Prepare_Omega);
            J4_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
            J5_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

            const bool j2_ready = (J2_Pitch_8009P.Get_ErrorCode() == ERROR_Enabled) &&
                                  (fabsf(Now_Joint_Radian[RoboticArm_Joint_J2] - Cali_Prepare_J2_Radian) <= Cali_Prepare_Tolerance_Radian);
            const bool j3_ready = (J3_Pitch_4340.Get_ErrorCode() == ERROR_Enabled) &&
                                  (fabsf(Now_Joint_Radian[RoboticArm_Joint_J3] - Cali_Prepare_J3_Radian) <= Cali_Prepare_Tolerance_Radian);
            if (j2_ready && j3_ready)
            {
                J4_Roll_2325.Start_Calibration();
                J5_Pitch_2325.Start_Calibration();
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_CALIBRATING);
            }
            break;
        }

        case RoboticArm_Cali_Status_CALIBRATING:
        {
            J1_Yaw_4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J2_Pitch_8009P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J3_Pitch_4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J1_Yaw_4340.Set_Joint_Target_Radian(Cali_Prepare_J1_Radian);
            J2_Pitch_8009P.Set_Joint_Target_Radian(Cali_Prepare_J2_Radian);
            J3_Pitch_4340.Set_Joint_Target_Radian(Cali_Prepare_J3_Radian);
            J1_Yaw_4340.Set_Joint_Target_Omega(Cali_Prepare_Omega);
            J2_Pitch_8009P.Set_Joint_Target_Omega(Cali_Prepare_Omega);
            J3_Pitch_4340.Set_Joint_Target_Omega(Cali_Prepare_Omega);

            J4_Roll_2325.Step_Calibration(Control_Period_Ms);
            J5_Pitch_2325.Step_Calibration(Control_Period_Ms);

            const Enum_Joint_S2325_Cali_Status j4_status = J4_Roll_2325.Get_Calibration_Status();
            const Enum_Joint_S2325_Cali_Status j5_status = J5_Pitch_2325.Get_Calibration_Status();
            if (j4_status == Joint_S2325_Cali_Status_TIMEOUT ||
                j5_status == Joint_S2325_Cali_Status_TIMEOUT ||
                j4_status == Joint_S2325_Cali_Status_IDLE ||
                j5_status == Joint_S2325_Cali_Status_IDLE)
            {
                // 任一轴超时或掉线即视为校准失败：全臂失能，禁止进入正常控制。
                Disable_Controlled_Joints();
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_FAILED);
                break;
            }

            if (J4_Roll_2325.Is_Calibrated() && J5_Pitch_2325.Is_Calibrated())
            {
                // 无扰交接至 NORMAL：前三轴保持当前角，后两轴保持 0.0f。
                Target_Joint_Radian[RoboticArm_Joint_J1] = Now_Joint_Radian[RoboticArm_Joint_J1];
                Target_Joint_Radian[RoboticArm_Joint_J2] = Now_Joint_Radian[RoboticArm_Joint_J2];
                Target_Joint_Radian[RoboticArm_Joint_J3] = Now_Joint_Radian[RoboticArm_Joint_J3];
                Target_Joint_Radian[RoboticArm_Joint_J4] = 0.0f;
                Target_Joint_Radian[RoboticArm_Joint_J5] = 0.0f;
                RoboticArm_Initialized = true;
                Calibration_FSM.Set_Status(RoboticArm_Cali_Status_DONE);
            }
            break;
        }

        case RoboticArm_Cali_Status_FAILED:
        {
            Disable_Controlled_Joints();
            break;
        }

        case RoboticArm_Cali_Status_DONE:
        default:
            break;
    }
}

void Class_RoboticArm::Run_Normal_Control()
{
    Class_Joint_DM_J4310 *const first_three[3] = {&J1_Yaw_4340, &J2_Pitch_8009P, &J3_Pitch_4340};
    for (uint8_t j = RoboticArm_Joint_J1; j <= RoboticArm_Joint_J3; ++j)
    {
        Class_Joint_DM_J4310 &joint = *first_three[j];
        if (joint.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
        {
            Target_Joint_Radian[j] = Now_Joint_Radian[j];
        }
        joint.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        joint.Set_Joint_Target_Radian(Target_Joint_Radian[j]);
        joint.Set_Joint_Target_Omega(Target_Joint_Omega[j]);
    }

    if (J4_Roll_2325.Is_Calibrated())
    {
        if (J4_Roll_2325.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
        {
            Target_Joint_Radian[RoboticArm_Joint_J4] = Now_Joint_Radian[RoboticArm_Joint_J4];
        }
        J4_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        J4_Roll_2325.Set_Joint_Target_Radian(Target_Joint_Radian[RoboticArm_Joint_J4]);
        J4_Roll_2325.Set_Joint_Target_Omega(Target_Joint_Omega[RoboticArm_Joint_J4]);
    }
    else
    {
        J4_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }

    if (J5_Pitch_2325.Is_Calibrated())
    {
        if (J5_Pitch_2325.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
        {
            Target_Joint_Radian[RoboticArm_Joint_J5] = Now_Joint_Radian[RoboticArm_Joint_J5];
        }
        J5_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        J5_Pitch_2325.Set_Joint_Target_Radian(Target_Joint_Radian[RoboticArm_Joint_J5]);
        J5_Pitch_2325.Set_Joint_Target_Omega(Target_Joint_Omega[RoboticArm_Joint_J5]);
    }
    else
    {
        J5_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
}

void Class_RoboticArm::Output()
{
    J1_Yaw_4340.TIM_Process_PeriodElapsedCallback();
    J2_Pitch_8009P.TIM_Process_PeriodElapsedCallback();
    J3_Pitch_4340.TIM_Process_PeriodElapsedCallback();
    J4_Roll_2325.TIM_Process_PeriodElapsedCallback();
    J5_Pitch_2325.TIM_Process_PeriodElapsedCallback();
    Gripper.TIM_Calculate_PeriodElapsedCallback();
}

void Class_RoboticArm::TIM_Calculate_PeriodElapsedCallback()
{
    Update_Feedback();

    if (RoboticArm_Control_Type == RoboticArm_Control_Type_NORMAL)
    {
        // 夹爪不与 J4/J5 校准阶段联动，切入 NORMAL 即直接使能（无扰锁存当前 Roll 角）
        if (Gripper.Get_Gripper_Control_Type() == Gripper_Control_Type_DISABLE)
        {
            Target_Joint_Radian[RoboticArm_Joint_J6] = Now_Joint_Radian[RoboticArm_Joint_J6];
        }
        Gripper.Set_Gripper_Control_Type(Gripper_Control_Type_ENABLE);
        Gripper.Set_Joint_Target_Radian(Target_Joint_Radian[RoboticArm_Joint_J6], Target_Joint_Omega[RoboticArm_Joint_J6]);

        if (Calibration_FSM.Get_Now_Status_Serial() == RoboticArm_Cali_Status_DONE)
        {
            Run_Normal_Control();
        }
        else
        {
            Update_Calibration_FSM();
        }
    }
    else
    {
        Disable_Controlled_Joints();
        Gripper.Set_Gripper_Control_Type(Gripper_Control_Type_DISABLE);
        // 校准未完成时切回 DISABLE，不保留半成品状态，下次完整重校；已完成校准的则保留状态。
        if (Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_DONE &&
            Calibration_FSM.Get_Now_Status_Serial() != RoboticArm_Cali_Status_WAIT_START)
        {
            J4_Roll_2325.Reset_Calibration();
            J5_Pitch_2325.Reset_Calibration();
            Calibration_FSM.Set_Status(RoboticArm_Cali_Status_WAIT_START);
        }
    }

    Output();
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
