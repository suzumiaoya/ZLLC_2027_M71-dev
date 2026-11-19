/**
 * @file dvc_dmmotor.h
 * @author HGCSJ by yssickjgd
 * @brief 达妙电机配置与操作
 * @version V1.1
 * @date 2026-10-30 27赛季定稿
 *
 * @copyright ZLLC 2027
 *
 */

#ifndef DVC_DMMOTOR_H
#define DVC_DMMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机状态
 *
 */
enum Enum_DM_Motor_Status
{
    DM_Motor_Status_DISABLE = 0,
    DM_Motor_Status_ENABLE,
};

/**
 * @brief 达妙电机的ID枚举类型
 *
 */
enum Enum_DM_Motor_ID : uint16_t
{
    DM_Motor_ID_UNDEFINED = 0,
    // 枚举值为上位机实际配置的 CAN_ID。
    DM_Motor_ID_0xA1 = 0xF1,
    DM_Motor_ID_0xA2 = 0xF2,
    DM_Motor_ID_0xA3 = 0xF3,
    DM_Motor_ID_0xA4 = 0xF4,
    DM_Motor_ID_0xA5 = 0xF5,
    DM_Motor_ID_0xA6 = 0xF6,
    DM_Motor_ID_0xA7 = 0xF7,
    DM_Motor_ID_0xA8 = 0xF8,

    // 同总线尾号须唯一。速度模式占用 0x201～0x208，禁止与大疆电机共线。
    DM_Motor_ID_0x01 = 0x01,
    DM_Motor_ID_0x02 = 0x02,
    DM_Motor_ID_0x03 = 0x03,
    DM_Motor_ID_0x04 = 0x04,
    DM_Motor_ID_0x05 = 0x05,
    DM_Motor_ID_0x06 = 0x06,
    DM_Motor_ID_0x07 = 0x07,
    DM_Motor_ID_0x08 = 0x08,
};

enum Enum_DM_Motor_ErrorCode : uint8_t
{
    ERROR_Disabled = 0x0,               // 失能
    ERROR_Enabled = 0x1,                // 使能
    ERROR_OutputShaftCalibration = 0x3, // 输出轴校准异常
    ERROR_SensorOutput = 0x4,           // 传感器输出异常
    ERROR_MotorSensor = 0x5,
    ERROR_MotorParameterRead = 0x6,     // 读取电机参数错误
    ERROR_OverVoltage = 0x8,            // 超压
    ERROR_UnderVoltage = 0x9,           // 欠压
    ERROR_OverCurrent = 0xA,            // 过电流
    ERROR_MOS_OverTemp = 0xB,           // MOS 过温
    ERROR_Motor_OverTemp = 0xC,         // 电机线圈过温
    ERROR_LossCAN = 0xD,                // 通讯丢失
    ERROR_OverLoad = 0xE,               // 过载
};


/**
 * @brief 达妙电机控制状态
 *
 */
enum Enum_DM_Motor_Control_Status
{
    DM_Motor_Control_Status_DISABLE = 0,
    DM_Motor_Control_Status_ENABLE,
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_DM_Motor_Control_Method
{
    DM_Motor_Control_Method_MIT_POSITION = 0,
    DM_Motor_Control_Method_MIT_OMEGA,
    DM_Motor_Control_Method_MIT_ANGLE,
    DM_Motor_Control_Method_POSITION_OMEGA,
    DM_Motor_Control_Method_OMEGA,
    DM_Motor_Control_Method_MIT_IMU_Angle,
    DM_Motor_Control_Method_MIT_OPENLOOP
};
/**
 * @brief 达妙电机源数据
 *
 */
struct Struct_DM_Motor_CAN_Rx_Data
{
    uint8_t CAN_ID; // 原始 D0：高四位 ERR，低四位 ID
    uint16_t Position_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机公共反馈数据
 */
struct Struct_DM_Motor_Feedback
{
    uint8_t CAN_ID; // 反馈 D0 的低四位
    Enum_DM_Motor_ErrorCode ErrorCode;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
};

/**
 * @brief 达妙电机基类，由具体型号公开初始化接口
 */
class Class_DM_Motor
{
public:

    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    virtual ~Class_DM_Motor() = default;

    Enum_DM_Motor_Status Get_DM_Motor_Status();
    Enum_DM_Motor_Control_Status Get_DM_Motor_Control_Status();
    Enum_DM_Motor_ErrorCode Get_ErrorCode();
    // 获取电机当前角度, rad
    float Get_Now_Angle();
    // 获取电机当前角速度，rad/s
    float Get_Now_Omega();
    // 获取电机当前扭矩，Nm
    float Get_Now_Torque();
    // 获取电机当前MOS管温度，摄氏度
    float Get_Now_MOS_Temperature();
    // 获取电机当前绕组温度，摄氏度
    float Get_Now_Rotor_Temperature();
    Enum_DM_Motor_Control_Method Get_Control_Method();
    float Get_MIT_K_P();
    float Get_MIT_K_D();
    float Get_Target_Angle();
    float Get_Target_Omega();
    float Get_Target_Torque();

    void Set_DM_Control_Status(Enum_DM_Motor_Control_Status __Control_Status);
    void Set_DM_Motor_Control_Method(Enum_DM_Motor_Control_Method __Control_Method);
    void Set_MIT_K_P(float __MIT_K_P);
    void Set_MIT_K_D(float __MIT_K_D);
    void Set_Target_Angle(float __Target_Angle);
    void Set_Target_Omega(float __Target_Omega);
    void Set_Target_Torque(float __Target_Torque);

    // Rx_Data 指向一帧已分发给本对象的 8 字节反馈
    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    virtual void TIM_Alive_PeriodElapsedCallback();
    virtual void TIM_Process_PeriodElapsedCallback();
    virtual void TIM_PID_PeriodElapsedCallback();

    // 提交最近一次打包的控制帧, 返回CAN发送状态
    uint8_t Send_Control_Frame();

protected:
    Class_DM_Motor() = default;
    Class_DM_Motor(const Class_DM_Motor&) = default;
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method, float __Omega_Max, float __Torque_Max, float __Angle_Max);

    //初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    Enum_DM_Motor_ID CAN_ID;
    uint8_t *CAN_Tx_Data = nullptr;
    // 协议量程, 与调参助手中的 PMAX、VMAX、TMAX 一致
    float Angle_Max;
    float Omega_Max;
    float Torque_Max;

    //内部变量
    // 待发送控制帧的ID和长度, 长度为0表示没有待发送帧
    uint16_t Control_Tx_ID = 0;
    uint16_t Control_Tx_Length = 0;
    volatile uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    //读变量
    Enum_DM_Motor_Status DM_Motor_Status = DM_Motor_Status_DISABLE;
    Struct_DM_Motor_Feedback Data = {};

    //读写变量
    Enum_DM_Motor_Control_Status DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    Enum_DM_Motor_Control_Method DM_Motor_Control_Method;
    float MIT_K_P = 0.0f;
    float MIT_K_D = 0.0f;
    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;

    //内部函数
    virtual void Pack_Control_Frame(float angle, float omega, float torque, float kp, float kd);
    // 使用相同反馈协议的子类应先调用父类实现, 再转换到输出轴侧
    virtual void Data_Process(uint8_t *Rx_Data);
};

/**
 * @brief 达妙电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_DM_Motor_Rx_Data
{
    uint8_t CAN_ID; // 反馈 D0 的低四位
    Enum_DM_Motor_ErrorCode ErrorCode;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint16_t Pre_Position;
    int32_t Total_Position;
    int32_t Total_Round;
};

/**
 * @brief J4310无刷电机, 单片机控制输出控制帧
 * DM_Motor_Control_Method_POSITION_OMEGA模式下, 需调参助手辅助设置位置环PI参数, 空载250与0
 * 
 * PMAX值需在调参助手设置为3.141593, 即PI, 此时可在MIT模式下当舵机使用
 *
 */
class Class_DM_Motor_J4310 : public Class_DM_Motor
{
public:

    virtual void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method = DM_Motor_Control_Method_MIT_POSITION, float __Omega_Max = 30.0f, float __Torque_Max = 10.0f, float __Angle_Max = PI);

    float Get_Out();
    void Set_Out(float __Out);
    void Limit_Out();

protected:
    //输出量
    float Out = 0.0f;

    void Output();
};

/**
 * @brief S2325 及采用相同转子侧协议的增量式电机。角度以本次上电位置为零点。
 * 量程与上位机 VMAX/TMAX/PMAX 一致；PMAX 应覆盖减速比乘以关节全行程并留有余量。
 * 减速比为转子速度 / 输出轴速度；传 1 可直接使用转子侧单位。
 */
class Class_DM_Motor_S2325 : public Class_DM_Motor
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method,
              float __Omega_Max, float __Torque_Max, float __Angle_Max, float __Gear_Ratio);

protected:
    void Pack_Control_Frame(float angle, float omega, float torque, float kp, float kd) override;
    void Data_Process(uint8_t *Rx_Data) override;

private:
    float Gear_Ratio = 1.0f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
