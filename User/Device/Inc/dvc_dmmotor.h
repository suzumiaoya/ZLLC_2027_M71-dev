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
enum Enum_DM_Motor_ID : uint8_t
{
    DM_Motor_ID_UNDEFINED = 0,
    DM_Motor_ID_0xA1,
    DM_Motor_ID_0xA2,
    DM_Motor_ID_0xA3,
    DM_Motor_ID_0xA4,
    DM_Motor_ID_0xA5,
    DM_Motor_ID_0xA6,
    DM_Motor_ID_0xA7,
    DM_Motor_ID_0xA8,
};

enum Enum_DM_Motor_ErrorCode : uint8_t
{
    ERROR_OverVoltage = 8,
    ERROR_UnderVoltage,
    ERROR_OverCurrent,
    ERROR_MOS_OverTemp,
    ERROR_Motor_OverTemp,
    ERROR_LossCAN,
    ERROR_OverLoad,
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
    Enum_DM_Motor_ID CAN_ID;
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
    Enum_DM_Motor_ID CAN_ID;
    Enum_DM_Motor_ErrorCode ErrorCode;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
};

/**
 * @brief 达妙电机抽象基类, 由具体型号实现控制帧发送
 */
class Class_DM_Motor
{
public:

    virtual ~Class_DM_Motor() = default;

    virtual void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method, float __Omega_Max, float __Torque_Max, float __Angle_Max) = 0;

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
    void TIM_Alive_PeriodElapsedCallback();
    virtual void TIM_Process_PeriodElapsedCallback() = 0;
    virtual void TIM_PID_PeriodElapsedCallback();

protected:

    //初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    Enum_DM_Motor_ID CAN_ID;
    uint8_t *CAN_Tx_Data = nullptr;
    // 协议量程, 与调参助手中的 PMAX、VMAX、TMAX 一致
    float Angle_Max;
    float Omega_Max;
    float Torque_Max;

    //内部变量
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
    // 使用相同反馈协议的子类应先调用父类实现, 再转换到输出轴侧
    virtual void Data_Process(uint8_t *Rx_Data);
};

/**
 * @brief 达妙电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_DM_Motor_Rx_Data
{
    Enum_DM_Motor_ID CAN_ID;
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
class Class_DM_Motor_J4310
{
public:

    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method = DM_Motor_Control_Method_MIT_POSITION, int32_t __Position_Offset = 0, float __Omega_Max = 20.94359f, float __Torque_Max = 10.0f);

    Enum_DM_Motor_Control_Status Get_DM_Motor_Control_Status();
    Enum_DM_Motor_Status Get_DM_Motor_Status();
    float Get_Now_Angle();
    float Get_Now_Radian();
    float Get_Now_Omega();
    float Get_Now_Torque();
    float Get_Now_MOS_Temperature();
    float Get_Now_Rotor_Temperature();
    Enum_DM_Motor_Control_Method Get_Control_Method();
    float Get_MIT_K_P();
    float Get_MIT_K_D();
    float Get_Target_Angle();
    float Get_Target_Omega();
    float Get_Target_Torque();
    float Get_Out();
    
    void Set_DM_Control_Status(Enum_DM_Motor_Control_Status __DM_Motor_Control_Status);
    void Set_DM_Motor_Control_Method(Enum_DM_Motor_Control_Method __DM_Motor_Control_Method);
    void Set_MIT_K_P(float __MIT_K_P);
    void Set_MIT_K_D(float __MIT_K_D);
    void Set_Target_Angle(float __Target_Angle);
    void Set_Target_Omega(float __Target_Omega);
    void Set_Target_Torque(float __Target_Torque);
    void Set_Out(float __Out);
	void Limit_Out();

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_DM_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //位置反馈偏移
    uint32_t Position_Offset;
    //最大速度, 调参助手设置, 推荐20.94359, 也就是最大转速200rpm
    float Omega_Max;
    //最大扭矩, 调参助手设置, 推荐7, 也就是最大输出7NM
    float Torque_Max;
    //输出量
    float Out=0.0f;
    //常量
    
    //一圈位置刻度
    uint32_t Position_Max = 65536;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //电机状态
    Enum_DM_Motor_Status DM_Motor_Status = DM_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_DM_Motor_Rx_Data Data;

    //写变量

    //读写变量

    //电机控制状态
    Enum_DM_Motor_Control_Status DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    //电机控制方式
    Enum_DM_Motor_Control_Method DM_Motor_Control_Method = DM_Motor_Control_Method_MIT_POSITION;
    //MIT的Kp值, 0~500, 空载6, 位置控制需要
    float MIT_K_P = 0.0f;
    //MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
    float MIT_K_D = 0.0f;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的扭矩
    float Target_Torque = 0.0f;

    //内部函数

    void Data_Process(uint8_t* Rx_Data);
    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
