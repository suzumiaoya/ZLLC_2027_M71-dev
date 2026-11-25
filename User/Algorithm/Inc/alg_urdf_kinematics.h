/**
 * @file alg_urdf_kinematics.h
 * @author suzumiaoya
 * @brief Generic serial URDF kinematics base class (mechanism-agnostic, no HAL/CMSIS deps)
 * @version V1.0
 * @date 2026-11-25
 *
 * @copyright ZLLC 2027
 *
 */

#ifndef ALG_URDF_KINEMATICS_H
#define ALG_URDF_KINEMATICS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_math_3d.h"

/* Exported macros -----------------------------------------------------------*/

static constexpr uint8_t URDF_MAX_DOF = 8U;
static constexpr uint8_t URDF_MAX_JOINTS = 12U;
static constexpr uint8_t URDF_MAX_IK_SOLUTIONS = 8U;

/* Exported types ------------------------------------------------------------*/

using Struct_URDF_Pose = Struct_Pose_3D;

/**
 * @brief URDF 关节类型
 *
 */
enum Enum_URDF_Joint_Type : uint8_t
{
    URDF_Joint_Type_REVOLUTE = 0, // 有限位旋转关节
    URDF_Joint_Type_CONTINUOUS,   // 连续无限旋转关节
    URDF_Joint_Type_FIXED,        // 固定关节（不消耗自由度）
};

/**
 * @brief URDF 单关节及子连杆统一描述（运动学与动力学共用）
 *
 */
struct Struct_URDF_Joint_Model
{
    Enum_URDF_Joint_Type type;
    int8_t q_index;   // 活动关节对应 q[] 下标（0 ~ dof-1），FIXED 为 -1
    float xyz[3];     // 关节原点相对父连杆平移 (m)
    float R_fixed[9]; // 关节固定旋转矩阵（行主序 3x3）
    float axis[3];    // 局部旋转轴单位向量（FIXED 置 0）
    float q_min;      // 关节下限 (rad)
    float q_max;      // 关节上限 (rad)
    // 子连杆惯性参数（供动力学模块复用）
    float mass;       // 质量 (kg)
    float com[3];     // 质心在子连杆坐标系下的位置 (m)
    float inertia[6]; // 质心惯量张量 [ixx, ixy, ixz, iyy, iyz, izz] (kg*m^2)
};

/**
 * @brief 串联机构 URDF 模型描述符
 *
 */
struct Struct_URDF_Model
{
    uint8_t joint_num; // 链路上总关节数（含 FIXED，<= URDF_MAX_JOINTS）
    uint8_t dof;       // 活动自由度数（<= URDF_MAX_DOF）
    const Struct_URDF_Joint_Model *joints;
};

/**
 * @brief 逆运动学状态枚举（通用封闭集合）
 *
 */
enum Enum_URDF_IK_Status : uint8_t
{
    URDF_IK_Status_OK = 0,           // 正常求解成功
    URDF_IK_Status_SINGULAR_HANDLED, // 奇异位形下已锁轴平稳求解（解有效）
    URDF_IK_Status_OUT_OF_REACH,     // 超出几何可达工作空间
    URDF_IK_Status_OUT_OF_LIMITS,    // 几何有解但全部超出关节限位
    URDF_IK_Status_NO_CONVERGENCE,   // 数值迭代未收敛（预留）
    URDF_IK_Status_INVALID_INPUT,    // 未 Init / 空指针 / 非法数值
    URDF_IK_Status_NOT_IMPLEMENTED,  // 调用了未实现解析解的基类默认 Ikine
};

/**
 * @brief 逆运动学详细诊断结果（按需传入 Ikine）
 *
 */
struct Struct_URDF_IK_Result
{
    Enum_URDF_IK_Status status;
    uint8_t raw_count;   // 限位过滤前的几何候选解数量
    uint8_t valid_count; // 通过限位过滤的有效解数量
    int8_t best_index;   // 最优解在 solutions 中的下标（无解为 -1）
    float best_cost;     // 最优解相对参考关节角的加权代价
    float solutions[URDF_MAX_IK_SOLUTIONS][URDF_MAX_DOF];
};

/**
 * @brief Generic, 通用 URDF 串联机构运动学基类
 * @note  可直接实例化并调用 Fkine（如云台或任意多轴机构）；具体机构的解析 IK 由子类重写 Ikine
 *
 */
class Class_URDF_Kinematics
{
public:
    ~Class_URDF_Kinematics() = default;

    void Init(const Struct_URDF_Model *model);

    // 正运动学：沿 URDF 链递推至末端，或递推至指定关节下标 target_joint_idx
    bool Fkine(const float *q, Struct_URDF_Pose *pose);
    bool Fkine(const float *q, uint8_t target_joint_idx, Struct_URDF_Pose *pose);

    // 解析逆运动学虚接口：直接调用基类实现会在编译期触发 deprecated 告警，运行时返回 false
    __attribute__((deprecated("Class_URDF_Kinematics::Ikine has no analytical solver; override in subclass")))
    __attribute__((warn_unused_result))
    virtual bool Ikine(const Struct_URDF_Pose &target_pose,
                       const float *q_ref,
                       float *q_out,
                       Struct_URDF_IK_Result *detail = nullptr);

    const Struct_URDF_Model *Get_Model() const { return Model; }
    uint8_t Get_DOF() const { return (Model != nullptr) ? Model->dof : 0U; }
    uint8_t Get_Joint_Num() const { return (Model != nullptr) ? Model->joint_num : 0U; }
    const Struct_URDF_Pose &Get_Last_Pose() const { return Last_Pose; }
    Enum_URDF_IK_Status Get_Last_IK_Status() const { return Last_IK_Status; }

protected:
    const Struct_URDF_Model *Model = nullptr;
    Struct_URDF_Pose Last_Pose = {};
    Enum_URDF_IK_Status Last_IK_Status = URDF_IK_Status_NOT_IMPLEMENTED;
};

#endif // ALG_URDF_KINEMATICS_H

/************************ COPYRIGHT(C) ZLLC 2027 ******************************/
