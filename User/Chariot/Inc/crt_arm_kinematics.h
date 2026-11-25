/**
 * @file crt_arm_kinematics.h
 * @author suzumiaoya
 * @brief RoboticArm 6-DOF spherical-wrist Pieper analytical kinematics
 * @version V1.0
 * @date 2026-11-25
 *
 * @copyright ZLLC 2027
 *
 */

#ifndef CRT_ARM_KINEMATICS_H
#define CRT_ARM_KINEMATICS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_urdf_kinematics.h"
#include "crt_arm_model.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 六轴球腕机械臂运动学类（基于 Pieper 解析逆解）
 * @note  Fkine 直接复用通用基类 Class_URDF_Kinematics；Ikine 由本类重写提供 8 分支精确解析解
 *
 */
class Class_RoboticArm_Kinematics : public Class_URDF_Kinematics
{
public:
    /**
     * @brief 绑定 RoboticArm_URDF_Model 并从 URDF 参数预计算 Pieper 所需几何不变量
     *
     */
    void Init();

    /**
     * @brief 设置多解优选时各关节的误差平方权重（默认大臂权重大于手腕，减少大臂甩动）
     *
     */
    void Set_Joint_Weights(const float weights[RoboticArm_URDF_DOF]);

    /**
     * @brief 设置关节限位边界判定容差，防止浮点舍入误杀极限位解，rad
     *
     */
    void Set_Limit_Tolerance(float tol_rad);

    /**
     * @brief 设置腕部奇异判定阈值（|sin(q5)| < eps 时锁住 q4 为参考角平滑求解）
     *
     */
    void Set_Wrist_Singularity_Threshold(float sin_q5_eps);

    /**
     * @brief Pieper 球腕解析逆运动学求解（相对参考关节角 q_ref 加权距离选优）
     *
     */
    __attribute__((warn_unused_result))
    bool Ikine(const Struct_URDF_Pose &target_pose,
               const float *q_ref,
               float *q_out,
               Struct_URDF_IK_Result *detail = nullptr) override;

private:
    bool Compute_R_0_3(float q1, float q2, float q3, float R_0_3[9]) const;

    float Joint_Weights[RoboticArm_URDF_DOF] = {4.0f, 4.0f, 3.0f, 1.5f, 1.5f, 1.0f};
    float Limit_Tolerance_Rad = 1e-4f;
    float Wrist_Singularity_Eps = 1e-4f;

    // 由 RoboticArm_URDF_Model 在 Init() 阶段预计算的 Pieper 几何不变量（避免运行时重复三角/矩阵推导，保持零硬编码）
    float d1_z = 0.0f;          // J1 原点 Z 向高度偏置 (m)
    float dy2 = 0.0f;           // J2 原点相对 L1 的 Y 向横向偏置 (m)
    float dz2 = 0.0f;           // J2 原点相对 L1 的 Z 向高度偏置 (m)
    float L2 = 0.0f;            // 大臂长度 J3.xyz[0] (m)
    float ux = 0.0f;            // 小臂至腕心在 L3 系下的 X 分量（折臂肩偏）(m)
    float uy = 0.0f;            // 小臂至腕心在 L3 系下的 Y 分量（含 L4 腕长）(m)
    float Y1_const = 0.0f;      // J2 与 J4 累加产生的肩-腕横向总偏置 dy2 + uz (m)
    float L3_eff = 0.0f;        // 小臂折臂至腕心的等效杆长 hypot(ux, uy) (m)
    float elbow_beta = 0.0f;    // 小臂折臂固有偏角 atan2(ux, uy) (rad)
    float R_tcp_to_6[9] = {};   // TCP 系到 L6 系的固定反旋矩阵 R_6_tcp^T
    float p_tcp_in_6[3] = {};   // TCP 原点在 L6 系下的坐标 (m)
    float p_wc_in_6[3] = {};    // 腕心（L5 原点）在 L6 系下的坐标 (m)
    float Rf4_T[9] = {};        // J4 固定旋转矩阵的转置 R_3_4_fixed^T（用于腕部欧拉角分解）
};

#endif // CRT_ARM_KINEMATICS_H

/************************ COPYRIGHT(C) ZLLC 2027 ******************************/
