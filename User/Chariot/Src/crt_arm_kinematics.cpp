/**
 * @file crt_arm_kinematics.cpp
 * @author suzumiaoya
 * @brief RoboticArm 6-DOF spherical-wrist Pieper analytical kinematics implementation
 * @version V1.0
 * @date 2026-11-25
 *
 * @copyright ZLLC 2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#define ROBOTICARM_URDF_MODEL_DEFINE
#include "crt_arm_kinematics.h"

using namespace urdf_math;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 检查目标位姿的位置与旋转矩阵是否全部为有限数值（拦截 NaN / Inf）
 *
 */
static bool Is_Pose_Finite(const Struct_URDF_Pose &pose)
{
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (!std::isfinite(pose.pos[i]))
        {
            return false;
        }
    }
    for (uint8_t i = 0U; i < 9U; ++i)
    {
        if (!std::isfinite(pose.R[i]))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 对有限位旋转关节尝试 {q, q-2π, q+2π} 等价分支，挑选距参考角最近且满足限位的解
 * @note  用于避免诸如 J3 = -π 被 Wrap_PI 归为 +π 后被误判超限
 *
 */
static bool Adjust_Bounded_Joint(float angle, float q_ref, float q_min, float q_max, float tol, float *q_adj)
{
    const float candidates[3] = {
        angle,
        angle - kTwoPi,
        angle + kTwoPi,
    };

    bool found = false;
    float best_val = 0.0f;
    float best_dist = 0.0f;

    for (uint8_t k = 0U; k < 3U; ++k)
    {
        const float cand = candidates[k];
        if ((cand >= (q_min - tol)) && (cand <= (q_max + tol)))
        {
            const float clamped = Clamp(cand, q_min, q_max);
            const float dist = fabsf(clamped - q_ref);
            if (!found || (dist < best_dist))
            {
                found = true;
                best_val = clamped;
                best_dist = dist;
            }
        }
    }

    if (found && (q_adj != nullptr))
    {
        *q_adj = best_val;
    }
    return found;
}

void Class_RoboticArm_Kinematics::Init()
{
    Class_URDF_Kinematics::Init(&RoboticArm_URDF_Model);
    if ((Model == nullptr) || (Model->joint_num < 7U) || (Model->dof != RoboticArm_URDF_DOF))
    {
        Model = nullptr;
        return;
    }

    const Struct_URDF_Joint &j1 = Model->joints[0];
    const Struct_URDF_Joint &j2 = Model->joints[1];
    const Struct_URDF_Joint &j3 = Model->joints[2];
    const Struct_URDF_Joint &j4 = Model->joints[3];
    const Struct_URDF_Joint &j5 = Model->joints[4];
    const Struct_URDF_Joint &j6 = Model->joints[5];
    const Struct_URDF_Joint &tcp = Model->joints[6];

    d1_z = j1.xyz[2];
    dy2 = j2.xyz[1];
    dz2 = j2.xyz[2];
    L2 = j3.xyz[0];

    for (uint8_t i = 0U; i < RoboticArm_URDF_DOF; ++i)
    {
        Axis_Sign[i] = (Model->joints[i].axis[2] >= 0.0f) ? 1.0f : -1.0f;
    }

    // 腕心位于 J4/J5/J6 交点（L5 原点），将其折算到 L3 坐标系：p_wc_in_3 = J4.xyz + J4.R_fixed * J5.xyz
    float j5_in_3[3];
    Mat3_Mul_Vec3(j4.R_fixed, j5.xyz, j5_in_3);
    const float p_wc_in_3[3] = {
        j4.xyz[0] + j5_in_3[0],
        j4.xyz[1] + j5_in_3[1],
        j4.xyz[2] + j5_in_3[2],
    };

    ux = p_wc_in_3[0];
    uy = p_wc_in_3[1];
    Y1_const = dy2 + p_wc_in_3[2];
    L3_eff = hypotf(ux, uy);
    elbow_beta = atan2f(ux, uy);

    Mat3_Transpose(tcp.R_fixed, R_tcp_to_6);
    p_tcp_in_6[0] = tcp.xyz[0];
    p_tcp_in_6[1] = tcp.xyz[1];
    p_tcp_in_6[2] = tcp.xyz[2];

    // 腕心（L5 原点）在 L6 系下的坐标：p_wc_in_6 = R_5_6_fixed^T * (-J6.xyz)
    const float neg_j6_xyz[3] = {-j6.xyz[0], -j6.xyz[1], -j6.xyz[2]};
    Mat3_Transpose_Mul_Vec3(j6.R_fixed, neg_j6_xyz, p_wc_in_6);

    Mat3_Transpose(j4.R_fixed, Rf4_T);
}

void Class_RoboticArm_Kinematics::Set_Joint_Weights(const float weights[RoboticArm_URDF_DOF])
{
    if (weights == nullptr)
    {
        return;
    }
    for (uint8_t i = 0U; i < RoboticArm_URDF_DOF; ++i)
    {
        if (std::isfinite(weights[i]) && (weights[i] >= 0.0f))
        {
            Joint_Weights[i] = weights[i];
        }
    }
}

void Class_RoboticArm_Kinematics::Set_Limit_Tolerance(float tol_rad)
{
    if (std::isfinite(tol_rad) && (tol_rad >= 0.0f))
    {
        Limit_Tolerance_Rad = tol_rad;
    }
}

void Class_RoboticArm_Kinematics::Set_Wrist_Singularity_Threshold(float sin_q5_eps)
{
    if (std::isfinite(sin_q5_eps) && (sin_q5_eps > 0.0f))
    {
        Wrist_Singularity_Eps = sin_q5_eps;
    }
}

/**
 * @brief 仅计算前 3 个关节（base -> L3）的正向旋转矩阵 R_0_3，供腕部解析分解复用
 *
 */
bool Class_RoboticArm_Kinematics::Compute_R_0_3(float q1, float q2, float q3, float R_0_3[9]) const
{
    if (Model == nullptr)
    {
        return false;
    }
    const float q123[3] = {q1, q2, q3};
    Mat3_Identity(R_0_3);
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        const Struct_URDF_Joint &j = Model->joints[i];
        Mat3_Mul(R_0_3, j.R_fixed, R_0_3);
        float Rj[9];
        Mat3_Rodrigues(j.axis, q123[i], Rj);
        Mat3_Mul(R_0_3, Rj, R_0_3);
    }
    return true;
}

bool Class_RoboticArm_Kinematics::Ikine(const Struct_URDF_Pose &target_pose,
                                        const float *q_ref,
                                        float *q_out,
                                        Struct_URDF_IK_Result *detail)
{
    if (detail != nullptr)
    {
        *detail = {};
        detail->best_index = -1;
    }

    if ((Model == nullptr) || (Model->joint_num < 7U) || (q_out == nullptr) || !Is_Pose_Finite(target_pose))
    {
        Last_IK_Status = URDF_IK_Status_INVALID_INPUT;
        if (detail != nullptr)
        {
            detail->status = Last_IK_Status;
        }
        return false;
    }

    float q_ref_safe[RoboticArm_URDF_DOF] = {0.0f};
    if (q_ref != nullptr)
    {
        for (uint8_t i = 0U; i < RoboticArm_URDF_DOF; ++i)
        {
            if (!std::isfinite(q_ref[i]))
            {
                Last_IK_Status = URDF_IK_Status_INVALID_INPUT;
                if (detail != nullptr)
                {
                    detail->status = Last_IK_Status;
                }
                return false;
            }
            q_ref_safe[i] = q_ref[i];
        }
    }

    // 1. 剥离 TCP 与末端腕心偏置，求取腕心在 base_link 下的坐标 P_wc 与 L6 姿态 R_0_6
    float R_0_6[9];
    Mat3_Mul(target_pose.R, R_tcp_to_6, R_0_6);

    float dp_tcp[3];
    Mat3_Mul_Vec3(R_0_6, p_tcp_in_6, dp_tcp);
    const float P_0_6[3] = {
        target_pose.pos[0] - dp_tcp[0],
        target_pose.pos[1] - dp_tcp[1],
        target_pose.pos[2] - dp_tcp[2],
    };

    float dp_wc[3];
    Mat3_Mul_Vec3(R_0_6, p_wc_in_6, dp_wc);
    const float P_wc[3] = {
        P_0_6[0] + dp_wc[0],
        P_0_6[1] + dp_wc[1],
        P_0_6[2] + dp_wc[2],
    };

    // 2. 在规范 +Z 旋转空间求解 q1t（px*sin(q1t) - py*cos(q1t) = -Y1_const 消去横向偏置），再按 Axis_Sign[0] 还原 q1
    const float r_xy = hypotf(P_wc[0], P_wc[1]);
    if (r_xy + 1e-6f < fabsf(Y1_const))
    {
        Last_IK_Status = URDF_IK_Status_OUT_OF_REACH;
        if (detail != nullptr)
        {
            detail->status = Last_IK_Status;
        }
        return false;
    }

    float sin_val = 0.0f;
    float phi = 0.0f;
    if (r_xy > 1e-7f)
    {
        sin_val = Clamp(Y1_const / r_xy, -1.0f, 1.0f);
        phi = atan2f(P_wc[1], P_wc[0]);
    }
    const float alpha = asinf(sin_val);
    const float q1t_cands[2] = {
        Wrap_PI(phi - alpha),
        Wrap_PI(phi + alpha - kPi),
    };

    const float Z1_rel = P_wc[2] - d1_z - dz2;

    uint8_t raw_count = 0U;
    uint8_t valid_count = 0U;
    float valid_sols[URDF_MAX_IK_SOLUTIONS][RoboticArm_URDF_DOF] = {};
    float valid_costs[URDF_MAX_IK_SOLUTIONS] = {};
    bool valid_singular[URDF_MAX_IK_SOLUTIONS] = {};

    for (uint8_t b1 = 0U; b1 < 2U; ++b1)
    {
        const float q1t = q1t_cands[b1];
        const float q1 = Wrap_PI(Axis_Sign[0] * q1t);
        const float X1 = P_wc[0] * cosf(q1t) + P_wc[1] * sinf(q1t);
        const float r2_sq = X1 * X1 + Z1_rel * Z1_rel;

        // 3. 肩臂平面余弦定理求解肘上 / 肘下两组 (q2, q3)
        const float denom = 2.0f * L2 * L3_eff;
        if (denom <= 1e-9f)
        {
            continue;
        }
        const float cos_q3b = (L2 * L2 + L3_eff * L3_eff - r2_sq) / denom;
        if ((cos_q3b < -1.0f - 1e-5f) || (cos_q3b > 1.0f + 1e-5f))
        {
            continue;
        }
        const float acos_v = acosf(Clamp(cos_q3b, -1.0f, 1.0f));

        const float s3_signs[2] = {1.0f, -1.0f};
        for (uint8_t b3 = 0U; b3 < 2U; ++b3)
        {
            const float q3t = Wrap_PI(elbow_beta + s3_signs[b3] * acos_v);
            const float c3 = cosf(q3t);
            const float s3 = sinf(q3t);
            const float x2 = L2 - ux * s3 - uy * c3;
            const float y2 = ux * c3 - uy * s3;
            const float q2t = atan2f(x2 * Z1_rel + y2 * X1, -x2 * X1 + y2 * Z1_rel);
            const float q2 = Wrap_PI(Axis_Sign[1] * q2t);
            const float q3 = Wrap_PI(Axis_Sign[2] * q3t);

            // 4. 在规范 +Z 空间求解球腕 (q4t, q5t, q6t)：R_w = Rf4^T * R_0_3^T * R_0_6 = Rz(q4t) * Ry(-q5t) * Rz(pi + q6t)
            float R_0_3[9];
            Compute_R_0_3(q1, q2, q3, R_0_3);

            float R_3_6[9];
            Mat3_Transpose_Mul(R_0_3, R_0_6, R_3_6);

            float R_w[9];
            Mat3_Mul(Rf4_T, R_3_6, R_w);

            // 直接用正交列向量求 |sin(q5)| 并以 atan2 求 |q5|，规避 acosf(1-eps) 在 q5=0/pi 附近的精度损失
            const float c5 = R_w[8];
            const float s5_abs = hypotf(R_w[2], R_w[5]);
            const float q5_abs = atan2f(s5_abs, c5);

            float wrist_cands[2][3];
            uint8_t wrist_num = 0U;
            bool is_singular = false;

            if (s5_abs < Wrist_Singularity_Eps)
            {
                // 腕部奇异：锁住 q4 为参考角，求解唯一匹配的 q6
                is_singular = true;
                wrist_num = 1U;
                const float q4_lock = Clamp(q_ref_safe[3], Model->joints[3].q_min, Model->joints[3].q_max);
                const float q4t = Axis_Sign[3] * q4_lock;
                float q5t = 0.0f;
                float q6t = 0.0f;
                if (c5 >= 0.0f)
                {
                    q5t = 0.0f;
                    q6t = Wrap_PI(atan2f(-R_w[3], -R_w[0]) - q4t);
                }
                else
                {
                    q5t = kPi;
                    q6t = Wrap_PI(q4t - atan2f(R_w[3], R_w[0]));
                }
                wrist_cands[0][0] = q4_lock;
                wrist_cands[0][1] = Wrap_PI(Axis_Sign[4] * q5t);
                wrist_cands[0][2] = Wrap_PI(Axis_Sign[5] * q6t);
            }
            else
            {
                wrist_num = 2U;
                const float s5_signs[2] = {1.0f, -1.0f};
                for (uint8_t w = 0U; w < 2U; ++w)
                {
                    const float sign5 = s5_signs[w];
                    const float q4t = atan2f(-sign5 * R_w[5], -sign5 * R_w[2]);
                    const float q5t = sign5 * q5_abs;
                    const float q6t = atan2f(sign5 * R_w[7], -sign5 * R_w[6]);
                    wrist_cands[w][0] = Wrap_PI(Axis_Sign[3] * q4t);
                    wrist_cands[w][1] = Wrap_PI(Axis_Sign[4] * q5t);
                    wrist_cands[w][2] = Wrap_PI(Axis_Sign[5] * q6t);
                }
            }

            for (uint8_t w = 0U; w < wrist_num; ++w)
            {
                if (raw_count < 255U)
                {
                    ++raw_count;
                }

                float cand[RoboticArm_URDF_DOF] = {
                    q1,
                    q2,
                    q3,
                    wrist_cands[w][0],
                    wrist_cands[w][1],
                    Wrap_PI(wrist_cands[w][2]),
                };

                bool within_limits = true;
                for (uint8_t j = 0U; j < 5U; ++j)
                {
                    if (!Adjust_Bounded_Joint(cand[j],
                                              q_ref_safe[j],
                                              Model->joints[j].q_min,
                                              Model->joints[j].q_max,
                                              Limit_Tolerance_Rad,
                                              &cand[j]))
                    {
                        within_limits = false;
                        break;
                    }
                }
                if (!within_limits)
                {
                    continue;
                }

                // 去重（如肘部伸直或奇异重根）
                bool duplicate = false;
                for (uint8_t prev = 0U; prev < valid_count; ++prev)
                {
                    float max_diff = 0.0f;
                    for (uint8_t j = 0U; j < 5U; ++j)
                    {
                        const float d = fabsf(cand[j] - valid_sols[prev][j]);
                        if (d > max_diff)
                        {
                            max_diff = d;
                        }
                    }
                    const float d6 = fabsf(Wrap_PI(cand[5] - valid_sols[prev][5]));
                    if (d6 > max_diff)
                    {
                        max_diff = d6;
                    }
                    if (max_diff < 1e-4f)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate)
                {
                    continue;
                }

                if (valid_count < URDF_MAX_IK_SOLUTIONS)
                {
                    float cost = 0.0f;
                    for (uint8_t j = 0U; j < 5U; ++j)
                    {
                        const float dq = cand[j] - q_ref_safe[j];
                        cost += Joint_Weights[j] * dq * dq;
                    }
                    const float dq6 = Wrap_PI(cand[5] - q_ref_safe[5]);
                    cost += Joint_Weights[5] * dq6 * dq6;

                    for (uint8_t j = 0U; j < RoboticArm_URDF_DOF; ++j)
                    {
                        valid_sols[valid_count][j] = cand[j];
                    }
                    valid_costs[valid_count] = cost;
                    valid_singular[valid_count] = is_singular;
                    ++valid_count;
                }
            }
        }
    }

    if (raw_count == 0U)
    {
        Last_IK_Status = URDF_IK_Status_OUT_OF_REACH;
        if (detail != nullptr)
        {
            detail->status = Last_IK_Status;
            detail->raw_count = 0U;
        }
        return false;
    }

    if (valid_count == 0U)
    {
        Last_IK_Status = URDF_IK_Status_OUT_OF_LIMITS;
        if (detail != nullptr)
        {
            detail->status = Last_IK_Status;
            detail->raw_count = raw_count;
        }
        return false;
    }

    uint8_t best_idx = 0U;
    for (uint8_t i = 1U; i < valid_count; ++i)
    {
        if (valid_costs[i] < valid_costs[best_idx])
        {
            best_idx = i;
        }
    }

    for (uint8_t j = 0U; j < RoboticArm_URDF_DOF; ++j)
    {
        q_out[j] = valid_sols[best_idx][j];
    }

    Last_IK_Status = valid_singular[best_idx] ? URDF_IK_Status_SINGULAR_HANDLED : URDF_IK_Status_OK;

    if (detail != nullptr)
    {
        detail->status = Last_IK_Status;
        detail->raw_count = raw_count;
        detail->valid_count = valid_count;
        detail->best_index = static_cast<int8_t>(best_idx);
        detail->best_cost = valid_costs[best_idx];
        for (uint8_t i = 0U; i < valid_count; ++i)
        {
            for (uint8_t j = 0U; j < RoboticArm_URDF_DOF; ++j)
            {
                detail->solutions[i][j] = valid_sols[i][j];
            }
        }
    }

    return true;
}

/************************ COPYRIGHT(C) ZLLC 2027 ******************************/
