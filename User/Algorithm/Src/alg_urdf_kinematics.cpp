/**
 * @file alg_urdf_kinematics.cpp
 * @author suzumiaoya
 * @brief Generic serial URDF kinematics implementation
 * @version V1.0
 * @date 2026-11-25
 *
 * @copyright ZLLC 2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_urdf_kinematics.h"

/* Function prototypes -------------------------------------------------------*/

void Class_URDF_Kinematics::Init(const Struct_URDF_Model *model)
{
    Model = nullptr;
    Last_Pose = {};
    Last_IK_Status = URDF_IK_Status_NOT_IMPLEMENTED;

    if ((model == nullptr) ||
        (model->joints == nullptr) ||
        (model->joint_num == 0U) ||
        (model->joint_num > URDF_MAX_JOINTS) ||
        (model->dof > URDF_MAX_DOF))
    {
        return;
    }

    for (uint8_t i = 0U; i < model->joint_num; ++i)
    {
        const Struct_URDF_Joint_Model &j = model->joints[i];
        if (j.type != URDF_Joint_Type_FIXED)
        {
            if ((j.q_index < 0) || (j.q_index >= static_cast<int8_t>(model->dof)))
            {
                return;
            }
        }
    }

    Model = model;
}

bool Class_URDF_Kinematics::Fkine(const float *q, Struct_URDF_Pose *pose)
{
    if (Model == nullptr)
    {
        return false;
    }
    return Fkine(q, static_cast<uint8_t>(Model->joint_num - 1U), pose);
}

bool Class_URDF_Kinematics::Fkine(const float *q, uint8_t target_joint_idx, Struct_URDF_Pose *pose)
{
    if ((Model == nullptr) || (pose == nullptr) || (target_joint_idx >= Model->joint_num))
    {
        return false;
    }

    float P[3] = {0.0f, 0.0f, 0.0f};
    float R[9];
    Alg_Mat3_Identity(R);

    for (uint8_t i = 0U; i <= target_joint_idx; ++i)
    {
        const Struct_URDF_Joint_Model &joint = Model->joints[i];

        float dp[3];
        Alg_Mat3_Mul_Vec3(R, joint.xyz, dp);
        P[0] += dp[0];
        P[1] += dp[1];
        P[2] += dp[2];

        Alg_Mat3_Mul(R, joint.R_fixed, R);

        if (joint.type != URDF_Joint_Type_FIXED)
        {
            if ((q == nullptr) || (joint.q_index < 0) || (joint.q_index >= static_cast<int8_t>(Model->dof)))
            {
                return false;
            }
            const float qi = q[joint.q_index];
            if (!std::isfinite(qi))
            {
                return false;
            }
            float Rj[9];
            Alg_Mat3_Rodrigues(joint.axis, qi, Rj);
            Alg_Mat3_Mul(R, Rj, R);
        }
    }

    Alg_Pose_From_Mat3(P, R, pose);
    Last_Pose = *pose;
    return true;
}

bool Class_URDF_Kinematics::Ikine(const Struct_URDF_Pose &target_pose,
                                  const float *q_ref,
                                  float *q_out,
                                  Struct_URDF_IK_Result *detail)
{
    (void)target_pose;
    (void)q_ref;
    (void)q_out;

    Last_IK_Status = URDF_IK_Status_NOT_IMPLEMENTED;
    if (detail != nullptr)
    {
        *detail = {};
        detail->status = URDF_IK_Status_NOT_IMPLEMENTED;
        detail->best_index = -1;
    }
    return false;
}

/************************ COPYRIGHT(C) ZLLC 2027 ******************************/
