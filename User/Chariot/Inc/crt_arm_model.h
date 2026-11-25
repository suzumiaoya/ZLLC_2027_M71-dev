/**
 * @file crt_arm_model.h
 * @brief Auto-generated URDF model parameters from M71_predev_with_mass.urdf
 *        Chain: base_link -> TCP_Link (joint_num=7, dof=6)
 *
 * Usage:
 *   - Include this header anywhere to reference `extern const Struct_URDF_Model RoboticArm_URDF_Model;`
 *   - Define `#define ROBOTICARM_URDF_MODEL_DEFINE` before including this header in EXACTLY ONE `.cpp` file.
 */

#ifndef ROBOTICARM_URDF_MODEL_H
#define ROBOTICARM_URDF_MODEL_H

#include "alg_urdf_kinematics.h"

static constexpr uint8_t RoboticArm_URDF_JOINT_NUM = 7U;
static constexpr uint8_t RoboticArm_URDF_DOF = 6U;

extern const Struct_URDF_Joint RoboticArm_URDF_Joints[RoboticArm_URDF_JOINT_NUM];
extern const Struct_URDF_Model RoboticArm_URDF_Model;

#ifdef ROBOTICARM_URDF_MODEL_DEFINE

const Struct_URDF_Joint RoboticArm_URDF_Joints[RoboticArm_URDF_JOINT_NUM] = {
    // [0] J1 (base_link -> L1, type=revolute, q_idx=0)
    {
        URDF_Joint_Type_REVOLUTE,
        0,
        {0.0f, 0.0f, 0.017f},
        {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        },
        {0.0f, 0.0f, -1.0f},
        -0.78539f,
        1.5708f,
        1.29562898f,
        {0.000802036573f, -0.0368014556f, 0.0720469351f},
        {0.000962377319f, -3.7983763e-08f, -7.12943182e-06f, 0.00141670791f, 1.3739751e-07f, 0.00113754934f}
    },
    // [1] J2 (L1 -> L2, type=revolute, q_idx=1)
    {
        URDF_Joint_Type_REVOLUTE,
        1,
        {0.0f, 0.0007f, 0.084f},
        {
            -1.0f, 1.12812875e-29f, 3.23108915e-15f,
            3.23108915e-15f, 3.49148134e-15f, 1.0f,
            0.0f, 1.0f, -3.49148134e-15f
        },
        {0.0f, 0.0f, 1.0f},
        0.0f,
        1.7279f,
        0.714295135f,
        {0.276022617f, -3.78941564e-05f, 0.00316030804f},
        {0.00032849061f, -5.87243264e-08f, -7.89437099e-07f, 0.00255635571f, -5.34954922e-08f, 0.00260746702f}
    },
    // [2] J3 (L2 -> L3, type=revolute, q_idx=2)
    {
        URDF_Joint_Type_REVOLUTE,
        2,
        {0.385f, 0.0f, 0.0f},
        {
            -3.49148134e-15f, -1.0f, 0.0f,
            1.0f, -3.49148134e-15f, 0.0f,
            0.0f, 0.0f, 1.0f
        },
        {0.0f, 0.0f, 1.0f},
        -3.1416f,
        0.0f,
        0.667504405f,
        {0.0495929539f, 0.191507423f, 0.0141280833f},
        {0.00159510507f, -1.70391362e-08f, 5.89735997e-10f, 0.000400787156f, -9.27275401e-07f, 0.00158342227f}
    },
    // [3] J4 (L3 -> L4, type=revolute, q_idx=3)
    {
        URDF_Joint_Type_REVOLUTE,
        3,
        {0.057f, 0.32f, 0.0003f},
        {
            1.0f, 0.0f, 0.0f,
            0.0f, -3.49148134e-15f, 1.0f,
            0.0f, -1.0f, -3.49148134e-15f
        },
        {0.0f, 0.0f, 1.0f},
        -2.618f,
        2.618f,
        0.384225987f,
        {0.00727821715f, -0.00545245034f, 0.0425828872f},
        {0.00022728292f, -6.18353461e-08f, 6.07765529e-06f, 0.00018728242f, 2.80928544e-07f, 0.000137391548f}
    },
    // [4] J5 (L4 -> L5, type=revolute, q_idx=4)
    {
        URDF_Joint_Type_REVOLUTE,
        4,
        {0.0f, 0.0f, 0.144f},
        {
            -3.49148134e-15f, 1.0f, -3.49148134e-15f,
            0.0f, -3.49148134e-15f, -1.0f,
            -1.0f, -3.49148134e-15f, 1.21904419e-29f
        },
        {0.0f, 0.0f, 1.0f},
        -1.5708f,
        1.5708f,
        0.434683848f,
        {-0.00519731545f, 0.0200802112f, -0.0012058227f},
        {0.000153158077f, -6.81580085e-08f, -3.41704757e-11f, 7.93741447e-05f, 6.49783066e-10f, 0.000112388253f}
    },
    // [5] J6 (L5 -> L6, type=continuous, q_idx=5)
    {
        URDF_Joint_Type_CONTINUOUS,
        5,
        {-0.0279f, 0.0f, 0.0f},
        {
            -3.49148134e-15f, -3.49148134e-15f, -1.0f,
            -1.0f, 1.21904419e-29f, 3.49148134e-15f,
            0.0f, 1.0f, -3.49148134e-15f
        },
        {0.0f, 0.0f, -1.0f},
        -6.28f,
        6.28f,
        0.182901343f,
        {0.000643868889f, 2.17485535e-15f, 0.0269254561f},
        {3.35536779e-05f, 3.84000717e-06f, -2.9905979e-21f, 0.000131853983f, -3.34133804e-13f, 0.000131614299f}
    },
    // [6] TCP (L6 -> TCP_Link, type=fixed, q_idx=-1)
    {
        URDF_Joint_Type_FIXED,
        -1,
        {0.0f, 0.0f, 0.063f},
        {
            -3.49148134e-15f, -1.0f, 3.49148134e-15f,
            0.0f, -3.49148134e-15f, -1.0f,
            1.0f, -3.49148134e-15f, 1.21904419e-29f
        },
        {0.0f, 0.0f, 0.0f},
        0.0f,
        0.0f,
        0.50002319f,
        {1.97103933e-07f, 1.74323636e-06f, 0.0145582889f},
        {0.00162675068f, 5.21909069e-09f, -2.37566739e-09f, 0.00162676276f, -1.69878735e-09f, 0.000582170392f}
    }
};

const Struct_URDF_Model RoboticArm_URDF_Model = {
    RoboticArm_URDF_JOINT_NUM,
    RoboticArm_URDF_DOF,
    RoboticArm_URDF_Joints
};

#endif // ROBOTICARM_URDF_MODEL_DEFINE

#endif // ROBOTICARM_URDF_MODEL_H
