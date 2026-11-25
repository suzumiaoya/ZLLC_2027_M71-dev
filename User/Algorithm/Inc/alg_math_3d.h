/**
 * @file alg_math_3d.h
 * @author suzumiaoya
 * @brief Pure C++11 header-only 3D geometry & attitude utilities (row-major 3x3, no HAL/CMSIS deps)
 * @version V1.0
 * @date 2026-11-25
 *
 * @copyright ZLLC 2027
 *
 */

#ifndef ALG_MATH_3D_H
#define ALG_MATH_3D_H

/* Includes ------------------------------------------------------------------*/

#include <cmath>
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

static constexpr float ALG_MATH_3D_PI = 3.14159265358979323846f;
static constexpr float ALG_MATH_3D_TWO_PI = 6.28318530717958647692f;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 通用 3D 位姿结构体
 * @note  R 为行主序 3x3 旋转矩阵；quat 为 [w, x, y, z]；rpy 为 URDF 标准 [roll, pitch, yaw]（ZYX 内旋）
 *
 */
struct Struct_Pose_3D
{
    float pos[3];
    float R[9];
    float quat[4];
    float rpy[3];
};

/* Exported function declarations / inline implementations -------------------*/

inline float Alg_Clamp_f(float val, float min_val, float max_val)
{
    if (val < min_val)
    {
        return min_val;
    }
    if (val > max_val)
    {
        return max_val;
    }
    return val;
}

inline float Alg_Wrap_PI(float rad)
{
    if (!std::isfinite(rad))
    {
        return 0.0f;
    }
    float wrapped = remainderf(rad, ALG_MATH_3D_TWO_PI);
    if (wrapped == -ALG_MATH_3D_PI)
    {
        wrapped = ALG_MATH_3D_PI;
    }
    return wrapped;
}

inline float Alg_Vec3_Dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void Alg_Vec3_Cross(const float a[3], const float b[3], float out[3])
{
    const float ox = a[1] * b[2] - a[2] * b[1];
    const float oy = a[2] * b[0] - a[0] * b[2];
    const float oz = a[0] * b[1] - a[1] * b[0];
    out[0] = ox;
    out[1] = oy;
    out[2] = oz;
}

inline void Alg_Mat3_Identity(float R[9])
{
    R[0] = 1.0f; R[1] = 0.0f; R[2] = 0.0f;
    R[3] = 0.0f; R[4] = 1.0f; R[5] = 0.0f;
    R[6] = 0.0f; R[7] = 0.0f; R[8] = 1.0f;
}

inline void Alg_Mat3_Transpose(const float A[9], float AT[9])
{
    if (A == AT)
    {
        float t = AT[1]; AT[1] = AT[3]; AT[3] = t;
        t = AT[2]; AT[2] = AT[6]; AT[6] = t;
        t = AT[5]; AT[5] = AT[7]; AT[7] = t;
        return;
    }
    AT[0] = A[0]; AT[1] = A[3]; AT[2] = A[6];
    AT[3] = A[1]; AT[4] = A[4]; AT[5] = A[7];
    AT[6] = A[2]; AT[7] = A[5]; AT[8] = A[8];
}

inline void Alg_Mat3_Mul(const float A[9], const float B[9], float C[9])
{
    float tmp[9];
    for (uint8_t r = 0; r < 3U; ++r)
    {
        const uint8_t row = static_cast<uint8_t>(r * 3U);
        for (uint8_t c = 0; c < 3U; ++c)
        {
            tmp[row + c] = A[row + 0U] * B[0U + c] +
                           A[row + 1U] * B[3U + c] +
                           A[row + 2U] * B[6U + c];
        }
    }
    for (uint8_t i = 0; i < 9U; ++i)
    {
        C[i] = tmp[i];
    }
}

inline void Alg_Mat3_Transpose_Mul(const float A[9], const float B[9], float C[9])
{
    float tmp[9];
    for (uint8_t r = 0; r < 3U; ++r)
    {
        for (uint8_t c = 0; c < 3U; ++c)
        {
            tmp[r * 3U + c] = A[0U + r] * B[0U + c] +
                              A[3U + r] * B[3U + c] +
                              A[6U + r] * B[6U + c];
        }
    }
    for (uint8_t i = 0; i < 9U; ++i)
    {
        C[i] = tmp[i];
    }
}

inline void Alg_Mat3_Mul_Vec3(const float A[9], const float v[3], float out[3])
{
    const float o0 = A[0] * v[0] + A[1] * v[1] + A[2] * v[2];
    const float o1 = A[3] * v[0] + A[4] * v[1] + A[5] * v[2];
    const float o2 = A[6] * v[0] + A[7] * v[1] + A[8] * v[2];
    out[0] = o0;
    out[1] = o1;
    out[2] = o2;
}

inline void Alg_Mat3_Transpose_Mul_Vec3(const float A[9], const float v[3], float out[3])
{
    const float o0 = A[0] * v[0] + A[3] * v[1] + A[6] * v[2];
    const float o1 = A[1] * v[0] + A[4] * v[1] + A[7] * v[2];
    const float o2 = A[2] * v[0] + A[5] * v[1] + A[8] * v[2];
    out[0] = o0;
    out[1] = o1;
    out[2] = o2;
}

inline void Alg_Mat3_Rodrigues(const float axis[3], float angle, float R[9])
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float t = 1.0f - c;
    const float x = axis[0];
    const float y = axis[1];
    const float z = axis[2];

    R[0] = t * x * x + c;      R[1] = t * x * y - s * z;  R[2] = t * x * z + s * y;
    R[3] = t * x * y + s * z;  R[4] = t * y * y + c;      R[5] = t * y * z - s * x;
    R[6] = t * x * z - s * y;  R[7] = t * y * z + s * x;  R[8] = t * z * z + c;
}

inline void Alg_RPY_To_Mat3(const float rpy[3], float R[9])
{
    const float cr = cosf(rpy[0]), sr = sinf(rpy[0]);
    const float cp = cosf(rpy[1]), sp = sinf(rpy[1]);
    const float cy = cosf(rpy[2]), sy = sinf(rpy[2]);

    R[0] = cy * cp;
    R[1] = cy * sp * sr - sy * cr;
    R[2] = cy * sp * cr + sy * sr;
    R[3] = sy * cp;
    R[4] = sy * sp * sr + cy * cr;
    R[5] = sy * sp * cr - cy * sr;
    R[6] = -sp;
    R[7] = cp * sr;
    R[8] = cp * cr;
}

inline void Alg_Mat3_To_RPY(const float R[9], float rpy[3])
{
    const float sin_p = Alg_Clamp_f(-R[6], -1.0f, 1.0f);
    const float pitch = asinf(sin_p);
    const float cos_p = cosf(pitch);

    if (fabsf(cos_p) > 1e-6f)
    {
        rpy[0] = atan2f(R[7], R[8]);
        rpy[1] = pitch;
        rpy[2] = atan2f(R[3], R[0]);
    }
    else
    {
        // 万向锁保护：pitch = ±pi/2 时令 roll = 0
        rpy[0] = 0.0f;
        rpy[1] = pitch;
        rpy[2] = atan2f(-R[1], R[4]);
    }
}

inline void Alg_Mat3_To_Quat(const float R[9], float quat[4])
{
    // Shepperd 四分支法（全角度无奇异，避免 180° 除零 NaN）
    const float trace = R[0] + R[4] + R[8];
    float w, x, y, z;

    if (trace > 0.0f)
    {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (R[7] - R[5]) / s;
        y = (R[2] - R[6]) / s;
        z = (R[3] - R[1]) / s;
    }
    else if ((R[0] > R[4]) && (R[0] > R[8]))
    {
        const float s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        w = (R[7] - R[5]) / s;
        x = 0.25f * s;
        y = (R[1] + R[3]) / s;
        z = (R[2] + R[6]) / s;
    }
    else if (R[4] > R[8])
    {
        const float s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        w = (R[2] - R[6]) / s;
        x = (R[1] + R[3]) / s;
        y = 0.25f * s;
        z = (R[5] + R[7]) / s;
    }
    else
    {
        const float s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        w = (R[3] - R[1]) / s;
        x = (R[2] + R[6]) / s;
        y = (R[5] + R[7]) / s;
        z = 0.25f * s;
    }

    const float norm = sqrtf(w * w + x * x + y * y + z * z);
    if (norm > 1e-9f)
    {
        const float inv_n = (w < 0.0f) ? (-1.0f / norm) : (1.0f / norm);
        quat[0] = w * inv_n;
        quat[1] = x * inv_n;
        quat[2] = y * inv_n;
        quat[3] = z * inv_n;
    }
    else
    {
        quat[0] = 1.0f;
        quat[1] = 0.0f;
        quat[2] = 0.0f;
        quat[3] = 0.0f;
    }
}

inline void Alg_Quat_To_Mat3(const float quat[4], float R[9])
{
    const float norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    if (norm < 1e-9f)
    {
        Alg_Mat3_Identity(R);
        return;
    }

    const float inv_n = 1.0f / norm;
    const float w = quat[0] * inv_n;
    const float x = quat[1] * inv_n;
    const float y = quat[2] * inv_n;
    const float z = quat[3] * inv_n;

    R[0] = 1.0f - 2.0f * (y * y + z * z);
    R[1] = 2.0f * (x * y - w * z);
    R[2] = 2.0f * (x * z + w * y);
    R[3] = 2.0f * (x * y + w * z);
    R[4] = 1.0f - 2.0f * (x * x + z * z);
    R[5] = 2.0f * (y * z - w * x);
    R[6] = 2.0f * (x * z - w * y);
    R[7] = 2.0f * (y * z + w * x);
    R[8] = 1.0f - 2.0f * (x * x + y * y);
}

inline void Alg_Pose_From_Mat3(const float pos[3], const float R[9], Struct_Pose_3D *pose)
{
    if (pose == nullptr)
    {
        return;
    }
    pose->pos[0] = pos[0];
    pose->pos[1] = pos[1];
    pose->pos[2] = pos[2];
    for (uint8_t i = 0; i < 9U; ++i)
    {
        pose->R[i] = R[i];
    }
    Alg_Mat3_To_Quat(R, pose->quat);
    Alg_Mat3_To_RPY(R, pose->rpy);
}

inline Struct_Pose_3D Alg_Make_Pose_From_RPY(const float pos[3], const float rpy[3])
{
    Struct_Pose_3D pose{};
    pose.pos[0] = pos[0];
    pose.pos[1] = pos[1];
    pose.pos[2] = pos[2];
    Alg_RPY_To_Mat3(rpy, pose.R);
    Alg_Mat3_To_Quat(pose.R, pose.quat);
    pose.rpy[0] = rpy[0];
    pose.rpy[1] = rpy[1];
    pose.rpy[2] = rpy[2];
    return pose;
}

inline Struct_Pose_3D Alg_Make_Pose_From_Quat(const float pos[3], const float quat[4])
{
    Struct_Pose_3D pose{};
    pose.pos[0] = pos[0];
    pose.pos[1] = pos[1];
    pose.pos[2] = pos[2];
    Alg_Quat_To_Mat3(quat, pose.R);
    Alg_Mat3_To_Quat(pose.R, pose.quat);
    Alg_Mat3_To_RPY(pose.R, pose.rpy);
    return pose;
}

#endif // ALG_MATH_3D_H

/************************ COPYRIGHT(C) ZLLC 2027 ******************************/
