/**
 * @file imuFusion.c
 * @author ZheWana(zhewana.cn)
 * @brief imuFusion source file
 * @date 2022/4/20
 * @note Do not modified the code if you do not know what you are doing!!!
 *                  ___           ___           ___           ___           ___                       ___           ___
 *      ___        /\__\         /\__\         /\  \         /\__\         /\  \          ___        /\  \         /\__\
 *     /\  \      /::|  |       /:/  /        /::\  \       /:/  /        /::\  \        /\  \      /::\  \       /::|  |
 *     \:\  \    /:|:|  |      /:/  /        /:/\:\  \     /:/  /        /:/\ \  \       \:\  \    /:/\:\  \     /:|:|  |
 *     /::\__\  /:/|:|__|__   /:/  /  ___   /::\~\:\  \   /:/  /  ___   _\:\~\ \  \      /::\__\  /:/  \:\  \   /:/|:|  |__
 *  __/:/\/__/ /:/ |::::\__\ /:/__/  /\__\ /:/\:\ \:\__\ /:/__/  /\__\ /\ \:\ \ \__\  __/:/\/__/ /:/__/ \:\__\ /:/ |:| /\__\
 * /\/:/  /    \/__/~~/:/  / \:\  \ /:/  / \/__\:\ \/__/ \:\  \ /:/  / \:\ \:\ \/__/ /\/:/  /    \:\  \ /:/  / \/__|:|/:/  /
 * \::/__/           /:/  /   \:\  /:/  /       \:\__\    \:\  /:/  /   \:\ \:\__\   \::/__/      \:\  /:/  /      |:/:/  /
 *  \:\__\          /:/  /     \:\/:/  /         \/__/     \:\/:/  /     \:\/:/  /    \:\__\       \:\/:/  /       |::/  /
 *   \/__/         /:/  /       \::/  /                     \::/  /       \::/  /      \/__/        \::/  /        /:/  /
 *                 \/__/         \/__/                       \/__/         \/__/                     \/__/         \/__/
 */

#include "imuFusion.h"
#include "stddef.h"

#define Fusion_ToDig(rad) (rad * 57.295779513082320876798154814105) ///>角度转化为弧度
#define Fusion_ToRad(dig) (dig * 0.01745329251994329576923690768489) ///>弧度转化为角度

static void Fusion_DataCorrect(fusion_t* fusion, bias_t* bias)
{
    if (bias) {
        fusion->Ax += bias->Ax;
        fusion->Ay += bias->Ay;
        fusion->Az += bias->Az;
        fusion->Gx += bias->Gx;
        fusion->Gy += bias->Gy;
        fusion->Gz += bias->Gz;
        fusion->Mx += bias->Mx;
        fusion->My += bias->My;
        fusion->Mz += bias->Mz;
    }
}

static float inv_sqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = *((long*)&y);
    i = 0x5f375a86 - (i >> 1);
    y = *((float*)&i);
    y = y * (f - (x * y * y));
    return y;
}

void Fusion_Init(fusion_t* fusionData, float delta)
{
    fusionData->delta = delta;

    fusionData->q0 = 1;
    fusionData->q1 = 0;
    fusionData->q2 = 0;
    fusionData->q3 = 0;
}

FusionStatus Fusion_DataUpdate_6axis(fusion_t* fusionData, bias_t* bias)
{
    Fusion_DataCorrect(fusionData, bias);
    static float vx, vy, vz;
    static float aex, aey, aez;
    static float aexInt, aeyInt, aezInt;
    float akp = Accel_Kp, aki = Accel_Ki;
    float recipNorm;

    float q0q0 = fusionData->q0 * fusionData->q0;
    float q0q1 = fusionData->q0 * fusionData->q1;
    float q0q2 = fusionData->q0 * fusionData->q2;
    float q1q1 = fusionData->q1 * fusionData->q1;
    float q1q3 = fusionData->q1 * fusionData->q3;
    float q2q2 = fusionData->q2 * fusionData->q2;
    float q2q3 = fusionData->q2 * fusionData->q3;
    float q3q3 = fusionData->q3 * fusionData->q3;

#if (!Fusion_Use_Rad_Gyro)
    fusionData->Gx = Fusion_ToRad(fusionData->Gx);
    fusionData->Gy = Fusion_ToRad(fusionData->Gy);
    fusionData->Gz = Fusion_ToRad(fusionData->Gz);
#endif

    if (fusionData->Ax == 0 && fusionData->Ay == 0 && fusionData->Az == 0) { //加速度向量归一化
        return Fusion_Error;
    } else {
        recipNorm = inv_sqrt(
                fusionData->Ax * fusionData->Ax + fusionData->Ay * fusionData->Ay + fusionData->Az * fusionData->Az);
        fusionData->Ax *= recipNorm;
        fusionData->Ay *= recipNorm;
        fusionData->Az *= recipNorm;
    }

    //参考坐标系中的重力向量旋转到载体坐标系
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = 2.0f * (q0q0 + q3q3) - 1.0f; // q0q0 - q1q1 - q2q2 + q3q3;

    //估算向量与实际向量求外积，获得向量间的误差（此处所指误差正比于轴角
    aex = (fusionData->Ay * vz - fusionData->Az * vy);
    aey = (fusionData->Az * vx - fusionData->Ax * vz);
    aez = (fusionData->Ax * vy - fusionData->Ay * vx);

    //对误差做PI控制，用以调整陀螺仪数据
    aexInt += aex;
    aeyInt += aey;
    aezInt += aez;
    // 校准陀螺仪数据
    fusionData->Gx += (aex * akp + aexInt * aki);
    fusionData->Gy += (aey * akp + aeyInt * aki);
    fusionData->Gz += (aez * akp + aezInt * aki);

    //一阶龙格库塔法更新四元数
    fusionData->dataLock = Locked;
    fusionData->q0 += 0.5f * (-fusionData->q1 * fusionData->Gx - fusionData->q2 * fusionData->Gy - fusionData->q3 * fusionData->Gz) * fusionData->delta;
    fusionData->q1 += 0.5f * (fusionData->q0 * fusionData->Gx + fusionData->q2 * fusionData->Gz - fusionData->q3 * fusionData->Gy) * fusionData->delta;
    fusionData->q2 += 0.5f * (fusionData->q0 * fusionData->Gy - fusionData->q1 * fusionData->Gz + fusionData->q3 * fusionData->Gx) * fusionData->delta;
    fusionData->q3 += 0.5f * (fusionData->q0 * fusionData->Gz + fusionData->q1 * fusionData->Gy - fusionData->q2 * fusionData->Gx) * fusionData->delta;

    //四元数归一化
    recipNorm = inv_sqrt(q0q0 + q1q1 + q2q2 + q3q3);
    fusionData->q0 *= recipNorm;
    fusionData->q1 *= recipNorm;
    fusionData->q2 *= recipNorm;
    fusionData->q3 *= recipNorm;
    fusionData->dataLock = Unlocked;

    return Fusion_OK;
}

FusionStatus Fusion_DataUpdate_9axis(fusion_t* fusionData, bias_t* bias)
{
    Fusion_DataCorrect(fusionData, bias);
    static float vx, vy, vz;
    static float aex, aey, aez;
    static float mex, mey, mez;
    static float aexInt, aeyInt, aezInt;
    static float mexInt, meyInt, mezInt;
    float hx, hy; // 旋转到参考坐标系的机体磁场
    float bz; // 参考坐标系的地磁场
    float wx, wy, wz; // 误差向量
    float akp = Accel_Kp, aki = Accel_Ki;
    float mkp = Magnetic_Kp, mki = Magnetic_Ki; // 170 0.05f
    float recipNorm;

    float q0q0 = fusionData->q0 * fusionData->q0;
    float q0q1 = fusionData->q0 * fusionData->q1;
    float q0q2 = fusionData->q0 * fusionData->q2;
    float q0q3 = fusionData->q0 * fusionData->q3;
    float q1q1 = fusionData->q1 * fusionData->q1;
    float q1q2 = fusionData->q1 * fusionData->q2;
    float q1q3 = fusionData->q1 * fusionData->q3;
    float q2q2 = fusionData->q2 * fusionData->q2;
    float q2q3 = fusionData->q2 * fusionData->q3;
    float q3q3 = fusionData->q3 * fusionData->q3;

#if (!Fusion_Use_Rad_Gyro)
    fusionData->Gx = Fusion_ToRad(fusionData->Gx);
    fusionData->Gy = Fusion_ToRad(fusionData->Gy);
    fusionData->Gz = Fusion_ToRad(fusionData->Gz);
#endif

    if (fusionData->Ax == 0 && fusionData->Ay == 0 && fusionData->Az == 0) { //加速度向量归一化
        return Fusion_Error;
    } else {
        recipNorm = inv_sqrt(
                fusionData->Ax * fusionData->Ax + fusionData->Ay * fusionData->Ay + fusionData->Az * fusionData->Az);
        fusionData->Ax *= recipNorm;
        fusionData->Ay *= recipNorm;
        fusionData->Az *= recipNorm;
    }

    //参考坐标系中的重力向量旋转到载体坐标系
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = 2.0f * (q0q0 + q3q3) - 1.0f;

    //估算向量与实际向量求外积，获得向量间的误差（此处所指误差正比于轴角
    aex = (fusionData->Ay * vz - fusionData->Az * vy);
    aey = (fusionData->Az * vx - fusionData->Ax * vz);
    aez = (fusionData->Ax * vy - fusionData->Ay * vx);

    //对误差做PI控制，用以调整陀螺仪数据
    aexInt += aex;
    aeyInt += aey;
    aezInt += aez;

    //磁力计矫正
    if (fusionData->Mx != 0 && fusionData->My != 0 && fusionData->Mz != 0) {

        recipNorm = inv_sqrt(fusionData->Mx * fusionData->Mx + fusionData->My * fusionData->My + fusionData->Mz * fusionData->Mz);
        fusionData->Mx *= recipNorm;
        fusionData->My *= recipNorm;
        fusionData->Mz *= recipNorm;

        hx = 2.0f * (fusionData->Mx * (0.5f - q2q2 - q3q3) + fusionData->My * (q1q2 - q0q3) + fusionData->Mz * (q1q3 + q0q2));
        hy = 2.0f * (fusionData->Mx * (q1q2 + q0q3) + fusionData->My * (0.5f - q1q1 - q3q3) + fusionData->Mz * (q2q3 - q0q1));

#if Fusion_Initial_X_North
        float bx = sqrtf(hx * hx + hy * hy);
#endif

#if Fusion_Initial_Y_North
        float by = sqrtf(hx * hx + hy * hy);
#endif
        bz = 2.0f * (fusionData->Mx * (q1q3 - q0q2) + // bz = hz
                     fusionData->My * (q2q3 + q0q1) + fusionData->Mz * (0.5f - q1q1 - q2q2));

        wx = 2.0f * (bz * (q1q3 - q0q2) +
                     #if Fusion_Initial_X_North
                     bx * (0.5f - q2q2 - q3q3));
                     #else
                     by * (q1q2 + q0q3));
#endif
        wy = 2.0f * (bz * (q0q1 + q2q3) +
                     #if Fusion_Initial_X_North
                     bx * (q1q2 - q0q3));
                     #else
                     by * (0.5f - q1q1 - q3q3));
#endif // Fusion_Initial_X_North
        wz = 2.0f * (bz * (0.5f - q1q1 - q2q2) +
                     #if Fusion_Initial_X_North
                     bx * (q0q2 + q1q3));
                     #else
                     by * (q2q3 - q0q1));
#endif // Fusion_Initial_X_North

        mex = (fusionData->My * wz - fusionData->Mz * wy);
        mey = (fusionData->Mz * wx - fusionData->Mx * wz);
        mez = (fusionData->Mx * wy - fusionData->My * wx);

        //对误差做PI控制，用以调整陀螺仪数据
        mexInt += mex;
        meyInt += mey;
        mezInt += mez;
    }

    // 校准陀螺仪数据
    fusionData->Gx += (aex * akp + aexInt * aki) + (mex * mkp + mexInt * mki);
    fusionData->Gy += (aey * akp + aeyInt * aki) + (mey * mkp + meyInt * mki);
    fusionData->Gz += (aez * akp + aezInt * aki) + (mez * mkp + mezInt * mki);

    //一阶龙格库塔法更新四元数
    fusionData->dataLock = Locked;
    fusionData->q0 += 0.5f * (-fusionData->q1 * fusionData->Gx - fusionData->q2 * fusionData->Gy - fusionData->q3 * fusionData->Gz) * fusionData->delta;
    fusionData->q1 += 0.5f * (fusionData->q0 * fusionData->Gx + fusionData->q2 * fusionData->Gz - fusionData->q3 * fusionData->Gy) * fusionData->delta;
    fusionData->q2 += 0.5f * (fusionData->q0 * fusionData->Gy - fusionData->q1 * fusionData->Gz + fusionData->q3 * fusionData->Gx) * fusionData->delta;
    fusionData->q3 += 0.5f * (fusionData->q0 * fusionData->Gz + fusionData->q1 * fusionData->Gy - fusionData->q2 * fusionData->Gx) * fusionData->delta;

    //四元数归一化
    recipNorm = inv_sqrt(q0q0 + q1q1 + q2q2 + q3q3);
    fusionData->q0 *= recipNorm;
    fusionData->q1 *= recipNorm;
    fusionData->q2 *= recipNorm;
    fusionData->q3 *= recipNorm;
    fusionData->dataLock = Unlocked;

    return Fusion_OK;
}

FusionStatus Fusion_GetEuler(fusion_t* fusionData, float* pitch, float* roll, float* yaw)
{
    if (fusionData->dataLock == Locked) {
        return Fusion_Error;
    }
    if (fusionData->q0 == 0 && fusionData->q1 == 0 && fusionData->q2 == 0 && fusionData->q3 == 0) {
        return Fusion_Error;
    }

    float q1q1 = fusionData->q1 * fusionData->q1;
    float q2q2 = fusionData->q2 * fusionData->q2;

    if (pitch != NULL) {
        *pitch = Fusion_ToDig(asinf(-2.0f * fusionData->q1 * fusionData->q3 + 2.0f * fusionData->q0 * fusionData->q2));
    }
    if (roll != NULL) {
        *roll = Fusion_ToDig(atan2f(2.0f * fusionData->q2 * fusionData->q3 + 2.0f * fusionData->q0 * fusionData->q1,
                                    -2.0f * q1q1 - 2.0f * q2q2 + 1));
    }
    if (yaw != NULL) {
        *yaw = Fusion_ToDig(atan2f(2.0f * (fusionData->q1 * fusionData->q2 + fusionData->q0 * fusionData->q3),
                                   fusionData->q0 * fusionData->q0 + q1q1 - q2q2 - fusionData->q3 * fusionData->q3));
    }
    return Fusion_OK;
}

FusionStatus Fusion_GetQuaternion(fusion_t* fusionData, float* q0, float* q1, float* q2, float* q3)
{
    if (fusionData->dataLock == Locked) {
        return Fusion_Error;
    }
    if (q0 == NULL || q1 == NULL || q2 == NULL || q3 == NULL) {
        return Fusion_Error;
    } else {
        *q0 = fusionData->q0;
        *q1 = fusionData->q1;
        *q2 = fusionData->q2;
        *q3 = fusionData->q3;
    }

    return Fusion_OK;
}