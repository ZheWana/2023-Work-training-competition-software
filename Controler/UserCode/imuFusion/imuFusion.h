/**
 * @file imuFusion.h
 * @author ZheWana(zhewana.cn)
 * @brief imuFusion header file
 * @date 2022/4/20
 * @note Do not modified the code if you do not know what you are doing!!!
 * @refer MahonyAHRS
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
#ifndef IMUFUSION_H
#define IMUFUSION_H

////////////////////////  Fusion Begin Config  ////////////////////////

////////////////////         6 Axis Config         ////////////////////

// 仅使用6轴解算
#define Use_Only_6axis (0)

// 规定输入角速度的单位为弧度制（默认为角度制
#define Fusion_Use_Rad_Gyro (0)

// 设置加速度修正PI值
#define Accel_Kp (3.0f)
#define Accel_Ki (0.00f)

#if (!Use_Only_6axis)
////////////////////        ex3 Axis Config        ////////////////////

// 设置初始姿态 X轴/Y轴 指北
#define Fusion_Initial_X_North (0)
#define Fusion_Initial_Y_North (!Fusion_Initial_X_North)

// 设置磁力计修正PI值
#define Magnetic_Kp (5.0f) // 10.0f
#define Magnetic_Ki (0.00f)
#endif // (!Use_Only_6axis)
////////////////////////  Fusion End Config  //////////////////////////

#include "math.h"

typedef struct {
    float Ax, Ay, Az; // 加速度信息 - 用户输入
    float Gx, Gy, Gz; // 角速度信息 - 用户输入
#if (!Use_Only_6axis)
    float Mx, My, Mz; // 磁力计信息 - 用户输入
#endif

    float delta; // 解算周期

    float q0, q1, q2, q3; // 四元数，系统内部解算使用

    enum dataLock {
        Unlocked = 0,
        Locked
    } dataLock;
} fusion_t;

typedef struct {
    float Ax, Ay, Az; // 加速度静差信息 - 用户输入
    float Gx, Gy, Gz; // 角速度静差信息 - 用户输入
#if (!Use_Only_6axis)
    float Mx, My, Mz; // 磁力计静差信息 - 用户输入
#endif
} bias_t;

typedef enum FusionStatus {
    Fusion_OK = 0,
    Fusion_Error
} FusionStatus;

#define Fusion_DataAssignment_6axis(Ax, Ay, Az, Gx, Gy, Gz) do{ \
icmFusion.Ax = ax;\
icmFusion.Ay = ay;\
icmFusion.Az = az;\
icmFusion.Gx = gx;\
icmFusion.Gy = gy;\
icmFusion.Gz = gz;\
}while(0)

#define Fusion_DataAssignment_9axis(ax, ay, az, gx, gy, gz, mx, my, mz) do{ \
icmFusion.Ax = ax;\
icmFusion.Ay = ay;\
icmFusion.Az = az;\
icmFusion.Gx = gx;\
icmFusion.Gy = gy;\
icmFusion.Gz = gz;\
icmFusion.Mx = mx;\
icmFusion.My = my;\
icmFusion.Mz = mz;\
}while(0)

/**
 * @brief 数据初始化
 * @param fusionData 数据融合句柄
 * @param delta 算法调用周期
 * @note 请务必以固定的周期调用该函数
 */
void Fusion_Init(fusion_t* fusionData, float delta);

/**
 * @brief 6轴数据更新(不可重入)
 * @param fusionData 数据融合句柄
 * @param bias 算法调用周期
 * @return 数据异常：Fusion_Error
 *         一切正常：Fusion_OK
 * @pre Fusion_Init
 */
FusionStatus Fusion_DataUpdate_6axis(fusion_t* fusionData, bias_t* bias);

/**
 * @brief 9轴数据更新（不可重入）
 * @param fusionData    数据融合句柄
 * @param bias          算法调用周期
 * @return              数据异常：Fusion_Error
 *                      一切正常：Fusion_OK
 * @pre 调用Fusion_Init
 */
FusionStatus Fusion_DataUpdate_9axis(fusion_t* fusionData, bias_t* bias);

/**
 * @brief 获取欧拉角函数
 * @param fusionData 融合数据句柄
 * @param pitch      俯仰角指针
 * @param roll       横滚角指针
 * @param yaw        航向角指针
 * @return
 */
FusionStatus Fusion_GetEuler(fusion_t* fusionData, float* pitch, float* roll, float* yaw);

/**
 * @brief 获取四元数函数
 * @param fusionData 融合数据句柄
 * @param q0         第一个四元数指针
 * @param q1         第二个四元数指针
 * @param q2         第三个四元数指针
 * @param q3         第四个四元数指针
 * @return
 */
FusionStatus Fusion_GetQuaternion(fusion_t* fusionData, float* q0, float* q1, float* q2, float* q3);

#endif // IMUFUSION_H
