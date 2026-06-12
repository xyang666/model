#pragma once

/**
 * FilterTypes.h
 * --------------
 * 滤波器相关定义：滤波器状态、滤波器类型枚举。
 */

// 滤波器生命周期状态
#define TM_STATE_UNINITIALIZED 0 /**< 尚未初始化 */
#define TM_STATE_INITIALIZING  1 /**< 初始化中 */
#define TM_STATE_TRACKING      2 /**< 正常跟踪中 */
#define TM_STATE_COASTING      3 /**< 目标丢失，惯性外推中 */

// 滤波器类型
#define TM_FILTER_ALPHA_BETA         0 /**< Alpha-Beta 滤波器（匀速模型） */
#define TM_FILTER_ALPHA_BETA_GAMMA   1 /**< Alpha-Beta-Gamma 滤波器（匀加速模型） */
#define TM_FILTER_KALMAN             2 /**< Kalman 滤波器（匀速模型） */
#define TM_FILTER_EKF                3 /**< 扩展 Kalman 滤波器（非线性模型） */
#define TM_FILTER_UKF                4 /**< 无迹 Kalman 滤波器（非线性模型） */
#define TM_FILTER_SLIDING_WINDOW     5 /**< 滑动窗口多项式拟合（无模型） */
#define TM_FILTER_DEFAULT            TM_FILTER_KALMAN /**< 默认滤波器类型 */
