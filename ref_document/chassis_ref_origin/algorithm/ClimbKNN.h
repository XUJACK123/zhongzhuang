/**
 * knn_impact_detector.h
 * 基于正常状态聚类的台阶撞击异常检测
 * 特征: v_body, v_wheel_avg, diff_body_wheel, diff_LR, tau_L, tau_R
 */
#ifndef KNN_IMPACT_DETECTOR_H
#define KNN_IMPACT_DETECTOR_H

#include <stdbool.h>

/**
 * 初始化检测器，清空历史缓冲区
 */
void impact_detector_init(void);

/**
 * 输入当前时刻的6维特征
 * @param body_vel          机体速度
 * @param wheel_avg_vel     轮子平均速度
 * @param body_wheel_diff   机体轮子速度差
 * @param wheel_diff_LR     左右轮速度差
 */
void impact_detector_update(float body_vel, float wheel_avg_vel, float body_wheel_diff,
                            float wheel_diff_LR);

/**
 * 获取当前撞击置信度 (0.0 ~ 1.0)
 * 越接近 1.0 说明越正常，越接近 0.0 说明越像撞击
 * 缓冲区未满时返回 1.0（认为正常）
 */
float impact_get_confidence(void);

/**
 * 判断是否发生撞击（置信度 < 0.75）
 */
bool impact_is_detected(void);

#endif
