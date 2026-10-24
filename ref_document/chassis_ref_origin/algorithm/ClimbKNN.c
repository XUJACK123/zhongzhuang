/**
 * knn_impact_detector.c
 * 台阶撞击检测实现（聚类异常检测）
 */
#include "ClimbKNN.h"
#include "knn_impact_model.h"
#include <string.h>
#include <math.h>

/* 环形缓冲区，存储最近 IMPACT_WINDOW_SIZE 个时刻的 4 维特征 */
static float buffer[IMPACT_WINDOW_SIZE][IMPACT_N_FEATURES];
static int buffer_idx = 0;
static bool buffer_full = false;

void impact_detector_init(void) {
    buffer_idx = 0;
    buffer_full = false;
    memset(buffer, 0, sizeof(buffer));
}

void impact_detector_update(float body_vel, float wheel_avg_vel, float body_wheel_diff,
                            float wheel_diff_LR) {
    buffer[buffer_idx][0] = body_vel;
    buffer[buffer_idx][1] = wheel_avg_vel;
    buffer[buffer_idx][2] = body_wheel_diff;
    buffer[buffer_idx][3] = wheel_diff_LR;

    buffer_idx++;
    if (buffer_idx >= IMPACT_WINDOW_SIZE) {
        buffer_idx = 0;
        buffer_full = true;
    }
}

/**
 * 将环形缓冲区展开为按时间顺序的一维向量（最早时刻在前）
 */
static void build_query_vector(float *vec) {
    int start = buffer_full ? buffer_idx : 0;
    for (int i = 0; i < IMPACT_WINDOW_SIZE; i++) {
        int idx = (start + i) % IMPACT_WINDOW_SIZE;
        memcpy(&vec[i * IMPACT_N_FEATURES], buffer[idx], IMPACT_N_FEATURES * sizeof(float));
    }
}

float impact_get_confidence(void) {
    if (!buffer_full) return 1.0f;   // 数据不足，默认为正常

    /* 1. 构建查询向量并标准化 */
    float query[IMPACT_VEC_LEN];
    build_query_vector(query);
    for (int i = 0; i < IMPACT_VEC_LEN; i++) {
        query[i] = (query[i] - impact_norm_mean[i]) / impact_norm_std[i];
    }

    /* 2. 计算到各聚类中心的最近平方距离 */
    float min_dist_sq = 1e10f;
    for (int c = 0; c < IMPACT_N_CLUSTERS; c++) {
        float dist_sq = 0.0f;
        for (int j = 0; j < IMPACT_VEC_LEN; j++) {
            float diff = query[j] - impact_centroids[c][j];
            dist_sq += diff * diff;
        }
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
        }
    }

    /* 3. 将距离映射为置信度（正常→接近1，撞击→接近0） */
    float confidence = expf(-min_dist_sq / (IMPACT_DIST_THRESHOLD * 1.5f));
    if (confidence > 1.0f) confidence = 1.0f;
    if (confidence < 0.0f) confidence = 0.0f;
    return confidence;
}

bool impact_is_detected(void) {
    return impact_get_confidence() < 0.75f;
}

