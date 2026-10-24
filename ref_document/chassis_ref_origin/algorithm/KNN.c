#include "knn_model.h"   
#include <stdint.h>

/* ========== 内部函数 ========== */

/**
 * @brief 对原始状态向量进行 Z-score 标准化
 * @param raw   输入：原始 10 维状态向量
 * @param norm  输出：标准化后的 10 维向量
 */
static void normalize_state(const float *raw, float *norm) {
    for (int i = 0; i < KNN_FEATURES; i++) {
        norm[i] = (raw[i] - knn_mean[i]) / knn_std[i];
    }
}

/**
 * @brief 计算两个向量的欧氏距离平方
 */
static float sqr_dist(const float *a, const float *b, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float diff = a[i] - b[i];
        sum += diff * diff;
    }
    return sum;
}

/**
 * @brief 在已排序（升序）的缓冲区中插入新距离和标签
 */
static void insert_k_sorted(float *dist_buf, int *label_buf, int k,
                            float new_dist, int new_label) {
    int i;
    for (i = k - 1; i >= 0; i--) {
        if (i == 0 || new_dist >= dist_buf[i - 1]) {
            // 后移元素
            for (int j = k - 1; j > i; j--) {
                dist_buf[j]  = dist_buf[j - 1];
                label_buf[j] = label_buf[j - 1];
            }
            dist_buf[i]  = new_dist;
            label_buf[i] = new_label;
            break;
        }
    }
}

/* ========== 公开接口 ========== */

/**
 * @brief  根据原始 10 维状态计算离地置信度
 * @param  raw_state  输入：原始状态向量，顺序必须与训练数据列一致
 *                     即 [pitch, pitch_dot, theta, theta_dot, x, x_dot, F, Tp, ...(后两个)]
 * @return 置信度 (0.0 ~ 1.0)
 */
float knn_predict(const float *raw_state) {
    // 1. 标准化原始状态
    float norm_state[KNN_FEATURES];
    normalize_state(raw_state, norm_state);

    // 2. 维护 K 个最近邻
    float min_dist[KNN_K];
    int   min_label[KNN_K];
    for (int i = 0; i < KNN_K; i++) {
        min_dist[i]  = 1e18f;
        min_label[i] = 0;
    }

    for (int i = 0; i < KNN_SAMPLES; i++) {
        // 训练数据每行：前 KNN_FEATURES 列是归一化特征，最后一列是标签
        float dist = sqr_dist(norm_state, knn_train_data[i], KNN_FEATURES);
        int label  = (int)knn_train_data[i][KNN_FEATURES];  // 标签为 0 或 1

        if (dist < min_dist[KNN_K - 1]) {
            insert_k_sorted(min_dist, min_label, KNN_K, dist, label);
        }
    }

    // 3. 统计正样本个数，计算置信度
    int positive = 0;
    for (int i = 0; i < KNN_K; i++) {
        if (min_label[i] == 1) positive++;
    }
    return (float)positive / (float)KNN_K;
}

/* ========== 使用示例 ========== */
/*
// 在主循环或其他函数中调用：
void some_control_loop(void) {
    float raw_state[10] = {
        current_pitch, current_pitch_dot,
        current_theta,  current_theta_dot,
        current_x,      current_x_dot,
        current_F,      current_Tp,
        extra_feature1, extra_feature2
    };

    float confidence = knn_predict(raw_state);

    if (confidence >= THRESHOLD) {
        // 判断为离地
        liftoff_detected = 1;
    } else {
        liftoff_detected = 0;
    }
}
*/
