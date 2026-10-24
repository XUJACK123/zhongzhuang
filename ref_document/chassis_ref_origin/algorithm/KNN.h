#ifndef KNN_H
#define KNN_H

/* ========== 用户可调参数 ========== */
#define KNN_K               5        // 近邻数，推荐奇数
#define THRESHOLD       0.75f    // 离地置信度阈值

float knn_predict(const float *raw_state) ;

#endif
