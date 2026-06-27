//
// Created by Pazfic on 2025/4/30.
//

#ifndef MODELCONFIG_H
#define MODELCONFIG_H
// Based On Model: P_predict = m_0*torq*rpm + m_1*abs(rpm) + m_2*torq*torq + m_3


//2.1参数改进
// #define m_0_3508  0.007578f
// #define m_1_3508  0.000104f
// #define m_2_3508  0.058200f
// #define m_3_3508  2.944601f
//
// #define m_0_6020  0.181430f
// #define m_1_6020  (-0.014875f)
// #define m_2_6020  1.481659f
// #define m_3_6020  2.980139f

//2.4数据,；烂完了，正常跑会扣血，抵住也会
// #define m_0_3508  0.005179f
// #define m_1_3508  0.000457f
// #define m_2_3508  1.082212f
// #define m_3_3508  (-2.162007f)
//
// #define m_0_6020  0.176452f
// #define m_1_6020  (-0.008318f)
// #define m_2_6020  1.652280f
// #define m_3_6020  3.219375f

//2.1+2.3数据拟合，效果一般，抵住不会超，正常画圈会扣血，在某些急转情况会控不住
// #define m_0_3508  0.006643f
// #define m_1_3508  (-0.000661f)
// #define m_2_3508  0.984687f
// #define m_3_3508  1.152662f
//
// #define m_0_6020  0.173908f
// #define m_1_6020  (-0.013209f)
// #define m_2_6020  1.338407f
// #define m_3_6020  3.197136f

//2.3参数，效果一般，抵住不会超功率，不扣血，急转情况会控不住扣血没解决，正常跑会扣血
// #define m_0_3508  0.006339f
// #define m_1_3508  (-0.000709f)
// #define m_2_3508  1.189082f
// #define m_3_3508  0.094376f
//
// #define m_0_6020  0.151264f
// #define m_1_6020  (-0.009118f)
// #define m_2_6020  0.238449f
// #define m_3_6020  3.380515f

//TODO:联盟赛使用，可上场
//2.1参数，效果尚可，会超几W，正常跑不会扣血，但是在某些急转情况会控不住，抵住会扣血
#define m_0_3508  0.006693f
#define m_1_3508  0.000472f
#define m_2_3508  0.628765f
#define m_3_3508  0.123292f

#define m_0_6020  0.181430f
#define m_1_6020  (-0.014875f)
#define m_2_6020  1.481659f
#define m_3_6020  2.980139f

//1.31参数
// #define m_0_3508  0.006660f
// #define m_1_3508  0.001522f
// #define m_2_3508  0.267923f
// #define m_3_3508  (-0.159139f)

// #define m_0_6020  0.0950f
// #define m_1_6020  0.0003f
// #define m_2_6020  8.3473f
// #define m_3_6020  0.1410f

#endif
